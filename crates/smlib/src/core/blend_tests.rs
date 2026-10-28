//! Fillets and chamfers on solidcore, against closed forms: every result exports to a record that
//! validates. With SMLib built, the same case on the C++ kernel gives the same volume, counts and
//! faces by surface kind, or the same error (parity only: the expected values are the analytic
//! ones).
//!
//! The bodies are solidcore's own boxes and cylinders: a record's curved surfaces do not come
//! into solidcore yet (solidcore#1432, #1433).

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::sync::mpsc;
use std::time::Duration;

use super::blend::{Corners, blend, chamfer_sides, fillet_variable};
use super::{Body, counts, make_box, make_cylinder, to_arrays, volume};
use crate::export::native;
use crate::{Blend, ChamferSecond, Error};

/// A case longer than this is a hang.
const TIMEOUT: Duration = Duration::from_secs(300);

/// How near a pick point must be.
const TOL: f64 = 0.01;

#[derive(Clone, Copy, Debug)]
enum Shape {
    Box([f64; 3], [f64; 3]),
    Cylinder([f64; 3], f64, f64),
}

impl Shape {
    fn core(self) -> Body {
        match self {
            Shape::Box(o, s) => make_box(o, s),
            Shape::Cylinder(o, r, h) => make_cylinder(o, r, h),
        }
        .unwrap()
    }
}

#[derive(Clone, Debug)]
enum Op {
    /// `Brep::blend` of one kind, one size for every edge.
    Blend(Shape, Blend, Vec<[f64; 3]>, f64),
    /// `Brep::chamfer_sides`.
    Sides(Shape, Vec<[f64; 3]>, f64, ChamferSecond, [f64; 3]),
    /// `Brep::blend_setback` of rounds: edges, radius, corners, setbacks.
    Setback(Shape, Vec<[f64; 3]>, f64, Vec<[f64; 3]>, Vec<f64>),
    /// `Brep::fillet_variable`: edge point, start, degree, knots, radii.
    Variable(Shape, [f64; 3], [f64; 3], usize, Vec<f64>, Vec<f64>),
}

/// What a case must give.
#[derive(Clone, Debug)]
enum Expect {
    /// A body of this volume (relative 1e-9).
    Volume(f64),
    /// A body of this volume to this relative accuracy.
    Near(f64, f64),
    /// A body whose volume lies strictly between these.
    Between(f64, f64),
    /// This error.
    Fails(Error),
}

struct Case {
    op: Op,
    expect: Expect,
}

/// A 30 x 20 x 10 block from the origin.
const BLOCK: Shape = Shape::Box([0.0; 3], [30.0, 20.0, 10.0]);
const ABC: f64 = 6000.0;
/// Its top front edge, along x at y = 0, z = 10, 30 long.
const TOP_FRONT: [f64; 3] = [15.0, 0.0, 10.0];

/// The cross-section a round of radius r takes from a right-angled edge: (1 - π/4) r².
fn spandrel(r: f64) -> f64 {
    (1.0 - PI / 4.0) * r * r
}

fn case(name: &str) -> Case {
    let c = |op, expect| Case { op, expect };
    let r = 2.0;
    match name {
        // abc - (1 - π/4) r² L.
        "fillet_one_edge" => c(Op::Blend(BLOCK, Blend::Round, vec![TOP_FRONT], r), Expect::Volume(ABC - spandrel(r) * 30.0)),
        // Every edge rounded at r, the corners spheres: (a-2r)(b-2r)(c-2r) + 2r Σ(faces less
        // 2r) + πr² Σ(edges less 2r) + 4/3 πr³.
        "fillet_all_edges" => {
            let (a, b, h) = (30.0 - 2.0 * r, 20.0 - 2.0 * r, 10.0 - 2.0 * r);
            let v = a * b * h + 2.0 * r * (a * b + b * h + h * a) + PI * r * r * (a + b + h) + 4.0 / 3.0 * PI * r * r * r;
            let mut edges = Vec::new();
            for (y, z) in [(0.0, 0.0), (20.0, 0.0), (0.0, 10.0), (20.0, 10.0)] {
                edges.push([15.0, y, z]);
            }
            for (x, z) in [(0.0, 0.0), (30.0, 0.0), (0.0, 10.0), (30.0, 10.0)] {
                edges.push([x, 10.0, z]);
            }
            for (x, y) in [(0.0, 0.0), (30.0, 0.0), (0.0, 20.0), (30.0, 20.0)] {
                edges.push([x, y, 5.0]);
            }
            c(Op::Blend(BLOCK, Blend::Round, edges, r), Expect::Volume(v))
        }
        // abc - d² L / 2.
        "chamfer_one_edge" => c(Op::Blend(BLOCK, Blend::Chamfer, vec![TOP_FRONT], r), Expect::Volume(ABC - r * r * 30.0 / 2.0)),
        // abc - d1 d2 L / 2: 2 back along the top (the face leaning most along +z), 3 down the
        // front.
        "chamfer_two_distances" => {
            c(Op::Sides(BLOCK, vec![TOP_FRONT], 2.0, ChamferSecond::Distance(3.0), [0.0, 0.0, 1.0]), Expect::Volume(ABC - 2.0 * 3.0 * 30.0 / 2.0))
        }
        // 2 back along the top, at 30° to it: d tan 30° down the front.
        "chamfer_distance_angle" => c(
            Op::Sides(BLOCK, vec![TOP_FRONT], 2.0, ChamferSecond::Angle(PI / 6.0), [0.0, 0.0, 1.0]),
            Expect::Volume(ABC - 2.0 * 2.0 * (PI / 6.0).tan() * 30.0 / 2.0),
        ),
        // A 5 mm cylinder 10 high, its top rim rounded at 1. Pappus: the spandrel (1 - π/4) r²,
        // its centroid (10 - 3π)/(12 - 3π) r in from the rim, swept round the axis.
        "fillet_cylinder_rim" => {
            let (big, h, r) = (5.0, 10.0, 1.0);
            let inset = (10.0 - 3.0 * PI) / (12.0 - 3.0 * PI) * r;
            c(
                Op::Blend(Shape::Cylinder([0.0; 3], big, h), Blend::Round, vec![[0.0, big, h]], r),
                Expect::Volume(PI * big * big * h - 2.0 * PI * (big - inset) * spandrel(r)),
            )
        }
        // The three edges at the corner (30, 20, 10) rounded at r, the corner set back 4 along
        // each. Without a setback the corner is a sphere's octant, and the block keeps
        // abc - (1 - π/4) r² Σ(L - r) - (1 - π/6) r³; the setback changes only what lies within
        // s + r of the corner, a cube of side s + r.
        "setback_corner" => {
            let s = 4.0;
            let plain = ABC - spandrel(r) * (30.0 + 20.0 + 10.0 - 3.0 * r) - (1.0 - PI / 6.0) * r * r * r;
            let near = (s + r) * (s + r) * (s + r);
            c(
                Op::Setback(BLOCK, vec![[15.0, 20.0, 10.0], [30.0, 10.0, 10.0], [30.0, 20.0, 5.0]], r, vec![[30.0, 20.0, 10.0]], vec![s]),
                Expect::Between(plain - near, plain + near),
            )
        }
        // The top front edge rounded from 1 at x = 0 to 3 at x = 30, linearly, each section
        // square to the edge a quarter circle: abc - (1 - π/4) ∫ r(x)² dx
        // = abc - (1 - π/4) L (r0² + r0 r1 + r1²) / 3. The round is a B-spline surface fitted to
        // those sections, which both kernels build 2.5e-7 short of it (and within 1e-9 of each
        // other), within the solver's fitting tolerance.
        "fillet_variable_linear" => {
            let (r0, r1, l) = (1.0, 3.0, 30.0);
            c(
                Op::Variable(BLOCK, TOP_FRONT, [0.0, 0.0, 10.0], 1, vec![0.0, 0.0, 1.0, 1.0], vec![r0, r1]),
                Expect::Near(ABC - (1.0 - PI / 4.0) * l * (r0 * r0 + r0 * r1 + r1 * r1) / 3.0, 1e-6),
            )
        }
        // A round of 12 on a 10 mm cube's top front edge is larger than either face beside it:
        // the fork builds it over the cliffs, on the faces' continuations, and cuts away all of
        // the cube outside the round's cylinder (axis y = 12, z = -2, radius 12). What stays is
        // 10 times the square's part inside that circle, ∫ (√(144 - u²) - 2) du over
        // u = y - 12 from -√140 to -2: 72 (asin(√140/12) - asin(1/6)) - 2 (√140 - 2).
        "fillet_over_a_cliff" => {
            let w = 140f64.sqrt();
            let area = 72.0 * ((w / 12.0).asin() - (1.0f64 / 6.0).asin()) - 2.0 * (w - 2.0);
            c(Op::Blend(Shape::Box([0.0; 3], [10.0; 3]), Blend::Round, vec![[5.0, 0.0, 10.0]], 12.0), Expect::Volume(10.0 * area))
        }
        // A round of 6 on the top rim of a cylinder of radius 5 is wider than the cylinder.
        "fillet_too_large" => {
            c(Op::Blend(Shape::Cylinder([0.0; 3], 5.0, 10.0), Blend::Round, vec![[0.0, 5.0, 10.0]], 6.0), Expect::Fails(Error::NoRoom))
        }
        "pick_near_nothing" => {
            c(Op::Blend(BLOCK, Blend::Round, vec![TOP_FRONT, [100.0, 100.0, 100.0]], r), Expect::Fails(Error::NoMatch { point: 1 }))
        }
        // √3 from all three edges at the corner (0, 0, 10).
        "pick_ambiguous" => c(Op::Blend(BLOCK, Blend::Round, vec![[-1.0, -1.0, 11.0]], r), Expect::Fails(Error::Ambiguous { point: 0 })),
        _ => panic!("{name}"),
    }
}

/// The tolerance a case picks with: the ambiguous point lies √3 from its edges.
fn tolerance(name: &str) -> f64 {
    if name == "pick_ambiguous" { 5.0 } else { TOL }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[derive(Debug, PartialEq)]
struct Measured {
    volume: f64,
    counts: [usize; 3],
    kinds: BTreeMap<String, usize>,
}

fn kinds(rec: &solidcraft_geom::brep::Brep) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    for f in &rec.faces {
        let v = serde_json::to_value(&f.surface).unwrap();
        *out.entry(v["kind"].as_str().unwrap_or("?").to_owned()).or_default() += 1;
    }
    out
}

fn on_core(name: &str, c: &Case) -> crate::Result<Body> {
    let tol = tolerance(name);
    match &c.op {
        Op::Blend(s, kind, points, size) => blend(&s.core(), *kind, points, &[*size], tol, &[], &[], Corners::Filled),
        Op::Sides(s, points, d, second, first) => chamfer_sides(&s.core(), points, tol, *d, *second, *first),
        Op::Setback(s, points, size, corners, setbacks) => {
            blend(&s.core(), Blend::Round, points, &vec![*size; points.len()], tol, corners, setbacks, Corners::Filled)
        }
        Op::Variable(s, point, start, degree, knots, radii) => fillet_variable(&s.core(), *point, tol, *start, *degree, knots, radii),
    }
}

/// Runs `f` on its own thread: a hang fails the test instead of stopping the run.
fn timed<T: Send + 'static>(name: &str, f: impl FnOnce() -> T + Send + 'static) -> T {
    let (tx, rx) = mpsc::channel();
    let worker = std::thread::spawn(move || {
        let _ = tx.send(f());
    });
    match rx.recv_timeout(TIMEOUT) {
        Ok(v) => v,
        Err(mpsc::RecvTimeoutError::Disconnected) => match worker.join() {
            Err(p) => std::panic::resume_unwind(p),
            Ok(()) => panic!("{name}: no result"),
        },
        Err(mpsc::RecvTimeoutError::Timeout) => panic!("{name}: solidcore hung (no result after {TIMEOUT:?})"),
    }
}

type Got = std::result::Result<Measured, Error>;

/// Checks a kernel's result against the case's closed form; what it measured, or its error.
fn judge(name: &str, who: &str, expect: &Expect, got: Got) -> Got {
    match (expect, &got) {
        (Expect::Fails(e), Err(g)) => assert_eq!(g, e, "{name}: {who} failed otherwise"),
        (Expect::Fails(e), Ok(m)) => panic!("{name}: {who} gave {m:?}, expected {e:?}"),
        (_, Err(g)) => panic!("{name}: {who} failed: {g}"),
        (Expect::Volume(v), Ok(m)) => assert!(rel(m.volume, *v) < 1e-9, "{name}: {who} volume {} vs analytic {v}", m.volume),
        (Expect::Near(v, within), Ok(m)) => assert!(rel(m.volume, *v) < *within, "{name}: {who} volume {} vs analytic {v}", m.volume),
        (Expect::Between(lo, hi), Ok(m)) => assert!(*lo < m.volume && m.volume < *hi, "{name}: {who} volume {} not in ({lo}, {hi})", m.volume),
    }
    got
}

fn solidcore_case(name: &'static str) -> Got {
    timed(name, move || {
        let c = case(name);
        let got = on_core(name, &c).map(|body| {
            let v = volume(&body, 1e-8).unwrap_or_else(|e| panic!("{name}: volume: {e}"));
            let n = counts(&body).unwrap();
            let (rec, _) = native(&to_arrays(&body).unwrap()).unwrap_or_else(|e| panic!("{name}: record: {e:?}"));
            rec.validate().unwrap_or_else(|e| panic!("{name}: record: {e}"));
            Measured { volume: v, counts: n, kinds: kinds(&rec) }
        });
        judge(name, "solidcore", &c.expect, got)
    })
}

/// The case on the C++ kernel, called directly (whatever the routing).
#[cfg(feature = "native")]
fn cpp_case(name: &str) -> Got {
    use crate::ops::{pick_error, setback_error};
    use crate::{Brep, ffi};
    crate::init().unwrap();
    let make = |s: Shape| -> Brep {
        match s {
            Shape::Box(o, z) => ffi::make_box(&o, &z),
            Shape::Cylinder(o, r, h) => ffi::make_cylinder(&o, r, h),
        }
        .map(Brep::made)
        .unwrap()
    };
    let kind = |k| match k {
        Blend::Round => ffi::ops::BLEND_ROUND,
        Blend::Curvature => ffi::ops::BLEND_CURVATURE,
        Blend::Chord => ffi::ops::BLEND_CHORD,
        Blend::Chamfer => ffi::ops::BLEND_CHAMFER,
    };
    let c = case(name);
    let tol = tolerance(name);
    let r = match &c.op {
        Op::Blend(s, k, points, size) => {
            ffi::ops::blend(kind(*k), make(*s).cpp().unwrap(), points, &vec![*size; points.len()], tol).map_err(pick_error)
        }
        Op::Sides(s, points, d, second, first) => {
            let (value, angle) = match second {
                ChamferSecond::Distance(d) => (*d, false),
                ChamferSecond::Angle(a) => (*a, true),
            };
            ffi::ops::chamfer_sides(make(*s).cpp().unwrap(), points, tol, *d, value, angle, first).map_err(pick_error)
        }
        Op::Setback(s, points, size, corners, setbacks) => {
            ffi::ops::blend_setback(ffi::ops::BLEND_ROUND, make(*s).cpp().unwrap(), points, &vec![*size; points.len()], corners, setbacks, tol)
                .map_err(setback_error)
        }
        Op::Variable(s, point, start, degree, knots, radii) => {
            ffi::ops::fillet_variable(make(*s).cpp().unwrap(), point, tol, start, i32::try_from(*degree).unwrap(), knots, radii).map_err(
                |s| match s {
                    ffi::ops::ERR_PICK_NONE => Error::NoMatch { point: 0 },
                    ffi::ops::ERR_PICK_AMBIGUOUS => Error::Ambiguous { point: 0 },
                    s => crate::error(s),
                },
            )
        }
    };
    let got = r.map(|h| {
        let v = ffi::volume(&h, 1e-8).unwrap();
        let n = ffi::counts(&h).unwrap().map(|x| usize::try_from(x).unwrap());
        let b = Brep::made(h);
        let (rec, _) = native(&b.cpp_arrays().unwrap()).unwrap();
        rec.validate().unwrap();
        Measured { volume: v, counts: n, kinds: kinds(&rec) }
    });
    judge(name, "C++", &c.expect, got)
}

/// solidcore and C++ agree: volume within 1e-9, the same counts and faces by kind, or the same
/// error.
#[cfg(feature = "native")]
fn parity(name: &'static str) {
    let cpp = cpp_case(name);
    let ours = solidcore_case(name);
    match (ours, cpp) {
        (Ok(a), Ok(b)) => {
            assert!(rel(a.volume, b.volume) < 1e-9, "{name}: solidcore {} vs C++ {}", a.volume, b.volume);
            assert_eq!(a.counts, b.counts, "{name}: face, edge, vertex counts (solidcore vs C++)");
            assert_eq!(a.kinds, b.kinds, "{name}: faces by surface kind (solidcore vs C++)");
        }
        (a, b) => assert_eq!(a.err(), b.err(), "{name}: errors (solidcore vs C++)"),
    }
}

macro_rules! cases {
    ($($m:ident $(: $ign:meta)?),* $(,)?) => {$(
        mod $m {
            #[test]
            $(#[$ign])?
            fn solidcore_matches_the_closed_form() {
                let _ = super::solidcore_case(stringify!($m));
            }
            #[cfg(feature = "native")]
            #[test]
            $(#[$ign])?
            fn solidcore_matches_the_cpp_kernel() {
                super::parity(stringify!($m));
            }
            #[cfg(feature = "native")]
            #[test]
            fn the_cpp_kernel_matches_the_closed_form() {
                let _ = super::cpp_case(stringify!($m));
            }
        }
    )*};
}

cases!(
    fillet_one_edge,
    fillet_all_edges,
    chamfer_one_edge,
    chamfer_two_distances,
    chamfer_distance_angle,
    fillet_cylinder_rim,
    setback_corner,
    fillet_variable_linear,
    fillet_over_a_cliff: ignore = "solidcore#960: no cliff overflow (fork 8ba4667)",
    fillet_too_large,
    pick_near_nothing,
    pick_ambiguous,
);

/// What solidcore does not have yet is refused as not supported, after the capi's own checks.
#[test]
fn fork_only_blends_are_not_supported_yet() {
    let r = timed("unsupported", || {
        let b = BLOCK.core();
        let curvature = blend(&b, Blend::Curvature, &[TOP_FRONT], &[2.0], TOL, &[], &[], Corners::Filled).err();
        let corner = [[15.0, 20.0, 10.0], [30.0, 10.0, 10.0], [30.0, 20.0, 5.0]];
        let open = blend(&b, Blend::Round, &corner, &[2.0; 3], TOL, &[[30.0, 20.0, 10.0]], &[4.0], Corners::Open).err();
        // 1 to 2 and back to 1: a corner at the middle.
        let bent = fillet_variable(&b, TOP_FRONT, TOL, [0.0, 0.0, 10.0], 1, &[0.0, 0.0, 0.5, 1.0, 1.0], &[1.0, 2.0, 1.0]).err();
        // A round over a cliff (fork 8ba4667): solidcore refuses every rail leaving its face.
        let cliff = blend(&Shape::Box([0.0; 3], [10.0; 3]).core(), Blend::Round, &[[5.0, 0.0, 10.0]], &[12.0], TOL, &[], &[], Corners::Filled).err();
        assert_eq!(cliff, Some(Error::Unsupported));
        // A pick that fails still says so first.
        let far = chamfer_sides(&b, &[[100.0; 3]], TOL, 2.0, ChamferSecond::Distance(3.0), [0.0, 0.0, 1.0]).err();
        (curvature, open, bent, far)
    });
    assert_eq!(r.0, Some(Error::Unsupported));
    assert_eq!(r.1, Some(Error::Unsupported));
    assert_eq!(r.2, Some(Error::Unsupported));
    assert_eq!(r.3, Some(Error::NoMatch { point: 0 }));
}

/// The capi's argument checks, before any kernel call.
#[test]
fn hostile_arguments_are_refused() {
    let b = BLOCK.core();
    let none = Corners::Filled;
    assert_eq!(blend(&b, Blend::Round, &[], &[1.0], TOL, &[], &[], none).err(), Some(Error::Argument));
    assert_eq!(blend(&b, Blend::Round, &[[f64::NAN; 3]], &[1.0], TOL, &[], &[], none).err(), Some(Error::Argument));
    assert_eq!(blend(&b, Blend::Round, &[TOP_FRONT], &[-1.0], TOL, &[], &[], none).err(), Some(Error::Argument));
    assert_eq!(blend(&b, Blend::Round, &[TOP_FRONT], &[1.0], 0.0, &[], &[], none).err(), Some(Error::Argument));
    assert_eq!(blend(&b, Blend::Round, &[TOP_FRONT], &[f64::INFINITY], TOL, &[], &[], none).err(), Some(Error::Argument));
    // A chamfer's corners are bevelled, which builds no setback.
    assert_eq!(blend(&b, Blend::Chamfer, &[TOP_FRONT], &[1.0], TOL, &[[0.0, 0.0, 10.0]], &[1.0], none).err(), Some(Error::Unsupported));
    assert_eq!(fillet_variable(&b, TOP_FRONT, TOL, [0.0; 3], 1, &[0.0, 1.0], &[1.0, 2.0]).err(), Some(Error::Argument));
    // A radius that pinches to nothing inside the edge.
    assert_eq!(fillet_variable(&b, TOP_FRONT, TOL, [0.0; 3], 1, &[0.0, 0.0, 0.5, 1.0, 1.0], &[1.0, 0.0, 1.0]).err(), Some(Error::Unsupported));
    assert_eq!(chamfer_sides(&b, &[TOP_FRONT], TOL, 1.0, ChamferSecond::Angle(4.0), [0.0, 0.0, 1.0]).err(), Some(Error::Argument));
    assert_eq!(chamfer_sides(&b, &[TOP_FRONT], TOL, 1.0, ChamferSecond::Distance(1.0), [0.0; 3]).err(), Some(Error::Argument));
}

/// A setback longer than an edge at its corner leaves no room, before any blend is made.
#[test]
fn a_setback_past_an_edge_has_no_room() {
    let r = timed("setback_long", || {
        let b = BLOCK.core();
        let corner = [[15.0, 20.0, 10.0], [30.0, 10.0, 10.0], [30.0, 20.0, 5.0]];
        blend(&b, Blend::Round, &corner, &[2.0; 3], TOL, &[[30.0, 20.0, 10.0]], &[10.0], Corners::Filled).err()
    });
    assert_eq!(r, Some(Error::NoRoom));
}

/// The body blended is never changed.
#[test]
fn the_body_stays_as_it_was() {
    let r = timed("operand", || {
        let b = BLOCK.core();
        let _ = blend(&b, Blend::Round, &[TOP_FRONT], &[2.0], TOL, &[], &[], Corners::Filled);
        (counts(&b).unwrap(), volume(&b, 1e-8).unwrap())
    });
    assert_eq!(r.0, [6, 12, 8]);
    assert!(rel(r.1, ABC) < 1e-12, "{}", r.1);
}

/// The chamfer's first distance goes on the face leaning most along `first`: 2 along the top and
/// 3 down the front for +z, the other way round for -y. The volumes are the same; the centroid
/// tells them apart. The removed prism's section is the triangle with legs t (back along the
/// top, in y) and f (down the front, in z) at the edge (y = 0, z = 10), centroid
/// (t/3, 10 - f/3), area t f / 2: the block keeps ȳ = (6000·10 - 15 t f · t/3) / (6000 - 15 t f)
/// and z̄ = (6000·5 - 15 t f (10 - f/3)) / (6000 - 15 t f).
#[test]
fn the_first_distance_goes_on_the_face_along_first() {
    for (first, top, front) in [([0.0, 0.0, 1.0], 2.0, 3.0), ([0.0, -1.0, 0.0], 3.0, 2.0)] {
        let r = timed("sides_first", move || {
            let b = BLOCK.core();
            chamfer_sides(&b, &[TOP_FRONT], TOL, 2.0, ChamferSecond::Distance(3.0), first).and_then(|c| super::mass_properties(&c, 1e-8))
        });
        let m = r.unwrap_or_else(|e| panic!("{first:?}: {e}"));
        let cut = 15.0 * top * front;
        let y = (6000.0 * 10.0 - cut * top / 3.0) / (6000.0 - cut);
        let z = (6000.0 * 5.0 - cut * (10.0 - front / 3.0)) / (6000.0 - cut);
        assert!(rel(m.volume, 6000.0 - cut) < 1e-9, "{first:?}: volume {}", m.volume);
        assert!((m.centroid[1] - y).abs() < 1e-7 && (m.centroid[2] - z).abs() < 1e-7, "{first:?}: centroid {:?} vs ({y}, {z})", m.centroid);
    }
}
