//! Extrusions, revolutions, sweeps, lofts and planar faces on solidcore, against closed forms:
//! every result exports to a record that validates. With SMLib built, the same case on the C++
//! kernel gives the same volume (or, for a sheet, area), counts and faces by surface kind, or the
//! same error (parity only: the expected values are the analytic ones).
//!
//! The profiles are built straight into solidcore curves from the curve lists (a record's curved
//! surfaces do not come into solidcore yet, solidcore#1432, #1433).

use std::collections::BTreeMap;
use std::f64::consts::{FRAC_PI_2, PI, TAU};
use std::sync::mpsc;
use std::time::Duration;

use super::sweep::{extrude, face_section, loft, loft_to_point, planar_faces, project_curves, revolve, screw_sweep, sweep_placed, sweep_stations};
use super::{Body, area, bounds, counts, make_box, to_arrays, volume};
use crate::Error;
use crate::export::native;
use crate::sweeps::{Affine, CornerRefusal, Curve, DraftCorner, Ends, SweepFrame};

/// A case longer than this is a hang.
const TIMEOUT: Duration = Duration::from_secs(300);

const Z: [f64; 3] = [0.0, 0.0, 1.0];

fn line(a: [f64; 3], b: [f64; 3]) -> Curve {
    Curve::Line { start: a, end: b }
}

/// The closed polygon through `pts`.
fn polygon(pts: &[[f64; 3]]) -> Vec<Curve> {
    (0..pts.len()).map(|i| line(pts[i], pts[(i + 1) % pts.len()])).collect()
}

/// The square of side `s` from `(x, y)` in the plane z = `z`.
fn square(x: f64, y: f64, s: f64, z: f64) -> Vec<Curve> {
    polygon(&[[x, y, z], [x + s, y, z], [x + s, y + s, z], [x, y + s, z]])
}

/// The circle of `radius` about `axis` through `center`, from `reference`.
fn circle(center: [f64; 3], axis: [f64; 3], reference: [f64; 3], radius: f64) -> Curve {
    Curve::Arc { center, axis, reference, radius, start: 0.0, end: TAU }
}

/// The rectangle from x = 5 to 8 and z = 0 to 4 in the plane y = 0: area 12, centroid at x 6.5.
fn ring_section() -> Vec<Curve> {
    polygon(&[[5.0, 0.0, 0.0], [8.0, 0.0, 0.0], [8.0, 0.0, 4.0], [5.0, 0.0, 4.0]])
}

fn translate(t: [f64; 3]) -> Affine {
    Affine { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]], t }
}

fn scale_up(s: f64, z: f64) -> Affine {
    Affine { m: [[s, 0.0, 0.0], [0.0, s, 0.0], [0.0, 0.0, 1.0]], t: [0.0, 0.0, z] }
}

#[derive(Clone, Debug)]
enum Op {
    PlanarFaces(Vec<Curve>),
    Extrude(Vec<Curve>, [f64; 3], f64, f64, DraftCorner, bool),
    Revolve(Vec<Curve>, [f64; 3], [f64; 3], f64, bool),
    Sweep(Vec<Curve>, Vec<Curve>, SweepFrame, bool),
    Loft(Vec<Vec<Curve>>, u32, bool),
    LoftToPoint(Vec<Curve>, [f64; 3]),
    Stations(Vec<Curve>, Vec<Affine>, Ends),
    Screw(Vec<Curve>, [f64; 3], [f64; 3], f64, f64, f64),
    /// A screw sweep with a radial law.
    ScrewRadial(Vec<Curve>, [f64; 3], [f64; 3], f64, f64, Vec<(f64, f64)>, f64),
}

/// What a case must give.
#[derive(Clone, Debug)]
enum Expect {
    /// A solid of this volume (relative 1e-9).
    Volume(f64),
    /// A solid of this volume within this relative error (walls approximated within a tolerance).
    About(f64, f64),
    /// A sheet of this area (relative 1e-9).
    Sheet(f64),
    /// A drafted corner refused for this reason, wherever.
    Corner(CornerRefusal),
}

struct Case {
    op: Op,
    expect: Expect,
}

/// The frustum of a pyramid or cone: h/3 (A1 + A2 + sqrt(A1 A2)).
fn frustum(a1: f64, a2: f64, h: f64) -> f64 {
    h / 3.0 * (a1 + a2 + (a1 * a2).sqrt())
}

fn case(name: &str) -> Case {
    let c = |op, expect| Case { op, expect };
    let rect = polygon(&[[0.0, 0.0, 0.0], [30.0, 0.0, 0.0], [30.0, 20.0, 0.0], [0.0, 20.0, 0.0]]);
    let theta = 5f64.to_radians();
    let disc = || vec![circle([0.0; 3], Z, [1.0, 0.0, 0.0], 5.0)];
    match name {
        // A square of 10 with a hole of radius 2: 100 - 4π.
        "planar_faces_with_a_hole" => {
            let mut loops = square(0.0, 0.0, 10.0, 0.0);
            loops.push(circle([5.0, 5.0, 0.0], Z, [1.0, 0.0, 0.0], 2.0));
            c(Op::PlanarFaces(loops), Expect::Sheet(100.0 - 4.0 * PI))
        }
        // V = A h.
        "extrude_rectangle" => c(Op::Extrude(rect, Z, 10.0, 0.0, DraftCorner::default(), true), Expect::Volume(6000.0)),
        // V = π r² h.
        "extrude_circle" => c(Op::Extrude(disc(), Z, 10.0, 0.0, DraftCorner::default(), true), Expect::Volume(250.0 * PI)),
        // Leaning in by θ, the walls' offsets overlap at the corners and are trimmed where they
        // cross, under every rule: the frustum from side 20 to 20 - 2 h tanθ.
        "extrude_square_drafted_round" => {
            let top = 20.0 - 2.0 * 10.0 * theta.tan();
            c(Op::Extrude(square(0.0, 0.0, 20.0, 0.0), Z, 10.0, theta, DraftCorner::Round, true), Expect::Volume(frustum(400.0, top * top, 10.0)))
        }
        "extrude_square_drafted_curvature" => {
            let top = 20.0 - 2.0 * 10.0 * theta.tan();
            c(Op::Extrude(square(0.0, 0.0, 20.0, 0.0), Z, 10.0, theta, DraftCorner::Curvature, true), Expect::Volume(frustum(400.0, top * top, 10.0)))
        }
        // No corner: the cone frustum from radius 5 to 5 - h tanθ.
        "extrude_circle_drafted" => {
            let r2 = 5.0 - 10.0 * theta.tan();
            c(Op::Extrude(disc(), Z, 10.0, theta, DraftCorner::default(), true), Expect::Volume(frustum(PI * 25.0, PI * r2 * r2, 10.0)))
        }
        // A strip 2 wide leaning in 10° over 10: the long walls meet across the short ones
        // (10 tan 10° > 1) before the far end.
        "extrude_draft_wall_closes" => c(
            Op::Extrude(
                polygon(&[[0.0, 0.0, 0.0], [20.0, 0.0, 0.0], [20.0, 2.0, 0.0], [0.0, 2.0, 0.0]]),
                Z,
                10.0,
                10f64.to_radians(),
                DraftCorner::Curvature,
                true,
            ),
            Expect::Corner(CornerRefusal::WallCloses),
        ),
        // Pappus: A · θ x̄ = 12 · θ · 6.5.
        "revolve_rectangle_full" => c(Op::Revolve(ring_section(), [0.0; 3], Z, TAU, true), Expect::Volume(12.0 * TAU * 6.5)),
        "revolve_rectangle_quarter" => c(Op::Revolve(ring_section(), [0.0; 3], Z, FRAC_PI_2, true), Expect::Volume(12.0 * FRAC_PI_2 * 6.5)),
        // π r² L, along two lines end to end.
        "sweep_circle_straight" => c(
            Op::Sweep(
                vec![circle([0.0; 3], Z, [1.0, 0.0, 0.0], 2.0)],
                vec![line([0.0; 3], [0.0, 0.0, 6.0]), line([0.0, 0.0, 6.0], [0.0, 0.0, 10.0])],
                SweepFrame::Follow,
                true,
            ),
            Expect::Volume(PI * 4.0 * 10.0),
        ),
        // A quarter of a torus: π r² · R θ.
        "sweep_circle_along_an_arc" => c(
            Op::Sweep(
                vec![circle([10.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], 1.0)],
                vec![Curve::Arc { center: [0.0; 3], axis: Z, reference: [1.0, 0.0, 0.0], radius: 10.0, start: 0.0, end: FRAC_PI_2 }],
                SweepFrame::Follow,
                true,
            ),
            Expect::Volume(PI * 10.0 * FRAC_PI_2),
        ),
        // A bent path (a line, then a quarter arc) needs the curve sweep: π r² times the path's
        // length, 10 + 10 π/2, within the walls' fit.
        "sweep_along_a_bent_path" => c(
            Op::Sweep(
                vec![circle([0.0; 3], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0], 1.0)],
                vec![
                    line([0.0; 3], [10.0, 0.0, 0.0]),
                    Curve::Arc { center: [10.0, 10.0, 0.0], axis: Z, reference: [0.0, -1.0, 0.0], radius: 10.0, start: 0.0, end: FRAC_PI_2 },
                ],
                SweepFrame::Follow,
                true,
            ),
            Expect::About(PI * (10.0 + 10.0 * FRAC_PI_2), 1e-4),
        ),
        // The frustum of a square pyramid: side 20 to 10 over 10.
        "loft_two_squares" => {
            c(Op::Loft(vec![square(0.0, 0.0, 20.0, 0.0), square(5.0, 5.0, 10.0, 10.0)], 1, true), Expect::Volume(frustum(400.0, 100.0, 10.0)))
        }
        // The cone frustum: radius 5 to 3 over 10.
        "loft_two_circles" => c(
            Op::Loft(vec![disc(), vec![circle([0.0, 0.0, 10.0], Z, [1.0, 0.0, 0.0], 3.0)]], 1, true),
            Expect::Volume(frustum(25.0 * PI, 9.0 * PI, 10.0)),
        ),
        // A h / 3.
        "loft_to_point_pyramid" => c(Op::LoftToPoint(square(0.0, 0.0, 10.0, 0.0), [5.0, 5.0, 6.0]), Expect::Volume(200.0)),
        // A square of 2 carried up 10: 40.
        "sweep_stations_prism" => c(
            Op::Stations(
                square(-1.0, -1.0, 2.0, 0.0),
                vec![translate([0.0; 3]), translate([0.0, 0.0, 5.0]), translate([0.0, 0.0, 10.0])],
                Ends::Capped,
            ),
            Expect::Volume(40.0),
        ),
        // Scaled by s = 1 + z/10 on the way: ∫ (2 s)² dz = 4 · 10 · (2³ - 1) / 3.
        "sweep_stations_scaled" => c(
            Op::Stations(square(-1.0, -1.0, 2.0, 0.0), vec![scale_up(1.0, 0.0), scale_up(1.5, 5.0), scale_up(2.0, 10.0)], Ends::Capped),
            Expect::Volume(4.0 * 10.0 * 7.0 / 3.0),
        ),
        // A square thread 1 x 1 from radius 5 to 6, pitch 2, two turns: Pappus along the screw
        // motion (its rise lies in the profile's plane), A · 2π x̄ · turns, the walls within the
        // tolerance of the screw surfaces.
        "screw_sweep_square_thread" => c(
            Op::Screw(polygon(&[[5.0, 0.0, 0.0], [6.0, 0.0, 0.0], [6.0, 0.0, 1.0], [5.0, 0.0, 1.0]]), [0.0; 3], Z, 2.0, 2.0, 1e-5),
            Expect::About(TAU * 5.5 * 2.0, 1e-5),
        ),
        // The same thread moved out by 0 to 1 over its two turns, linearly: Pappus with the
        // centroid at 5.5 + t/2, A · 2π · turns · (5.5 + 1/2).
        "screw_sweep_radial_taper" => c(
            Op::ScrewRadial(
                polygon(&[[5.0, 0.0, 0.0], [6.0, 0.0, 0.0], [6.0, 0.0, 1.0], [5.0, 0.0, 1.0]]),
                [0.0; 3],
                Z,
                2.0,
                2.0,
                vec![(0.0, 0.0), (2.0, 1.0)],
                1e-5,
            ),
            Expect::About(TAU * 2.0 * 6.0, 1e-5),
        ),
        _ => panic!("{name}"),
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

#[derive(Debug, PartialEq)]
struct Measured {
    /// The volume of a solid, the area of a sheet.
    size: f64,
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

fn on_core(op: &Op) -> crate::Result<Body> {
    match op {
        Op::PlanarFaces(l) => planar_faces(l),
        Op::Extrude(p, d, dist, draft, corner, cap) => extrude(p, *d, *dist, *draft, *corner, *cap),
        Op::Revolve(p, o, a, angle, cap) => revolve(p, *o, *a, *angle, *cap),
        Op::Sweep(p, path, frame, cap) => sweep_placed(p, path, *frame, *cap).map(|(b, _)| b),
        Op::Loft(s, d, cap) => loft(s, *d, *cap),
        Op::LoftToPoint(s, apex) => loft_to_point(s, *apex),
        Op::Stations(p, s, e) => sweep_stations(p, s, *e),
        Op::Screw(p, o, a, pitch, turns, tol) => screw_sweep(p, *o, *a, *pitch, *turns, None, *tol),
        Op::ScrewRadial(p, o, a, pitch, turns, law, tol) => screw_sweep(p, *o, *a, *pitch, *turns, Some(law), *tol),
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
        (Expect::Corner(why), Err(g)) => {
            assert!(
                matches!(g, Error::Corner { why: w, at } if w == why && at.iter().all(|x| x.is_finite())),
                "{name}: {who} failed otherwise: {g:?}"
            )
        }
        (Expect::Corner(_), Ok(m)) => panic!("{name}: {who} gave {m:?}, expected {expect:?}"),
        (_, Err(g)) => panic!("{name}: {who} failed: {g}"),
        (Expect::Volume(v) | Expect::Sheet(v), Ok(m)) => assert!(rel(m.size, *v) < 1e-9, "{name}: {who} {} vs analytic {v}", m.size),
        (Expect::About(v, within), Ok(m)) => assert!(rel(m.size, *v) < *within, "{name}: {who} {} vs analytic {v}", m.size),
    }
    got
}

fn solidcore_case(name: &'static str) -> Got {
    timed(name, move || {
        let c = case(name);
        let sheet = matches!(c.expect, Expect::Sheet(_));
        let got = on_core(&c.op).map(|body| {
            let size = if sheet { area(&body, 1e-8) } else { volume(&body, 1e-8) }.unwrap_or_else(|e| panic!("{name}: size: {e}"));
            let n = counts(&body).unwrap();
            let (rec, _) = native(&to_arrays(&body).unwrap()).unwrap_or_else(|e| panic!("{name}: record: {e:?}"));
            rec.validate().unwrap_or_else(|e| panic!("{name}: record: {e}"));
            bounds(&body).unwrap();
            Measured { size, counts: n, kinds: kinds(&rec) }
        });
        judge(name, "solidcore", &c.expect, got)
    })
}

/// The case on the C++ kernel, through the public sweeps forced onto it on this thread.
#[cfg(feature = "native")]
fn cpp_case(name: &str) -> Got {
    use crate::{Brep, ffi};
    crate::sweeps::ON_CPP.with(|c| c.set(true));
    let c = case(name);
    let r = match &c.op {
        Op::PlanarFaces(l) => Brep::planar_faces(l),
        Op::Extrude(p, d, dist, draft, corner, cap) => Brep::extrude(p, *d, *dist, *draft, *corner, *cap),
        Op::Revolve(p, o, a, angle, cap) => Brep::revolve(p, *o, *a, *angle, *cap),
        Op::Sweep(p, path, frame, cap) => Brep::sweep(p, path, *frame, *cap),
        Op::Loft(s, d, cap) => Brep::loft(s, *d, *cap),
        Op::LoftToPoint(s, apex) => Brep::loft_to_point(s, *apex),
        Op::Stations(p, s, e) => Brep::sweep_stations(p, s, *e),
        Op::Screw(p, o, a, pitch, turns, tol) => Brep::screw_sweep(p, *o, *a, *pitch, *turns, *tol),
        Op::ScrewRadial(p, o, a, pitch, turns, law, tol) => Brep::screw_sweep_radial(p, *o, *a, *pitch, *turns, law, *tol),
    };
    crate::sweeps::ON_CPP.with(|c| c.set(false));
    let sheet = matches!(c.expect, Expect::Sheet(_));
    let got = r.map(|b| {
        let h = b.cpp().unwrap();
        let size = if sheet { ffi::measure::area(h, 1e-8) } else { ffi::volume(h, 1e-8) }.unwrap();
        let n = ffi::counts(h).unwrap().map(|x| usize::try_from(x).unwrap());
        let (rec, _) = native(&b.cpp_arrays().unwrap()).unwrap();
        rec.validate().unwrap();
        Measured { size, counts: n, kinds: kinds(&rec) }
    });
    judge(name, "C++", &c.expect, got)
}

/// solidcore and C++ agree: volume (area) within 1e-9, the same counts and faces by kind, or the
/// same error.
#[cfg(feature = "native")]
fn parity(name: &'static str) {
    let cpp = cpp_case(name);
    let ours = solidcore_case(name);
    match (ours, cpp) {
        (Ok(a), Ok(b)) => {
            assert!(rel(a.size, b.size) < 1e-9, "{name}: solidcore {} vs C++ {}", a.size, b.size);
            assert_eq!(a.counts, b.counts, "{name}: face, edge, vertex counts (solidcore vs C++)");
            assert_eq!(a.kinds, b.kinds, "{name}: faces by surface kind (solidcore vs C++)");
        }
        (a, b) => assert_eq!(a.err(), b.err(), "{name}: errors (solidcore vs C++)"),
    }
}

macro_rules! cases {
    ($($m:ident $([$pign:meta])? $(: $ign:meta)?),* $(,)?) => {$(
        mod $m {
            #[test]
            $(#[$ign])?
            fn solidcore_matches_the_closed_form() {
                let _ = super::solidcore_case(stringify!($m));
            }
            #[cfg(feature = "native")]
            #[test]
            $(#[$ign])?
            $(#[$pign])?
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

// Cases marked `[ignore = …]` match the closed form on solidcore but not the C++ kernel's
// numbers; `: ignore = …` ones fail on solidcore.
cases!(
    planar_faces_with_a_hole,
    extrude_rectangle,
    extrude_circle,
    extrude_square_drafted_round,
    extrude_square_drafted_curvature: ignore = "solidcore: no curvature-keeping draft corners, fork 54148db",
    extrude_circle_drafted,
    extrude_draft_wall_closes: ignore = "solidcore: no curvature-keeping draft corners nor SmTaperCornerRefusal, fork 54148db",
    revolve_rectangle_full,
    revolve_rectangle_quarter,
    sweep_circle_straight,
    sweep_circle_along_an_arc: ignore = "solidcore#1435: a torus's volume at 1e-8 is 2.2e-9 off; precise properties do not split at the surface's knots, fork 08d9a14 (solidcore#1435)",
    sweep_along_a_bent_path: ignore = "solidcore: no curve sweep keeping the profile at the path's start, fork 2dd71a0 / 5d3939e",
    loft_two_squares,
    loft_two_circles,
    loft_to_point_pyramid,
    sweep_stations_prism,
    sweep_stations_scaled,
    screw_sweep_square_thread,
    screw_sweep_radial_taper [ignore = "solidcore#1435: B-spline walls' volume at 1e-8 is 3.6e-8 off the closed form (C++ 1.1e-8); precise properties do not split at the surface's knots, fork 08d9a14 (solidcore#1435)"],
);

fn near(a: [f64; 3], b: [f64; 3]) -> bool {
    a.iter().zip(&b).all(|(x, y)| (x - y).abs() < 1e-9)
}

fn same_motion(a: &Affine, b: &Affine) -> bool {
    a.m.iter().zip(&b.m).all(|(x, y)| near(*x, *y)) && near(a.t, b.t)
}

/// The motions of a quarter turn about the z axis: by 0, π/4 and π/2.
fn quarter_turn() -> [Affine; 3] {
    [0.0, FRAC_PI_2 / 2.0, FRAC_PI_2].map(|a: f64| {
        let (c, s) = (a.cos(), a.sin());
        Affine { m: [[c, s, 0.0], [-s, c, 0.0], [0.0, 0.0, 1.0]], t: [0.0; 3] }
    })
}

fn straight_up() -> (Vec<Curve>, Vec<Curve>) {
    (vec![circle([0.0; 3], Z, [1.0, 0.0, 0.0], 2.0)], vec![line([0.0; 3], [0.0, 0.0, 10.0])])
}

fn round_z() -> (Vec<Curve>, Vec<Curve>) {
    (
        vec![circle([10.0, 0.0, 0.0], [0.0, 1.0, 0.0], [1.0, 0.0, 0.0], 1.0)],
        vec![Curve::Arc { center: [0.0; 3], axis: Z, reference: [1.0, 0.0, 0.0], radius: 10.0, start: 0.0, end: FRAC_PI_2 }],
    )
}

fn check_motions(placed: &[Affine; 3], want: &[Affine; 3]) {
    assert!(placed.iter().zip(want).all(|(a, b)| same_motion(a, b)), "{placed:?}, expected {want:?}");
}

/// `sweep_placed`: along a straight path the profile only moves, by none, half and all of the
/// path; along an arc about z it turns with it about the arc's centre, by none, half and all of
/// the arc's angle.
#[test]
fn sweep_placed_reports_the_motions() {
    let (placed, turned) = timed("placed", || {
        let (p, up) = straight_up();
        let (q, arc) = round_z();
        (sweep_placed(&p, &up, SweepFrame::Follow, true).unwrap().1, sweep_placed(&q, &arc, SweepFrame::Follow, true).unwrap().1)
    });
    check_motions(&placed, &[0.0, 5.0, 10.0].map(|z| translate([0.0, 0.0, z])));
    check_motions(&turned, &quarter_turn());
}

/// The same motions on the C++ kernel.
#[cfg(feature = "native")]
#[test]
fn the_cpp_kernel_places_the_profile_alike() {
    crate::sweeps::ON_CPP.with(|c| c.set(true));
    let (p, up) = straight_up();
    let (q, arc) = round_z();
    let placed = crate::Brep::sweep_placed(&p, &up, SweepFrame::Follow, true).unwrap().1;
    let turned = crate::Brep::sweep_placed(&q, &arc, SweepFrame::Follow, true).unwrap().1;
    crate::sweeps::ON_CPP.with(|c| c.set(false));
    check_motions(&placed, &[0.0, 5.0, 10.0].map(|z| translate([0.0, 0.0, z])));
    check_motions(&turned, &quarter_turn());
}

/// A diagonal 10 above a 10 x 10 x 10 box's top, projected down onto the top: the same diagonal
/// at z = 10.
#[test]
#[ignore = "solidcore#1440: the topology point solve misses faces, so SmApiFaceClosestPoint picks no face (solidcore#1440)"]
fn project_a_line_onto_a_face() {
    let got = timed("project", || {
        let b = make_box([0.0; 3], [10.0; 3]).unwrap();
        project_curves(&b, [5.0, 5.0, 10.0], 0.01, &[line([1.0, 1.0, 20.0], [9.0, 9.0, 20.0])], [0.0, 0.0, -1.0], 1e-6)
    })
    .unwrap();
    assert_eq!(got.len(), 1, "{got:?}");
    let Some((Curve::Nurbs { points, .. }, _)) = got.first() else { panic!("{got:?}") };
    let (a, b) = (points[0], points[points.len() - 1]);
    let ends = [[a[0], a[1], a[2]], [b[0], b[1], b[2]]];
    let want = [[1.0, 1.0, 10.0], [9.0, 9.0, 10.0]];
    assert!((near(ends[0], want[0]) && near(ends[1], want[1])) || (near(ends[0], want[1]) && near(ends[1], want[0])), "{ends:?}");
}

/// A face's planar section needs `SmBrep::CreatePlanarSectionCurves`: not supported, after the
/// face pick.
#[test]
#[ignore = "solidcore#1440: the topology point solve misses faces, so the face pick fails first (solidcore#1440)"]
fn a_face_section_is_not_supported_yet() {
    let got = timed("section", || {
        let b = make_box([0.0; 3], [10.0; 3]).unwrap();
        face_section(&b, [5.0, 5.0, 10.0], 0.01, [5.0, 5.0, 5.0], [1.0, 0.0, 0.0], 1e-6)
    });
    assert_eq!(got.err(), Some(Error::Unsupported));
}

/// The capi's argument checks, before any kernel call.
#[test]
fn hostile_arguments_are_refused() {
    let sq = square(0.0, 0.0, 1.0, 0.0);
    let d = DraftCorner::default();
    for dist in [0.0, -1.0, f64::NAN, 1e10] {
        assert_eq!(extrude(&sq, Z, dist, 0.0, d, true).err(), Some(Error::Argument), "distance {dist}");
    }
    assert_eq!(extrude(&sq, Z, 1.0, FRAC_PI_2, d, true).err(), Some(Error::Argument), "a right-angled draft");
    assert_eq!(extrude(&sq, [0.0; 3], 1.0, 0.0, d, true).err(), Some(Error::Argument), "no direction");
    assert_eq!(extrude(&[], Z, 1.0, 0.0, d, true).err(), Some(Error::Argument), "no profile");
    assert_eq!(extrude(&[line([0.0; 3], [0.0; 3])], Z, 1.0, 0.0, d, true).err(), Some(Error::Argument), "a point");
    assert_eq!(extrude(&[line([f64::NAN; 3], [1.0; 3])], Z, 1.0, 0.0, d, true).err(), Some(Error::Argument));
    // A drafted profile is one loop, square to the direction.
    assert_eq!(extrude(&sq[..3], Z, 1.0, 0.1, d, true).err(), Some(Error::Argument), "an open profile");
    assert_eq!(extrude(&sq, [1.0, 0.0, 0.0], 1.0, 0.1, d, true).err(), Some(Error::Argument), "along the profile");
    let bad_arc = Curve::Arc { center: [0.0; 3], axis: Z, reference: Z, radius: 1.0, start: 0.0, end: 1.0 };
    assert_eq!(extrude(&[bad_arc], Z, 1.0, 0.0, d, false).err(), Some(Error::Argument), "reference along the axis");
    let bad_nurbs = Curve::Nurbs { degree: 1, points: vec![[0.0; 4], [1.0, 0.0, 0.0, 1.0]], knots: vec![0.0, 0.0, 1.0, 1.0] };
    assert_eq!(extrude(&[bad_nurbs], Z, 1.0, 0.0, d, false).err(), Some(Error::Argument), "a zero weight");
    let unclamped = Curve::Nurbs { degree: 1, points: vec![[0.0, 0.0, 0.0, 1.0], [1.0, 0.0, 0.0, 1.0]], knots: vec![0.0, 0.5, 1.0, 1.0] };
    assert_eq!(extrude(&[unclamped], Z, 1.0, 0.0, d, false).err(), Some(Error::Argument), "unclamped knots");
    assert_eq!(revolve(&sq, [0.0; 3], Z, 0.0, true).err(), Some(Error::Argument), "no angle");
    assert_eq!(revolve(&sq, [0.0; 3], Z, 7.0, true).err(), Some(Error::Argument), "more than a turn");
    assert_eq!(revolve(&sq, [f64::INFINITY; 3], Z, 1.0, true).err(), Some(Error::Argument));
    assert_eq!(loft(std::slice::from_ref(&sq), 1, true).err(), Some(Error::Argument), "one section");
    assert_eq!(loft(&[sq.clone(), sq.clone()], 4, true).err(), Some(Error::Argument), "degree 4");
    assert_eq!(loft_to_point(&sq, [0.5, 0.5, 0.0]).err(), Some(Error::Argument), "apex in the base's plane");
    assert_eq!(loft_to_point(&[circle([0.0; 3], Z, [1.0, 0.0, 0.0], 1.0)], Z).err(), Some(Error::Argument), "not lines");
    let flat = Affine { m: [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 0.0]], t: [0.0; 3] };
    assert_eq!(sweep_stations(&sq, &[translate([0.0; 3]), flat], Ends::Capped).err(), Some(Error::Argument), "a singular map");
    assert_eq!(sweep_stations(&sq, &[translate([0.0; 3])], Ends::Capped).err(), Some(Error::Argument), "one station");
    assert_eq!(screw_sweep(&sq, [0.0; 3], Z, 0.0, 1.0, None, 1e-3).err(), Some(Error::Argument), "no pitch");
    assert_eq!(screw_sweep(&sq, [0.0; 3], Z, 1.0, 300.0, None, 1e-3).err(), Some(Error::Argument), "too many turns");
    assert_eq!(screw_sweep(&sq, [0.0; 3], Z, 1.0, 1.0, Some(&[(0.5, 0.0), (0.2, 0.0)]), 1e-3).err(), Some(Error::Argument), "a law going back");
    assert_eq!(screw_sweep(&sq, [0.0; 3], Z, 1.0, 1.0, Some(&[]), 1e-3).err(), Some(Error::Argument), "an empty law");
    let b = make_box([0.0; 3], [1.0; 3]).unwrap();
    assert_eq!(project_curves(&b, [0.5; 3], 0.01, &sq, [0.0; 3], 1e-6).err(), Some(Error::Argument), "no direction");
    assert_eq!(project_curves(&b, [0.5; 3], 0.0, &sq, Z, 1e-6).err(), Some(Error::Argument), "no pick tolerance");
    assert_eq!(face_section(&b, [0.5; 3], 0.01, [0.5; 3], [0.0; 3], 1e-6).err(), Some(Error::Argument), "no plane normal");
}
