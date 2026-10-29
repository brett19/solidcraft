//! Booleans on solidcore, against analytic volumes: every result exports to a record that
//! validates and counts the solids it should. With SMLib built, the same case on the C++ kernel
//! gives the same volume, counts, faces by surface kind and solids (parity only: the expected
//! values are the analytic ones).
//!
//! Curved operands are built on solidcore itself.

use std::collections::BTreeMap;
use std::f64::consts::PI;
use std::sync::mpsc;
use std::time::Duration;

use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::geom::vector3d::Vec3;
use solidcore::mass::integrator::properties::brep_area;
use solidcore::primitive::primitive_creation::p1::create_rectangle;

use super::boolean::{boolean, boolean2d, imprint, union_all};
use super::{Body, counts, make_box, make_cylinder, make_sphere, run, solid_count, to_arrays, volume, with_body};
use crate::export::native;
use crate::{BooleanOp, Error};

/// A case longer than this is a hang.
const TIMEOUT: Duration = Duration::from_secs(300);

/// An operand.
#[derive(Clone, Copy, Debug)]
enum Shape {
    Box([f64; 3], [f64; 3]),
    Cylinder([f64; 3], f64, f64),
    Sphere([f64; 3], f64),
}

impl Shape {
    fn core(self) -> Body {
        match self {
            Shape::Box(o, s) => make_box(o, s),
            Shape::Cylinder(o, r, h) => make_cylinder(o, r, h),
            Shape::Sphere(c, r) => make_sphere(c, r),
        }
        .unwrap()
    }
}

/// What a case does.
#[derive(Clone, Debug)]
enum Op {
    Boolean(Shape, Shape, BooleanOp),
    UnionAll(Vec<Shape>),
    Imprint(Shape, Shape),
}

/// A case: the operation, its analytic volume (`None`: the result is empty), the faces it
/// must have where they are known analytically, and its solids (`smlib_brep_solid_count`).
struct Case {
    op: Op,
    volume: Option<f64>,
    faces: Option<usize>,
    solids: usize,
}

const A: Shape = Shape::Box([0.0; 3], [10.0; 3]);

fn case(name: &str) -> Case {
    let plain = |op, volume: Option<f64>, faces| Case { op, volume, faces, solids: 1 };
    match name {
        // |A| + |B| - |A ∩ B|: two 10 mm cubes overlapping in a 5 mm cube.
        "union_overlapping" => plain(Op::Boolean(A, Shape::Box([5.0; 3], [10.0; 3]), BooleanOp::Union), Some(1000.0 + 1000.0 - 125.0), Some(12)),
        // Two 10 mm cubes 10 mm apart: one body of two solids, 12 faces.
        "union_disjoint" => {
            Case { solids: 2, ..plain(Op::Boolean(A, Shape::Box([20.0, 0.0, 0.0], [10.0; 3]), BooleanOp::Union), Some(2000.0), Some(12)) }
        }
        "intersect_overlapping" => plain(Op::Boolean(A, Shape::Box([5.0; 3], [10.0; 3]), BooleanOp::Intersect), Some(125.0), Some(6)),
        // A slot 5 mm wide and 5 mm deep from the top, right through a 30 x 20 x 10 block.
        "subtract_through_slot" => plain(
            Op::Boolean(Shape::Box([0.0; 3], [30.0, 20.0, 10.0]), Shape::Box([10.0, -1.0, 5.0], [5.0, 22.0, 6.0]), BooleanOp::Subtract),
            Some(6000.0 - 5.0 * 20.0 * 5.0),
            Some(10),
        ),
        // A notch whose tool is flush with the block's front, back and top faces.
        "subtract_flush_notch" => plain(
            Op::Boolean(Shape::Box([0.0; 3], [30.0, 20.0, 10.0]), Shape::Box([10.0, 0.0, 5.0], [10.0, 20.0, 5.0]), BooleanOp::Subtract),
            Some(6000.0 - 10.0 * 20.0 * 5.0),
            Some(10),
        ),
        // A 4 mm cube inside a 10 mm one: a void, still one solid.
        "subtract_inside_void" => plain(Op::Boolean(A, Shape::Box([3.0; 3], [4.0; 3]), BooleanOp::Subtract), Some(1000.0 - 64.0), Some(12)),
        // abc - πr²c: a 3 mm hole through a 20 x 20 x 10 block.
        "subtract_cylinder_hole" => plain(
            Op::Boolean(Shape::Box([0.0; 3], [20.0, 20.0, 10.0]), Shape::Cylinder([10.0, 10.0, -1.0], 3.0, 12.0), BooleanOp::Subtract),
            Some(4000.0 - PI * 9.0 * 10.0),
            None,
        ),
        // Nine 10 mm cubes, each touching its neighbours face to face.
        "union_all_grid" => plain(
            Op::UnionAll((0..9).map(|i| Shape::Box([f64::from(i % 3) * 10.0, f64::from(i / 3) * 10.0, 0.0], [10.0; 3])).collect()),
            Some(9000.0),
            None,
        ),
        "intersect_disjoint" => plain(Op::Boolean(A, Shape::Box([20.0; 3], [10.0; 3]), BooleanOp::Intersect), None, None),
        // A 10 mm sphere less a cap of height h = 5: 4/3 πr³ - πh²(3r - h)/3.
        // (solidcore#1431, brep_volume of a whole sphere failing at 1e-8, does not touch the cut
        // sphere: it measures at 1e-8.)
        "sphere_minus_box" => plain(
            Op::Boolean(Shape::Sphere([0.0; 3], 10.0), Shape::Box([-20.0, -20.0, 5.0], [40.0, 40.0, 20.0]), BooleanOp::Subtract),
            Some(4.0 / 3.0 * PI * 1000.0 - PI * 25.0 * 25.0 / 3.0),
            Some(2),
        ),
        // A 4 mm square tool through the cube's top face: that face is cut in two (a square and
        // the ring round it), nothing else changes.
        "imprint_top_face" => plain(Op::Imprint(A, Shape::Box([3.0, 3.0, 8.0], [4.0; 3])), Some(1000.0), Some(7)),
        _ => panic!("{name}"),
    }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// What a kernel's result measures.
#[derive(Debug, PartialEq)]
struct Measured {
    volume: f64,
    counts: [usize; 3],
    /// The record's faces by surface kind.
    kinds: BTreeMap<String, usize>,
    solids: usize,
}

fn kinds(rec: &solidcraft_geom::brep::Brep) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    for f in &rec.faces {
        let v = serde_json::to_value(&f.surface).unwrap();
        *out.entry(v["kind"].as_str().unwrap_or("?").to_owned()).or_default() += 1;
    }
    out
}

/// The case on solidcore: `None` for an empty result.
fn on_core(c: &Case) -> crate::Result<Option<Body>> {
    match &c.op {
        Op::Boolean(a, b, op) => boolean(&a.core(), &b.core(), *op),
        Op::UnionAll(shapes) => {
            let bodies: Vec<Body> = shapes.iter().map(|s| s.core()).collect();
            union_all(&bodies.iter().collect::<Vec<_>>()).map(Some)
        }
        Op::Imprint(a, t) => imprint(&a.core(), &t.core()),
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
        // The case panicked: its own message is the failure.
        Err(mpsc::RecvTimeoutError::Disconnected) => match worker.join() {
            Err(p) => std::panic::resume_unwind(p),
            Ok(()) => panic!("{name}: no result"),
        },
        Err(mpsc::RecvTimeoutError::Timeout) => panic!("{name}: solidcore hung (no result after {TIMEOUT:?})"),
    }
}

/// Runs the case on solidcore and checks it against the analytic answer; what it measured.
fn solidcore_case(name: &'static str) -> Option<Measured> {
    timed(name, move || {
        let c = case(name);
        let r = on_core(&c).unwrap_or_else(|e| panic!("{name}: solidcore failed: {e}"));
        let (Some(body), Some(expected)) = (&r, c.volume) else {
            assert!(r.is_none() && c.volume.is_none(), "{name}: solidcore gave {}, expected {:?}", r.is_some(), c.volume);
            return None;
        };
        let v = volume(body, 1e-8).unwrap_or_else(|e| panic!("{name}: volume: {e}"));
        assert!(rel(v, expected) < 1e-9, "{name}: solidcore volume {v} vs analytic {expected}");
        let n = counts(body).unwrap();
        if let Some(f) = c.faces {
            assert_eq!(n[0], f, "{name}: faces {n:?}");
        }
        let solids = solid_count(body).unwrap_or_else(|e| panic!("{name}: solid count: {e}"));
        assert_eq!(solids, c.solids, "{name}: solids");
        let (rec, _) = native(&to_arrays(body).unwrap()).unwrap_or_else(|e| panic!("{name}: record: {e:?}"));
        rec.validate().unwrap_or_else(|e| panic!("{name}: record: {e}"));
        Some(Measured { volume: v, counts: n, kinds: kinds(&rec), solids })
    })
}

/// The case on the C++ kernel, called directly (whatever the routing): what it measured.
#[cfg(feature = "native")]
fn cpp_case(name: &str) -> Option<Measured> {
    use crate::{Brep, ffi};
    crate::init().unwrap();
    let make = |s: Shape| -> Brep {
        match s {
            Shape::Box(o, z) => ffi::make_box(&o, &z),
            Shape::Cylinder(o, r, h) => ffi::make_cylinder(&o, r, h),
            Shape::Sphere(c, r) => ffi::make_sphere(&c, r),
        }
        .map(Brep::made)
        .unwrap()
    };
    let bool_op = |op| match op {
        BooleanOp::Union => ffi::ops::BOOLEAN_UNION,
        BooleanOp::Subtract => ffi::ops::BOOLEAN_SUBTRACT,
        BooleanOp::Intersect => ffi::ops::BOOLEAN_INTERSECT,
    };
    let c = case(name);
    let r = match &c.op {
        Op::Boolean(a, b, op) => ffi::ops::boolean(make(*a).cpp().unwrap(), make(*b).cpp().unwrap(), bool_op(*op)),
        Op::UnionAll(shapes) => {
            let bodies: Vec<Brep> = shapes.iter().map(|s| make(*s)).collect();
            let handles: Vec<&ffi::Handle> = bodies.iter().map(|b| b.cpp().unwrap()).collect();
            ffi::ops::boolean_union_all(&handles)
        }
        Op::Imprint(a, t) => ffi::ops::imprint(make(*a).cpp().unwrap(), make(*t).cpp().unwrap()),
    };
    let h = match r {
        Ok(h) => h,
        Err(ffi::ERR_EMPTY) => return None,
        Err(s) => panic!("{name}: C++ failed: {:?}", crate::error(s)),
    };
    let v = ffi::volume(&h, 1e-8).unwrap();
    let n = ffi::counts(&h).unwrap().map(|x| usize::try_from(x).unwrap());
    let solids = usize::try_from(ffi::ops::solid_count(&h).unwrap()).unwrap();
    let b = Brep::made(h);
    let (rec, _) = native(&b.cpp_arrays().unwrap()).unwrap();
    rec.validate().unwrap();
    Some(Measured { volume: v, counts: n, kinds: kinds(&rec), solids })
}

/// solidcore and C++ agree: volume within 1e-9, the same counts and faces by kind.
#[cfg(feature = "native")]
fn parity(name: &'static str) {
    let ours = solidcore_case(name);
    let cpp = cpp_case(name);
    match (ours, cpp) {
        (None, None) => {}
        (Some(a), Some(b)) => {
            assert!(rel(a.volume, b.volume) < 1e-9, "{name}: solidcore {} vs C++ {}", a.volume, b.volume);
            assert_eq!(a.counts, b.counts, "{name}: face, edge, vertex counts (solidcore vs C++)");
            assert_eq!(a.kinds, b.kinds, "{name}: faces by surface kind (solidcore vs C++)");
            assert_eq!(a.solids, b.solids, "{name}: solids (solidcore vs C++)");
        }
        (a, b) => panic!("{name}: solidcore {a:?} vs C++ {b:?}"),
    }
}

macro_rules! cases {
    ($($m:ident $(: $ign:meta)?),* $(,)?) => {$(
        mod $m {
            #[test]
            $(#[$ign])?
            fn solidcore_matches_the_analytic_volume() {
                super::solidcore_case(stringify!($m));
            }
            #[cfg(feature = "native")]
            #[test]
            $(#[$ign])?
            fn solidcore_matches_the_cpp_kernel() {
                super::parity(stringify!($m));
            }
        }
    )*};
}

cases!(
    union_overlapping,
    union_disjoint,
    intersect_overlapping,
    subtract_through_slot,
    subtract_flush_notch,
    subtract_inside_void,
    subtract_cylinder_hole,
    union_all_grid,
    intersect_disjoint,
    sphere_minus_box,
    imprint_top_face,
);

/// A `w` x `h` planar sheet on z = 0 from `(x, y)`, solidcore's `CreateRectangle`.
fn rectangle(x: f64, y: f64, w: f64, h: f64) -> Body {
    let frame = Axis2Placement::new(Vec3::new(x, y, 0.0), Vec3::new(1.0, 0.0, 0.0), Vec3::new(0.0, 1.0, 0.0));
    run("create_rectangle", |k| create_rectangle(k, 0.0, w, h, &frame)).map(Body::new).unwrap()
}

fn area(b: &Body) -> f64 {
    with_body("brep_area", b, |k, b| brep_area(k, b, 1e-8)).unwrap()
}

/// Two 10 mm squares overlapping in a 5 mm one, as regions of their plane.
#[test]
fn boolean2d_combines_overlapping_squares() {
    for (op, expected) in [(BooleanOp::Union, 175.0), (BooleanOp::Intersect, 25.0), (BooleanOp::Subtract, 75.0)] {
        let (a, b) = (rectangle(0.0, 0.0, 10.0, 10.0), rectangle(5.0, 5.0, 10.0, 10.0));
        let r = timed("boolean2d", move || boolean2d(&a, &b, op).map(|r| r.map(|r| (area(&r), counts(&r).unwrap()))));
        let (got, n) = r.unwrap_or_else(|e| panic!("{op:?}: {e}")).unwrap_or_else(|| panic!("{op:?}: empty"));
        assert!(rel(got, expected) < 1e-9, "{op:?}: area {got} vs {expected}");
        assert_eq!(n[0], 1, "{op:?}: {n:?}");
    }
    let (a, far) = (rectangle(0.0, 0.0, 10.0, 10.0), rectangle(20.0, 0.0, 10.0, 10.0));
    assert!(boolean2d(&a, &far, BooleanOp::Intersect).unwrap().is_none(), "disjoint squares intersected");
}

/// The capi's argument checks: the same body imprinted on itself, an empty union.
#[test]
fn hostile_operands_are_refused() {
    let a = A.core();
    assert_eq!(imprint(&a, &a).err(), Some(Error::Argument));
    assert_eq!(union_all(&[]).err(), Some(Error::Argument));
}

/// A tool that meets no face cuts none.
#[test]
fn imprint_of_a_far_tool_is_none() {
    let r = timed("imprint_far", || imprint(&A.core(), &Shape::Box([20.0; 3], [1.0; 3]).core()).map(|r| r.is_none()));
    assert!(r.unwrap());
}

/// An operand is never changed: the same body twice, and again after.
#[test]
fn operands_stay_as_they_were() {
    let r = timed("operands", || {
        let a = A.core();
        let b = Shape::Box([5.0; 3], [10.0; 3]).core();
        let u = boolean(&a, &b, BooleanOp::Union).map(|r| r.is_some());
        let same = boolean(&a, &a, BooleanOp::Union).map(|r| r.map(|r| volume(&r, 1e-8)));
        (u, same, counts(&a).unwrap(), volume(&a, 1e-8).unwrap(), counts(&b).unwrap())
    });
    let (u, same, ca, va, cb) = r;
    assert!(u.unwrap());
    assert_eq!(ca, [6, 12, 8]);
    assert_eq!(cb, [6, 12, 8]);
    assert!(rel(va, 1000.0) < 1e-12, "{va}");
    let v = same.unwrap().unwrap().unwrap();
    assert!(rel(v, 1000.0) < 1e-9, "A ∪ A: {v}");
}

/// Face and edge tags (`Brep::tagged`) through a boolean routed to solidcore, as on C++.
#[cfg(feature = "native")]
mod history {
    use crate::history::Raw;
    use crate::{BooleanOp, Brep, ffi};

    fn tagged_box(origin: [f64; 3], operand: usize) -> Brep {
        crate::init().unwrap();
        Brep::made(ffi::make_box(&origin, &[10.0; 3]).unwrap()).tagged(operand).unwrap()
    }

    /// Two 10 mm cubes overlapping in a 5 mm one (the capi's `overlapping_boxes_union`): 12
    /// faces, each from one face of one box, six from each.
    fn check(h: &Raw) {
        assert_eq!(h.faces.len(), 12, "{h:?}");
        assert!(h.faces.iter().all(|f| f.origins.len() == 1), "a face without exactly one origin: {h:?}");
        for operand in [0, 1] {
            assert_eq!(h.faces.iter().filter(|f| f.origins[0].0 == operand).count(), 6, "operand {operand}: {h:?}");
        }
    }

    #[test]
    fn tags_cross_a_cpp_boolean() {
        let (a, b) = (tagged_box([0.0; 3], 0), tagged_box([5.0; 3], 1));
        let r = Brep::made(ffi::ops::boolean(a.cpp().unwrap(), b.cpp().unwrap(), ffi::ops::BOOLEAN_UNION).unwrap());
        check(&r.history().unwrap());
    }

    #[test]
    #[ignore = "solidcore has no way to carry attributes through a boolean: scratchpad/bugs/boolean-operand-attributes-lost.md"]
    fn tags_cross_a_solidcore_boolean() {
        let (a, b) = (tagged_box([0.0; 3], 0), tagged_box([5.0; 3], 1));
        let r = super::boolean(a.core().unwrap(), b.core().unwrap(), BooleanOp::Union).unwrap().unwrap();
        check(&Brep::made_core(r).history().unwrap());
    }
}
