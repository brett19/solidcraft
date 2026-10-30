//! Stitch and merge faces on solidcore, against closed forms: counts, volumes (or areas) and
//! whether the result closes, and every result exports to a record that validates. With SMLib
//! built, the same input converted to the C++ kernel gives the same result or the same error
//! (parity only: the expected values are the analytic ones).
//!
//! A stitch's input is six (or five) separate planar faces of a 10 mm cube, each its own face of
//! one body with its own edges, none shared; a merge's is a cube with four faces cut across, or
//! two boxes unioned, on solidcore.

use std::collections::BTreeMap;

use solidcore::geom::extent2d::Extent2d;
use solidcore::geom::vector2d::Vec2;
use solidcore::geom::vector3d::Vec3;
use solidcore::surface::plane::p1::Plane;
use solidcore::surface::surface::Surface;
use solidcore::topology::brep::p3::new_brep;
use solidcore::topology::brep::p8::create_face_from_surface;

use super::merge::{merge_faces, stitch};
use super::sweep::build_faces;
use super::{Body, area, bounds, counts, make_box, run, to_arrays, volume};
use crate::export::native;
use crate::{BooleanOp, Error};

const TOL: f64 = 0.01;

/// The sides of the cube [0, 10]^3 as (origin, u axis, v axis), each over [0, 10]^2.
const SIDES: [([f64; 3], [f64; 3], [f64; 3]); 6] = [
    ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
    ([0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ([0.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
    ([10.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]),
    ([0.0, 10.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]),
    ([0.0, 0.0, 10.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]),
];

fn v3(p: [f64; 3]) -> Vec3 {
    Vec3::new(p[0], p[1], p[2])
}

/// The sides `sides` of the cube as separate faces of one body, the top lifted by `lift`.
fn loose_faces(sides: &[usize], lift: f64) -> Body {
    run("loose faces", |k| {
        let mut b = new_brep(k, solidcore::geom::tol::USE_DEFAULT, solidcore::geom::tol::USE_DEFAULT)?;
        for &i in sides {
            let Some(&(mut o, u, v)) = SIDES.get(i) else { continue };
            if i == 5 {
                o[2] += lift;
            }
            let plane = Plane::new(v3(o), v3(u), v3(v), Vec2::new(1.0, 1.0), &Extent2d::from_bounds(0.0, 0.0, 10.0, 10.0));
            let domain = Extent2d::from_bounds(0.0, 0.0, 10.0, 10.0);
            build_faces(&mut b, |b, fenv| create_face_from_surface(k, b, Some(Surface::from(plane)), &domain, fenv))?;
        }
        b.refresh_cached_topology();
        Ok(b)
    })
    .map(Body::new)
    .unwrap()
}

/// Two 10 mm cubes side by side along x, unioned: one 20 x 10 x 10 block.
fn two_boxes() -> Body {
    let a = make_box([0.0; 3], [10.0; 3]).unwrap();
    let b = make_box([10.0, 0.0, 0.0], [10.0; 3]).unwrap();
    super::boolean::boolean(&a, &b, BooleanOp::Union).unwrap().unwrap()
}

/// The cube with its four faces across x = 5 cut there (`split_faces`): ten faces.
fn split_cube() -> Body {
    let plane = crate::ssi::Spec::Plane { origin: [5.0, 0.0, 0.0], xaxis: [0.0, 1.0, 0.0], yaxis: [0.0, 0.0, 1.0], u: [-1.0, 11.0], v: [-1.0, 11.0] };
    let cube = make_box([0.0; 3], [10.0; 3]).unwrap();
    super::split::split_faces(&cube, &[0, 1, 2, 3, 4, 5], &[&plane]).unwrap().unwrap().0
}

#[derive(Clone, Copy, Debug)]
enum Input {
    SplitCube,
    /// [`loose_faces`]: which sides, the top's lift.
    Loose(&'static [usize], f64),
    TwoBoxes,
    Box,
}

impl Input {
    fn core(self) -> Body {
        match self {
            Input::Loose(sides, lift) => loose_faces(sides, lift),
            Input::TwoBoxes => two_boxes(),
            Input::SplitCube => split_cube(),
            Input::Box => make_box([0.0; 3], [10.0; 3]).unwrap(),
        }
    }
}

#[derive(Clone, Copy, Debug)]
enum Op {
    Stitch(Input, f64),
    Merge(Input, &'static [[f64; 3]], f64),
}

#[derive(Clone, Debug, PartialEq)]
enum Expect {
    /// A solid: volume, counts, closed.
    Solid(f64, [usize; 3], bool),
    /// A sheet: area, counts, closed.
    Sheet(f64, [usize; 3], bool),
    Fails(Error),
}

const ALL: &[usize] = &[0, 1, 2, 3, 4, 5];
const OPEN: &[usize] = &[0, 1, 2, 3, 4];

fn case(name: &str) -> (Op, Expect) {
    match name {
        "stitch_six_faces_into_a_cube" => (Op::Stitch(Input::Loose(ALL, 0.0), TOL), Expect::Solid(1000.0, [6, 12, 8], true)),
        "stitch_five_faces_into_an_open_box" => (Op::Stitch(Input::Loose(OPEN, 0.0), TOL), Expect::Sheet(500.0, [5, 12, 8], false)),
        // The top 0.05 mm above the walls: farther than the tolerance, so it stays apart.
        "stitch_leaves_a_gap_wider_than_the_tolerance" => (Op::Stitch(Input::Loose(ALL, 0.05), TOL), Expect::Sheet(600.0, [6, 16, 12], false)),
        // Within a 0.1 mm tolerance, the same gap closes.
        // (the top stays on its own plane, so the solid reaches up to it)
        "stitch_closes_a_gap_within_the_tolerance" => (Op::Stitch(Input::Loose(ALL, 0.05), 0.1), Expect::Solid(1005.0, [6, 12, 8], true)),
        "stitch_a_cube_already_sewn" => (Op::Stitch(Input::Box, TOL), Expect::Solid(1000.0, [6, 12, 8], true)),
        "merge_two_boxes_into_one_block" => (Op::Merge(Input::TwoBoxes, &[], TOL), Expect::Solid(2000.0, [6, 12, 8], false)),
        "merge_a_split_cube_into_one" => (Op::Merge(Input::SplitCube, &[], TOL), Expect::Solid(1000.0, [6, 12, 8], false)),
        "merge_a_box_changes_nothing" => (Op::Merge(Input::Box, &[], TOL), Expect::Solid(1000.0, [6, 12, 8], false)),
        "merge_keeps_a_picked_edge" => (Op::Merge(Input::SplitCube, &[[5.0, 5.0, 10.0]], TOL), Expect::Solid(1000.0, [7, 15, 10], false)),
        "merge_picks_near_nothing" => {
            (Op::Merge(Input::SplitCube, &[[5.0, 5.0, 10.0], [50.0, 50.0, 50.0]], TOL), Expect::Fails(Error::NoMatch { point: 1 }))
        }
        _ => panic!("no case {name}"),
    }
}

#[derive(Debug, PartialEq)]
struct Measured {
    size: f64,
    counts: [usize; 3],
    closed: bool,
    bounds: ([f64; 3], [f64; 3]),
    kinds: BTreeMap<String, usize>,
}

type Got = std::result::Result<Measured, Error>;

fn kinds(rec: &solidcraft_geom::brep::Brep) -> BTreeMap<String, usize> {
    let mut out = BTreeMap::new();
    for f in &rec.faces {
        let v = serde_json::to_value(&f.surface).unwrap();
        *out.entry(v["kind"].as_str().unwrap_or("?").to_owned()).or_default() += 1;
    }
    out
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

fn judge(name: &str, who: &str, got: Got) -> Got {
    let (op, expect) = case(name);
    match (&expect, &got) {
        (Expect::Fails(e), Err(g)) => assert_eq!(g, e, "{name}: {who} failed otherwise"),
        (Expect::Fails(e), Ok(m)) => panic!("{name}: {who} gave {m:?}, expected {e:?}"),
        (_, Err(g)) => panic!("{name}: {who} failed: {g}"),
        (Expect::Solid(s, n, closed) | Expect::Sheet(s, n, closed), Ok(m)) => {
            assert!(rel(m.size, *s) < 1e-9, "{name}: {who} size {} vs analytic {s}", m.size);
            assert_eq!(m.counts, *n, "{name}: {who} counts");
            if matches!(op, Op::Stitch(..)) {
                assert_eq!(m.closed, *closed, "{name}: {who} closed");
            }
        }
    }
    got
}

fn is_sheet(name: &str) -> bool {
    matches!(case(name).1, Expect::Sheet(..))
}

fn solidcore_case(name: &str) -> Got {
    let (op, _) = case(name);
    let r = match op {
        Op::Stitch(input, tol) => stitch(&input.core(), tol),
        Op::Merge(input, keep, tol) => merge_faces(&input.core(), keep, tol).map(|b| (b, false)),
    };
    let got = r.map(|(b, closed)| {
        let (rec, _) = native(&to_arrays(&b).unwrap()).unwrap_or_else(|e| panic!("{name}: record: {e:?}"));
        rec.validate().unwrap_or_else(|e| panic!("{name}: record: {e}"));
        let size = if is_sheet(name) { area(&b, 1e-8) } else { volume(&b, 1e-8) }.unwrap();
        Measured { size, counts: counts(&b).unwrap(), closed, bounds: bounds(&b).unwrap(), kinds: kinds(&rec) }
    });
    judge(name, "solidcore", got)
}

/// The case on the C++ kernel, called directly (whatever the routing), on the input converted
/// from solidcore's.
#[cfg(feature = "native")]
fn cpp_case(name: &str) -> Got {
    use crate::ops::pick_error;
    use crate::{Brep, error, ffi};
    crate::init().unwrap();
    let (op, _) = case(name);
    let r = match op {
        Op::Stitch(input, tol) => ffi::sheets::stitch(Brep::made_core(input.core()).cpp().unwrap(), tol).map_err(error),
        Op::Merge(input, keep, tol) => {
            ffi::ops::merge_faces(Brep::made_core(input.core()).cpp().unwrap(), keep, tol).map(|h| (h, false)).map_err(pick_error)
        }
    };
    let got = r.map(|(h, closed)| {
        let size = if is_sheet(name) { ffi::measure::area(&h, 1e-8) } else { ffi::volume(&h, 1e-8) }.unwrap();
        let n = ffi::counts(&h).unwrap().map(|x| usize::try_from(x).unwrap());
        let bx = ffi::measure::bounds(&h).unwrap();
        let (rec, _) = native(&Brep::made(h).cpp_arrays().unwrap()).unwrap();
        rec.validate().unwrap();
        Measured { size, counts: n, closed, bounds: bx, kinds: kinds(&rec) }
    });
    judge(name, "C++", got)
}

#[cfg(feature = "native")]
fn parity(name: &str) {
    match (solidcore_case(name), cpp_case(name)) {
        (Ok(a), Ok(b)) => {
            assert!(rel(a.size, b.size) < 1e-9, "{name}: solidcore {} vs C++ {}", a.size, b.size);
            assert_eq!(a.counts, b.counts, "{name}: counts (solidcore vs C++)");
            assert_eq!(a.closed, b.closed, "{name}: closed (solidcore vs C++)");
            assert_eq!(a.kinds, b.kinds, "{name}: faces by surface kind (solidcore vs C++)");
            let near = |x: [f64; 3], y: [f64; 3]| x.iter().zip(&y).all(|(p, q)| (p - q).abs() < 1e-6);
            assert!(near(a.bounds.0, b.bounds.0) && near(a.bounds.1, b.bounds.1), "{name}: bounds {:?} vs C++ {:?}", a.bounds, b.bounds);
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
    stitch_six_faces_into_a_cube,
    stitch_five_faces_into_an_open_box,
    stitch_leaves_a_gap_wider_than_the_tolerance,
    stitch_closes_a_gap_within_the_tolerance,
    stitch_a_cube_already_sewn,
    merge_two_boxes_into_one_block,
    merge_a_split_cube_into_one,
    merge_a_box_changes_nothing,
    merge_keeps_a_picked_edge,
    merge_picks_near_nothing,
);

/// The capi's argument checks, before any kernel call.
#[test]
fn hostile_arguments_are_refused() {
    let b = make_box([0.0; 3], [10.0; 3]).unwrap();
    for t in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e3 + 1.0] {
        assert_eq!(stitch(&b, t).err(), Some(Error::Argument), "stitch tolerance {t}");
    }
    for t in [0.0, -1.0, f64::NAN, f64::INFINITY, 1e6] {
        assert_eq!(merge_faces(&b, &[], t).err(), Some(Error::Argument), "merge tolerance {t}");
    }
    assert_eq!(merge_faces(&b, &[[f64::NAN, 0.0, 0.0]], TOL).err(), Some(Error::Argument));
    let many = vec![[0.0; 3]; crate::MAX_POINTS + 1];
    assert_eq!(merge_faces(&b, &many, TOL).err(), Some(Error::Argument));
}

/// The inputs are never changed.
#[test]
fn the_body_stays_as_it_was() {
    let loose = loose_faces(ALL, 0.0);
    let _ = stitch(&loose, TOL).unwrap();
    assert_eq!(counts(&loose).unwrap(), [6, 24, 24]);
    let block = two_boxes();
    let was = counts(&block).unwrap();
    let _ = merge_faces(&block, &[], TOL).unwrap();
    assert_eq!(counts(&block).unwrap(), was);
}

/// Face and edge tags (`Brep::tagged`) through a stitch and a merge routed to solidcore, as on
/// C++: the input is tagged on solidcore, converted to C++ and tagged the same way there.
#[cfg(feature = "native")]
mod history {
    use crate::{Brep, ffi};

    #[test]
    fn tags_cross_a_stitch() {
        crate::init().unwrap();
        let a = Brep::made_core(super::loose_faces(super::ALL, 0.0)).tagged(0).unwrap();
        let cpp = Brep::made(ffi::sheets::stitch(a.cpp().unwrap(), super::TOL).unwrap().0).history().unwrap();
        let core = Brep::made_core(super::stitch(a.core().unwrap(), super::TOL).unwrap().0).history().unwrap();
        assert_eq!(core.canonical(), cpp.canonical());
    }

    #[test]
    fn tags_cross_a_merge() {
        crate::init().unwrap();
        let a = Brep::made_core(super::split_cube()).tagged(0).unwrap();
        let cpp = Brep::made(ffi::ops::merge_faces(a.cpp().unwrap(), &[], super::TOL).unwrap()).history().unwrap();
        let core = Brep::made_core(super::merge_faces(a.core().unwrap(), &[], super::TOL).unwrap()).history().unwrap();
        assert_eq!(core.canonical(), cpp.canonical());
    }
}
