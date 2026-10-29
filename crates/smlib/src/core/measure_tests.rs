//! The Measure and Check families on solidcore's primitives: mass properties, area and bounds
//! against their closed forms ([`crate::measure::closed`]), one solid each, edges on their faces;
//! and (with SMLib built) the same answers as the C++ kernel for the same primitive.

use super::{Body, area, bounds, check, edge_face_distance, is_manifold_solid, make_box, make_cone, make_cylinder, make_sphere, make_torus};
use super::{counts, mass_properties, solid_count};
use crate::check::CheckLevel;
use crate::measure::MEASURE_ACCURACY;
use crate::measure::closed::{self, Closed};

/// The tolerance every quantity meets, relative.
const TOL: f64 = 1e-9;

/// The primitives, as `convert_tests` makes them, with their closed forms.
fn case(name: &str) -> (Body, Closed) {
    let (body, c) = match name {
        "box" => (make_box([1.0, -2.0, 3.0], [10.0, 20.0, 30.0]), closed::cuboid([1.0, -2.0, 3.0], [10.0, 20.0, 30.0])),
        "cylinder" => (make_cylinder([1.0, 2.0, -3.0], 5.0, 12.0), closed::cylinder([1.0, 2.0, -3.0], 5.0, 12.0)),
        "sphere" => (make_sphere([1.0, 2.0, 3.0], 7.0), closed::sphere([1.0, 2.0, 3.0], 7.0)),
        "cone" => (make_cone([0.0, 1.0, 2.0], 6.0, 0.0, 9.0), closed::cone([0.0, 1.0, 2.0], 6.0, 0.0, 9.0)),
        "frustum" => (make_cone([0.0, 1.0, 2.0], 6.0, 2.0, 9.0), closed::cone([0.0, 1.0, 2.0], 6.0, 2.0, 9.0)),
        "torus" => (make_torus([1.0, -1.0, 0.5], 10.0, 3.0), closed::torus([1.0, -1.0, 0.5], 10.0, 3.0)),
        _ => panic!("{name}"),
    };
    (body.unwrap(), c)
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// The body's size: twice its largest radius of gyration, the scale of centroid errors.
fn size(c: &Closed) -> f64 {
    2.0 * (c.moments.iter().fold(0.0f64, |a, &b| a.max(b)) / c.volume).sqrt()
}

fn mass_properties_match(name: &str) {
    let (b, c) = case(name);
    let m = mass_properties(&b, MEASURE_ACCURACY).unwrap();
    assert!(rel(m.area, c.area) < TOL, "{name}: area {} vs {}", m.area, c.area);
    assert!(rel(m.volume, c.volume) < TOL, "{name}: volume {} vs {}", m.volume, c.volume);
    let imax = c.moments.iter().fold(0.0f64, |a, &b| a.max(b));
    for i in 0..3 {
        assert!((m.centroid[i] - c.centroid[i]).abs() < TOL * size(&c), "{name}: centroid {:?} vs {:?}", m.centroid, c.centroid);
        assert!(rel(m.moments[i], c.moments[i]) < TOL, "{name}: moments {:?} vs {:?}", m.moments, c.moments);
        assert!(m.products[i].abs() < TOL * imax, "{name}: products {:?}", m.products);
    }
}

fn area_matches(name: &str) {
    let (b, c) = case(name);
    let a = area(&b, MEASURE_ACCURACY).unwrap();
    assert!(rel(a, c.area) < TOL, "{name}: area {a} vs {}", c.area);
}

fn bounds_match(name: &str) {
    let (b, c) = case(name);
    let (lo, hi) = bounds(&b).unwrap();
    let s = size(&c);
    for i in 0..3 {
        assert!((lo[i] - c.bounds.0[i]).abs() < TOL * s, "{name}: low {lo:?} vs {:?}", c.bounds.0);
        assert!((hi[i] - c.bounds.1[i]).abs() < TOL * s, "{name}: high {hi:?} vs {:?}", c.bounds.1);
    }
}

fn one_solid(name: &str) {
    let (b, _) = case(name);
    assert_eq!(solid_count(&b).unwrap(), 1, "{name}");
}

fn manifold_solid(name: &str) {
    let (b, _) = case(name);
    assert!(is_manifold_solid(&b).unwrap(), "{name}");
}

/// Every edge lies on the faces it bounds: its distance to the nearest face is 0.
fn edges_lie_on_their_faces(name: &str) {
    let (b, c) = case(name);
    let [faces, edges, _] = counts(&b).unwrap();
    for e in 0..edges {
        let mut d: Vec<f64> = (0..faces).filter_map(|f| edge_face_distance(&b, e, f).ok()).collect();
        d.sort_by(f64::total_cmp);
        assert!(d.first().is_some_and(|&d| d < TOL * size(&c)), "{name}: edge {e}: {d:?}");
    }
    assert!(edge_face_distance(&b, edges, 0).is_err() && edge_face_distance(&b, 0, faces).is_err());
}

fn checks_valid(name: &str) {
    let (b, _) = case(name);
    for level in [CheckLevel::Basic, CheckLevel::Geometry, CheckLevel::All] {
        let r = check(&b, level).unwrap();
        assert!(r.is_valid(), "{name}, {level:?}: {r}");
    }
}

/// The C++ kernel's body of the same primitive, built by its C API whatever `Op::Primitive`
/// routes to: a body imported from solidcore's arrays is not the C++ kernel's own (its
/// `edge_face_distance` measures to each face's surface over its domain, which the import sets).
#[cfg(feature = "native")]
fn cpp(name: &str) -> crate::Brep {
    use crate::{Brep, ffi};
    crate::init().unwrap();
    match name {
        "box" => ffi::make_box(&[1.0, -2.0, 3.0], &[10.0, 20.0, 30.0]),
        "cylinder" => ffi::make_cylinder(&[1.0, 2.0, -3.0], 5.0, 12.0),
        "sphere" => ffi::make_sphere(&[1.0, 2.0, 3.0], 7.0),
        "cone" => ffi::make_cone(&[0.0, 1.0, 2.0], 6.0, 0.0, 9.0),
        "frustum" => ffi::make_cone(&[0.0, 1.0, 2.0], 6.0, 2.0, 9.0),
        "torus" => ffi::make_torus(&[1.0, -1.0, 0.5], 10.0, 3.0),
        _ => panic!("{name}"),
    }
    .map(Brep::made)
    .unwrap()
}

/// `x` within `TOL` of the C++ kernel's `y`, relative to `scale`.
#[cfg(feature = "native")]
fn same(x: f64, y: f64, scale: f64, what: &str) {
    assert!((x - y).abs() <= TOL * scale, "{what}: solidcore {x} vs C++ {y}");
}

#[cfg(feature = "native")]
fn mass_properties_parity(name: &str) {
    let (b, c) = case(name);
    let ours = mass_properties(&b, MEASURE_ACCURACY).unwrap();
    let theirs = cpp(name).mass_properties(MEASURE_ACCURACY).unwrap();
    same(ours.area, theirs.area, theirs.area, &format!("{name} area"));
    same(ours.volume, theirs.volume, theirs.volume, &format!("{name} volume"));
    let imax = theirs.moments.iter().fold(0.0f64, |a, &b| a.max(b));
    for i in 0..3 {
        same(ours.centroid[i], theirs.centroid[i], size(&c), &format!("{name} centroid[{i}]"));
        same(ours.moments[i], theirs.moments[i], theirs.moments[i], &format!("{name} moments[{i}]"));
        same(ours.products[i], theirs.products[i], imax, &format!("{name} products[{i}]"));
    }
}

#[cfg(feature = "native")]
fn area_parity(name: &str) {
    let (b, _) = case(name);
    let theirs = cpp(name).area(MEASURE_ACCURACY).unwrap();
    same(area(&b, MEASURE_ACCURACY).unwrap(), theirs, theirs, &format!("{name} area"));
}

#[cfg(feature = "native")]
fn bounds_parity(name: &str) {
    let (b, c) = case(name);
    let (lo, hi) = bounds(&b).unwrap();
    let (tlo, thi) = cpp(name).bounds().unwrap();
    for i in 0..3 {
        same(lo[i], tlo[i], size(&c), &format!("{name} low[{i}]"));
        same(hi[i], thi[i], size(&c), &format!("{name} high[{i}]"));
    }
}

#[cfg(feature = "native")]
fn solid_count_parity(name: &str) {
    let (b, _) = case(name);
    assert_eq!(solid_count(&b).unwrap(), cpp(name).solid_count().unwrap(), "{name}");
}

#[cfg(feature = "native")]
fn manifold_parity(name: &str) {
    let (b, _) = case(name);
    assert_eq!(is_manifold_solid(&b).unwrap(), cpp(name).is_manifold_solid().unwrap(), "{name}");
}

/// The whole edge-by-face distance matrix, the C++ kernel's own (both bodies list their edges
/// and faces in the same order: `convert_tests` shows their arrays are equal).
#[cfg(feature = "native")]
fn distance_parity(name: &str) {
    let (b, c) = case(name);
    let theirs = cpp(name);
    let [faces, edges, _] = counts(&b).unwrap();
    for e in 0..edges {
        for f in 0..faces {
            let (x, y) = (edge_face_distance(&b, e, f), theirs.edge_face_distance(e, f));
            match (x, y) {
                (Ok(x), Ok(y)) => same(x, y, size(&c), &format!("{name} edge {e} face {f}")),
                (x, y) => assert_eq!(x.is_ok(), y.is_ok(), "{name} edge {e} face {f}: {x:?} vs {y:?}"),
            }
        }
    }
}

#[cfg(feature = "native")]
fn check_parity(name: &str) {
    let (b, _) = case(name);
    for level in [CheckLevel::Basic, CheckLevel::Geometry, CheckLevel::All] {
        assert_eq!(check(&b, level).unwrap(), cpp(name).check(level).unwrap(), "{name}, {level:?}");
    }
}

macro_rules! primitive {
    ($m:ident, $name:literal $(, mass: $mass:meta)? $(, area: $area:meta)?) => {
        mod $m {
            use super::*;
            #[test]
            $(#[$mass])?
            fn mass_properties_match_the_closed_forms() {
                mass_properties_match($name);
            }
            #[test]
            $(#[$area])?
            fn area_matches_the_closed_form() {
                area_matches($name);
            }
            #[test]
            fn bounds_match_the_closed_form() {
                bounds_match($name);
            }
            #[test]
            fn is_one_solid() {
                one_solid($name);
            }
            #[test]
            fn is_a_manifold_solid() {
                manifold_solid($name);
            }
            #[test]
            fn edges_lie_on_their_faces() {
                super::edges_lie_on_their_faces($name);
            }
            #[test]
            #[ignore = "solidcore#1419: no topology-level SmBrep::AssertValid"]
            fn checks_valid() {
                super::checks_valid($name);
            }
            #[cfg(feature = "native")]
            #[test]
            $(#[$mass])?
            fn mass_properties_match_the_cpp_kernel() {
                mass_properties_parity($name);
            }
            #[cfg(feature = "native")]
            #[test]
            $(#[$area])?
            fn area_matches_the_cpp_kernel() {
                area_parity($name);
            }
            #[cfg(feature = "native")]
            #[test]
            fn bounds_match_the_cpp_kernel() {
                bounds_parity($name);
            }
            #[cfg(feature = "native")]
            #[test]
            fn solid_count_matches_the_cpp_kernel() {
                solid_count_parity($name);
            }
            #[cfg(feature = "native")]
            #[test]
            fn manifold_matches_the_cpp_kernel() {
                manifold_parity($name);
            }
            #[cfg(feature = "native")]
            #[test]
            fn distances_match_the_cpp_kernel() {
                distance_parity($name);
            }
            #[cfg(feature = "native")]
            #[test]
            #[ignore = "solidcore#1419: no topology-level SmBrep::AssertValid"]
            fn check_matches_the_cpp_kernel() {
                check_parity($name);
            }
        }
    };
}

primitive!(box_, "box");
primitive!(cylinder, "cylinder", area: ignore = "solidcore#1435: face area at 1e-8 is 3.7e-8 off");
primitive!(sphere, "sphere",
    mass: ignore = "solidcore#1435: a sphere's properties at 1e-8 miss 1e-9",
    area: ignore = "solidcore#1435: face area at 1e-8 is 1.2e-8 off");
primitive!(cone, "cone",
    mass: ignore = "solidcore#1435: cone inertia at 1e-8 is 1.7e-9 off",
    area: ignore = "solidcore#1435: face area at 1e-8 is 4.2e-8 off");
primitive!(frustum, "frustum",
    mass: ignore = "solidcore#1435: frustum inertia at 1e-8 misses 1e-9",
    area: ignore = "solidcore#1435: face area at 1e-8 is 3.5e-8 off");
primitive!(torus, "torus",
    mass: ignore = "solidcore#1435: torus inertia at 1e-8 is 1.7e-9 off",
    area: ignore = "solidcore#1435: face area at 1e-8 is 6.0e-9 off");

/// Until solidcore ports it (solidcore#1419), the check says so instead of guessing.
#[test]
fn unported_calls_say_not_supported_yet() {
    let (b, _) = case("box");
    let e = check(&b, CheckLevel::Basic).unwrap_err();
    assert!(e.to_string().contains("not supported yet"), "{e}");
}

#[test]
fn hostile_numbers_are_refused() {
    let (b, _) = case("box");
    assert_eq!(mass_properties(&b, f64::NAN).unwrap_err(), crate::Error::Argument);
    assert_eq!(area(&b, f64::INFINITY).unwrap_err(), crate::Error::Argument);
    assert_eq!(edge_face_distance(&b, usize::MAX, 0).unwrap_err(), crate::Error::Argument);
    // Out of range is clamped, not refused.
    assert!(rel(mass_properties(&b, 1e-30).unwrap().volume, 6000.0) < TOL);
    assert!(rel(mass_properties(&b, 10.0).unwrap().volume, 6000.0) < TOL);
}
