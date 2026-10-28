//! solidcore's primitives through the flat arrays: each exports to a valid record of the right
//! volume, the record comes back to an equal body, and (with SMLib built) the arrays are the C++
//! kernel's own.

use std::f64::consts::PI;

use solidcraft_geom::brep as b;

use super::{Body, from_arrays, make_box, make_cone, make_cylinder, make_sphere, make_torus, to_arrays, volume};
use crate::export::native;
use crate::import::{self, Arrays};

/// A primitive: solidcore's body and its analytic volume.
struct Case {
    body: Body,
    volume: f64,
}

fn case(name: &str) -> Case {
    let (body, volume) = match name {
        "box" => (make_box([1.0, -2.0, 3.0], [10.0, 20.0, 30.0]), 6000.0),
        "cylinder" => (make_cylinder([1.0, 2.0, -3.0], 5.0, 12.0), PI * 25.0 * 12.0),
        "sphere" => (make_sphere([1.0, 2.0, 3.0], 7.0), 4.0 / 3.0 * PI * 343.0),
        "cone" => (make_cone([0.0, 1.0, 2.0], 6.0, 0.0, 9.0), PI * 36.0 * 9.0 / 3.0),
        "frustum" => (make_cone([0.0, 1.0, 2.0], 6.0, 2.0, 9.0), PI * 9.0 / 3.0 * (36.0 + 12.0 + 4.0)),
        "torus" => (make_torus([1.0, -1.0, 0.5], 10.0, 3.0), 2.0 * PI * PI * 10.0 * 9.0),
        _ => panic!("{name}"),
    };
    Case { body: body.unwrap(), volume }
}

fn rel(a: f64, b: f64) -> f64 {
    (a - b).abs() / b.abs()
}

/// The record of solidcore's arrays, validated and its geometry checked.
fn record(a: &Arrays) -> b::Brep {
    let (rec, _) = native(a).unwrap();
    rec.validate().unwrap();
    rec.check_geometry().unwrap();
    rec
}

/// Every number in `a` within `tol` (relative past 1) of `b`'s, everything else equal.
fn close(a: &serde_json::Value, b: &serde_json::Value, tol: f64, at: &str) {
    use serde_json::Value as V;
    match (a, b) {
        (V::Number(x), V::Number(y)) => {
            let (x, y) = (x.as_f64().unwrap(), y.as_f64().unwrap());
            assert!((x - y).abs() <= tol * x.abs().max(1.0), "{at}: {x} vs {y}");
        }
        (V::Array(x), V::Array(y)) => {
            assert_eq!(x.len(), y.len(), "{at}: length");
            for (i, (x, y)) in x.iter().zip(y).enumerate() {
                close(x, y, tol, &format!("{at}[{i}]"));
            }
        }
        (V::Object(x), V::Object(y)) => {
            assert_eq!(x.keys().collect::<Vec<_>>(), y.keys().collect::<Vec<_>>(), "{at}: keys");
            for (k, x) in x {
                close(x, &y[k], tol, &format!("{at}.{k}"));
            }
        }
        _ => assert_eq!(a, b, "{at}"),
    }
}

/// to_arrays gives a record that validates and whose geometry checks.
fn exports(name: &str) {
    record(&to_arrays(&case(name).body).unwrap());
}

/// solidcore's body measures its analytic volume.
fn measures(name: &str) {
    let c = case(name);
    let v = volume(&c.body, 1e-8).unwrap();
    assert!(rel(v, c.volume) < 1e-9, "{name}: solidcore volume {v} vs {}", c.volume);
}

/// record -> arrays -> from_arrays -> to_arrays -> record is the record, the volume exact.
fn round_trips(name: &str) {
    let c = case(name);
    let rec = record(&to_arrays(&c.body).unwrap());
    let back = from_arrays(&import::arrays(&rec, true).unwrap()).unwrap();
    let again = record(&to_arrays(&back).unwrap());
    close(&serde_json::to_value(&rec).unwrap(), &serde_json::to_value(&again).unwrap(), 1e-12, name);
    let v = volume(&back, 1e-8).unwrap();
    assert!(rel(v, c.volume) < 1e-9, "{name}: {v} vs {}", c.volume);
}

/// The C++ kernel's arrays of the same primitive: every id, ints exact, reals within 1e-12.
#[cfg(feature = "native")]
fn matches_cpp(name: &str) {
    use crate::Brep;
    use crate::flat::{TOLERANCE, VERTEX_TOLERANCE, WIRE_EDGE_NURBS_FORM};
    let cpp = match name {
        "box" => Brep::make_box([1.0, -2.0, 3.0], [10.0, 20.0, 30.0]),
        "cylinder" => Brep::make_cylinder([1.0, 2.0, -3.0], 5.0, 12.0),
        "sphere" => Brep::make_sphere([1.0, 2.0, 3.0], 7.0),
        "cone" => Brep::make_cone([0.0, 1.0, 2.0], 6.0, 0.0, 9.0),
        "frustum" => Brep::make_cone([0.0, 1.0, 2.0], 6.0, 2.0, 9.0),
        "torus" => Brep::make_torus([1.0, -1.0, 0.5], 10.0, 3.0),
        _ => panic!("{name}"),
    }
    .unwrap();
    let flat = cpp.export().unwrap();
    let ours = to_arrays(&case(name).body).unwrap();
    for id in 1..=WIRE_EDGE_NURBS_FORM {
        assert_eq!(ours.ints.get(&id).unwrap(), &flat.get_i64(id).unwrap(), "{name}: int array {id}");
    }
    for id in TOLERANCE..=VERTEX_TOLERANCE {
        let (a, b) = (ours.reals.get(&id).unwrap(), flat.get_f64(id).unwrap());
        assert_eq!(a.len(), b.len(), "{name}: real array {id} length");
        for (i, (x, y)) in a.iter().zip(&b).enumerate() {
            assert!((x - y).abs() <= 1e-12 * x.abs().max(1.0), "{name}: real array {id}[{i}]: {x} vs {y}");
        }
    }
}

/// The record of solidcore's arrays, built in the C++ kernel, measures the analytic volume.
#[cfg(feature = "native")]
fn record_measures_in_cpp(name: &str) {
    let c = case(name);
    let rec = record(&to_arrays(&c.body).unwrap());
    let v = crate::Brep::from_record(&rec, true).unwrap().volume(1e-8).unwrap();
    assert!(rel(v, c.volume) < 1e-9, "{name}: {v} vs {}", c.volume);
}

macro_rules! primitive {
    ($m:ident, $name:literal $(, volume: $vol:meta)? $(, round_trip: $rt:meta)?) => {
        mod $m {
            #[test]
            fn exports_a_valid_record() {
                super::exports($name);
            }
            #[test]
            $(#[$vol])?
            fn measures_its_volume() {
                super::measures($name);
            }
            #[test]
            $(#[$rt])?
            fn round_trips_through_the_record() {
                super::round_trips($name);
            }
            #[cfg(feature = "native")]
            #[test]
            fn its_record_measures_in_the_cpp_kernel() {
                super::record_measures_in_cpp($name);
            }
            #[cfg(feature = "native")]
            #[test]
            fn arrays_match_the_cpp_kernel() {
                super::matches_cpp($name);
            }
        }
    };
}

primitive!(box_, "box");
primitive!(cylinder, "cylinder", round_trip: ignore = "solidcore: no SmCylinder::IsNurbSurfaceCylinder (nor SmCircle::IsNurbCurveCircle)");
primitive!(sphere, "sphere",
    volume: ignore = "solidcore: brep_volume of a sphere fails with SM_ERR at accuracy 1e-8",
    round_trip: ignore = "solidcore: no SmCircle::IsNurbCurveCircle");
primitive!(cone, "cone", round_trip: ignore = "solidcore: no SmCircle::IsNurbCurveCircle");
primitive!(frustum, "frustum", round_trip: ignore = "solidcore: no SmCircle::IsNurbCurveCircle");
primitive!(torus, "torus", round_trip: ignore = "solidcore: no SmTorus::IsNurbSurfaceTorus (nor SmCircle::IsNurbCurveCircle)");

#[test]
fn hostile_arrays_are_refused() {
    let c = case("box");
    let good = to_arrays(&c.body).unwrap();
    assert!(from_arrays(&Arrays::default()).is_err());
    for id in good.ints.keys().copied().collect::<Vec<_>>() {
        let mut a = good.clone();
        if let Some(v) = a.ints.get_mut(&id) {
            v.push(7);
        }
        let _ = from_arrays(&a);
        let mut a = good.clone();
        if let Some(v) = a.ints.get_mut(&id).and_then(|v| v.first_mut()) {
            *v = i64::MAX;
        }
        let _ = from_arrays(&a);
    }
    for id in good.reals.keys().copied().collect::<Vec<_>>() {
        let mut a = good.clone();
        if let Some(v) = a.reals.get_mut(&id).and_then(|v| v.first_mut()) {
            *v = f64::NAN;
            assert!(from_arrays(&a).is_err(), "NaN in {id}");
        }
    }
}

/// The C++ kernel's own box arrays (what `Brep::core` converts) build in solidcore.
#[cfg(feature = "native")]
#[test]
fn a_cpp_box_comes_into_solidcore() {
    let cpp = crate::Brep::make_box([1.0, -2.0, 3.0], [10.0, 20.0, 30.0]).unwrap();
    let body = from_arrays(&cpp.cpp_arrays().unwrap()).unwrap();
    let v = volume(&body, 1e-8).unwrap();
    assert!(rel(v, 6000.0) < 1e-9, "{v}");
    record(&to_arrays(&body).unwrap());
}
