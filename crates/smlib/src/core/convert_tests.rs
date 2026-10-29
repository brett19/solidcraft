//! solidcore's primitives through the flat arrays: each exports to a valid record of the right
//! volume, the record comes back to an equal body, and (with SMLib built) the arrays are the C++
//! kernel's own. A body with a sealed void crosses with each region's enclosing shell first,
//! whatever order its arrays list the shells in (the capi's `PutEnclosingShellsFirst`).

use std::f64::consts::PI;

use solidcraft_geom::brep as b;

use super::boolean::boolean;
use super::{Body, from_arrays, make_box, make_cone, make_cylinder, make_sphere, make_torus, to_arrays, volume};
use crate::BooleanOp;
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
    use crate::flat::{TOLERANCE, VERTEX_TOLERANCE, WIRE_EDGE_NURBS_FORM};
    use crate::{Brep, ffi};
    // built by the C API whatever `Op::Primitive` routes to
    crate::init().unwrap();
    let cpp = match name {
        "box" => ffi::make_box(&[1.0, -2.0, 3.0], &[10.0, 20.0, 30.0]),
        "cylinder" => ffi::make_cylinder(&[1.0, 2.0, -3.0], 5.0, 12.0),
        "sphere" => ffi::make_sphere(&[1.0, 2.0, 3.0], 7.0),
        "cone" => ffi::make_cone(&[0.0, 1.0, 2.0], 6.0, 0.0, 9.0),
        "frustum" => ffi::make_cone(&[0.0, 1.0, 2.0], 6.0, 2.0, 9.0),
        "torus" => ffi::make_torus(&[1.0, -1.0, 0.5], 10.0, 3.0),
        _ => panic!("{name}"),
    }
    .map(Brep::made)
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
primitive!(cylinder, "cylinder");
primitive!(sphere, "sphere");
primitive!(cone, "cone");
primitive!(frustum, "frustum");
primitive!(torus, "torus");

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
    crate::init().unwrap();
    let cpp = crate::ffi::make_box(&[1.0, -2.0, 3.0], &[10.0, 20.0, 30.0]).map(crate::Brep::made).unwrap();
    let body = from_arrays(&cpp.cpp_arrays().unwrap()).unwrap();
    let v = volume(&body, 1e-8).unwrap();
    assert!(rel(v, 6000.0) < 1e-9, "{v}");
    record(&to_arrays(&body).unwrap());
}

/// Each edge's and vertex's tolerance crosses the kernels as it is. One box edge given eight times
/// the box's tolerance, in arrays whose TOLERANCE is the largest (as the C++ kernel writes them),
/// keeps it round after round, and the B-rep and every other edge and vertex keep the box's. Every
/// crossing used to give every entity the largest, so a chain of blends on a mixed route grew the
/// whole body's tolerance fourfold per blend (fillets/2341).
#[test]
fn tolerances_cross_the_arrays_per_entity() {
    use crate::flat::{EDGE_TOLERANCE, TOLERANCE, VERTEX_TOLERANCE};
    let mut a = to_arrays(&case("box").body).unwrap();
    let base = a.reals[&TOLERANCE][0];
    assert!(a.reals[&EDGE_TOLERANCE].iter().chain(&a.reals[&VERTEX_TOLERANCE]).all(|&t| t == base), "{a:?}");
    a.reals.get_mut(&EDGE_TOLERANCE).unwrap()[0] = 8.0 * base;
    a.reals.get_mut(&TOLERANCE).unwrap()[0] = 8.0 * base;
    for round in 0..3 {
        let body = from_arrays(&a).unwrap();
        assert_eq!(body.0.lock().unwrap().tolerance(), base / 2.0, "round {round}");
        a = to_arrays(&body).unwrap();
        assert_eq!(a.reals[&TOLERANCE], [base], "round {round}");
        let edges = &a.reals[&EDGE_TOLERANCE];
        assert_eq!(edges[0], 8.0 * base, "round {round}");
        assert!(edges[1..].iter().chain(&a.reals[&VERTEX_TOLERANCE]).all(|&t| t == base), "round {round}: {a:?}");
        // and through the C++ kernel, which keeps them too
        #[cfg(feature = "native")]
        {
            let cpp = crate::flat::Flat::from_arrays(&a).unwrap().import().unwrap().cpp_arrays().unwrap();
            assert_eq!((&cpp.reals[&EDGE_TOLERANCE], &cpp.reals[&VERTEX_TOLERANCE]), (edges, &a.reals[&VERTEX_TOLERANCE]), "round {round}");
            a = cpp;
        }
    }
}

/// A 20 mm box minus a centred 10 mm box: a sealed void, so its solid region has two shells.
fn sealed_void() -> Body {
    boolean(&make_box([-10.0; 3], [20.0; 3]).unwrap(), &make_box([-5.0; 3], [10.0; 3]).unwrap(), BooleanOp::Subtract).unwrap().unwrap()
}

/// 20³ − 10³.
const SEALED_VOID_VOLUME: f64 = 8000.0 - 1000.0;

/// Each bounded region's shells, as each shell's reach: the largest |coordinate| of its vertices
/// (10 for the 20 mm box, 5 for the 10 mm one).
fn reaches(rec: &b::Brep) -> Vec<Vec<f64>> {
    let reach = |s: u32| {
        let Some(b::Shell::Faces { faces, .. }) = rec.shells.get(s as usize) else { panic!("shell {s}") };
        let mut r = 0.0f64;
        for f in faces {
            for l in &rec.faces[f.face as usize].loops {
                let b::Loop::Edges { edges } = l else { continue };
                for e in edges {
                    for &v in &rec.edges[e.edge as usize].vertices {
                        r = rec.vertices[v as usize].point.iter().fold(r, |r, c| r.max(c.abs()));
                    }
                }
            }
        }
        r
    };
    rec.regions.iter().skip(1).map(|r| r.shells.iter().map(|&s| reach(s)).collect()).collect()
}

/// The sealed void's record: the solid region lists the outer box first, then the void's box;
/// the void region its box alone.
fn outer_first(a: &Arrays, at: &str) {
    let mut got = reaches(&record(a));
    got.sort_by_key(|x| std::cmp::Reverse(x.len()));
    assert_eq!(got, [vec![10.0, 5.0], vec![5.0]], "{at}");
}

/// The arrays with every region's shells listed in reverse: each shell's block (its counts and
/// its face uses) moved, nothing else touched.
fn shells_reversed(a: &Arrays) -> Arrays {
    use crate::flat::{FACEUSE_FACE, FACEUSE_ORIENTATION, REGION_SHELL_COUNT, SHELL_FACEUSE_COUNT, SHELL_POINT_TYPE, SHELL_WIRE_EDGE_COUNT};
    let mut p = a.clone();
    let blocks = [SHELL_FACEUSE_COUNT, SHELL_WIRE_EDGE_COUNT, SHELL_POINT_TYPE, FACEUSE_FACE, FACEUSE_ORIENTATION];
    for id in blocks {
        p.ints.get_mut(&id).unwrap().clear();
    }
    let (mut shell, mut fu) = (0usize, 0usize);
    for &n in &a.ints[&REGION_SHELL_COUNT] {
        let mut region = Vec::new();
        for _ in 0..n {
            let uses = usize::try_from(a.ints[&SHELL_FACEUSE_COUNT][shell]).unwrap();
            region.push((shell, fu..fu + uses));
            shell += 1;
            fu += uses;
        }
        for (s, uses) in region.into_iter().rev() {
            for id in [SHELL_FACEUSE_COUNT, SHELL_WIRE_EDGE_COUNT, SHELL_POINT_TYPE] {
                let v = a.ints[&id][s];
                p.ints.get_mut(&id).unwrap().push(v);
            }
            for id in [FACEUSE_FACE, FACEUSE_ORIENTATION] {
                p.ints.get_mut(&id).unwrap().extend_from_slice(&a.ints[&id][uses.clone()]);
            }
        }
    }
    assert_ne!(p.ints, a.ints, "nothing to reverse");
    p
}

/// solidcore's sealed void exports with its outer shell first, comes back through the arrays
/// with it first, and keeps its volume.
#[test]
fn a_sealed_void_crosses_the_arrays_outer_shell_first() {
    let body = sealed_void();
    let v = volume(&body, 1e-9).unwrap();
    assert!(rel(v, SEALED_VOID_VOLUME) < 1e-9, "{v}");
    let a = to_arrays(&body).unwrap();
    outer_first(&a, "export");
    let back = from_arrays(&import::arrays(&record(&a), true).unwrap()).unwrap();
    outer_first(&to_arrays(&back).unwrap(), "round trip");
    let v = volume(&back, 1e-9).unwrap();
    assert!(rel(v, SEALED_VOID_VOLUME) < 1e-9, "round trip: {v}");
}

/// Arrays listing the solid region's void shell before the box around it import with the box
/// first (SMLib takes a region's first shell as its enclosing one), the volume unchanged.
#[test]
fn shells_listed_inner_first_import_outer_first() {
    let a = shells_reversed(&to_arrays(&sealed_void()).unwrap());
    let back = from_arrays(&a).unwrap();
    outer_first(&to_arrays(&back).unwrap(), "inner first");
    let v = volume(&back, 1e-9).unwrap();
    assert!(rel(v, SEALED_VOID_VOLUME) < 1e-9, "inner first: {v}");
}

/// The C++ kernel's sealed void: solidcore imports its arrays with the outer shell first, in
/// either shell order, at the C++ kernel's volume, and the C++ kernel reads solidcore's arrays
/// (either order) back as a body that passes its checks at that volume.
#[cfg(feature = "native")]
#[test]
fn a_sealed_void_matches_the_cpp_kernel() {
    use crate::{Brep, ffi};
    crate::init().unwrap();
    let make = |o: f64, s: f64| ffi::make_box(&[o; 3], &[s; 3]).map(Brep::made).unwrap();
    let (outer, inner) = (make(-10.0, 20.0), make(-5.0, 10.0));
    let cpp = Brep::made(ffi::ops::boolean(outer.cpp().unwrap(), inner.cpp().unwrap(), ffi::ops::BOOLEAN_SUBTRACT).unwrap());
    let theirs = ffi::volume(cpp.cpp().unwrap(), 1e-9).unwrap();
    assert!(rel(theirs, SEALED_VOID_VOLUME) < 1e-9, "C++ {theirs}");
    let arrays = cpp.cpp_arrays().unwrap();
    outer_first(&arrays, "C++ export");
    for (at, a) in [("C++ arrays", arrays.clone()), ("C++ arrays, inner first", shells_reversed(&arrays))] {
        let body = from_arrays(&a).unwrap();
        outer_first(&to_arrays(&body).unwrap(), at);
        let v = volume(&body, 1e-9).unwrap();
        assert!(rel(v, theirs) < 1e-9, "{at}: solidcore {v} vs C++ {theirs}");
    }
    let ours = to_arrays(&sealed_void()).unwrap();
    for (at, a) in [("solidcore arrays", ours.clone()), ("solidcore arrays, inner first", shells_reversed(&ours))] {
        let h = crate::flat::Flat::from_arrays(&a).unwrap().import().unwrap();
        let v = ffi::volume(h.cpp().unwrap(), 1e-9).unwrap();
        assert!(rel(v, theirs) < 1e-9, "{at}: C++ {v} vs {theirs}");
        let mut buf = vec![0u8; 1 << 16];
        let (len, failures) = ffi::check::check(h.cpp().unwrap(), 2, &mut buf).unwrap();
        assert_eq!(failures, 0, "{at}: {}", String::from_utf8_lossy(&buf[..len]));
        outer_first(&h.cpp_arrays().unwrap(), &format!("{at}, C++ export"));
    }
}
