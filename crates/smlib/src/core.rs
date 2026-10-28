//! The solidcore backend: one kernel for the process (SMLib's context is process-global too, and
//! some of its state, such as the fillet solver's latched step, lives for the whole process),
//! every call under its lock and a panic guard.
//!
//! Upstream fixes ([`fixes`]) are solidcore's opt-in corrections of SMLib defects that the C++
//! fork fixed in its own sources; each one turned on here says which fork commit it matches.

use std::panic::{AssertUnwindSafe, catch_unwind};
use std::sync::{Mutex, PoisonError};

use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::vector3d::Vec3;
use solidcore::kernel::{Kernel, KernelOptions, UpstreamFixes};
use solidcore::mass::integrator::properties::{PropertyAttributes, brep_volume};
use solidcore::primitive::api_primitives as primitives;

use crate::{Error, Result};

pub(crate) mod boolean;
#[cfg(test)]
mod boolean_tests;
mod convert;
#[cfg(test)]
mod convert_tests;

pub(crate) use convert::{from_arrays, to_arrays};

/// A solidcore body. Its own lock: solidcore's operations take `&mut Brep`, even to measure.
pub(crate) struct Body(pub(crate) Mutex<solidcore::topology::Brep>);

impl Body {
    pub(crate) fn new(b: solidcore::topology::Brep) -> Body {
        Body(Mutex::new(b))
    }
}

/// The fixes this backend runs with. None yet: each one is turned on with the fork commit it
/// matches and the rig case that needs it.
pub(crate) fn fixes() -> UpstreamFixes {
    UpstreamFixes::default()
}

fn fresh() -> Kernel {
    let mut k = Kernel::new(&KernelOptions::default());
    k.fixes = fixes();
    k
}

static KERNEL: Mutex<Option<Kernel>> = Mutex::new(None);

/// Runs `f` on the process's kernel. A panic in solidcore is caught: the kernel, which it may have
/// left half-updated, is replaced by a fresh one and the call fails with [`Error::Internal`].
pub(crate) fn run<T>(call: &'static str, f: impl FnOnce(&mut Kernel) -> std::result::Result<T, solidcore::SolidError>) -> Result<T> {
    let mut guard = KERNEL.lock().unwrap_or_else(PoisonError::into_inner);
    let k = guard.get_or_insert_with(fresh);
    match catch_unwind(AssertUnwindSafe(|| f(k))) {
        Ok(Ok(v)) => Ok(v),
        Ok(Err(e)) => Err(Error::Solidcore(format!("{call}: {e}"))),
        Err(p) => {
            *guard = Some(fresh());
            let why = p.downcast_ref::<&str>().map(|s| (*s).to_owned()).or_else(|| p.downcast_ref::<String>().cloned()).unwrap_or_default();
            log::warn!(target: "smlib", "solidcore panicked in {call}: {why}");
            Err(Error::Internal(format!("{call}: {why}")))
        }
    }
}

/// Runs `f` on the kernel and `body`, locking the kernel first.
pub(crate) fn with_body<T>(
    call: &'static str,
    body: &Body,
    f: impl FnOnce(&mut Kernel, &mut solidcore::topology::Brep) -> std::result::Result<T, solidcore::SolidError>,
) -> Result<T> {
    // A body a panic interrupted may be half-edited: it is never used again.
    if body.0.is_poisoned() {
        return Err(Error::Internal(format!("{call}: the body was left unfinished by an earlier panic")));
    }
    run(call, |k| {
        let mut b = body.0.lock().unwrap_or_else(PoisonError::into_inner);
        f(k, &mut b)
    })
}

/// An axis-aligned box from its lower corner (`SmApiCreateBox`).
pub(crate) fn make_box(origin: [f64; 3], size: [f64; 3]) -> Result<Body> {
    if !origin.iter().chain(&size).all(|x| x.is_finite()) {
        return Err(Error::Argument);
    }
    let [x, y, z] = size;
    run("create_box", |k| primitives::create_box(k, Vec3::new(origin[0], origin[1], origin[2]), x, y, z)).map(Body::new)
}

fn point(p: [f64; 3]) -> Result<Vec3> {
    if !p.iter().all(|x| x.is_finite()) {
        return Err(Error::Argument);
    }
    Ok(Vec3::new(p[0], p[1], p[2]))
}

/// A cylinder about +Z from its base centre (`SmApiCreateCylinder`, as `smlib_make_cylinder`
/// checks it: radius and height positive).
pub(crate) fn make_cylinder(origin: [f64; 3], radius: f64, height: f64) -> Result<Body> {
    let o = point(origin)?;
    if !(radius > 0.0 && height > 0.0 && radius.is_finite() && height.is_finite()) {
        return Err(Error::Argument);
    }
    run("create_cylinder", |k| primitives::create_cylinder(k, o, radius, height)).map(Body::new)
}

/// A sphere (`SmApiCreateSphere`; radius positive).
pub(crate) fn make_sphere(center: [f64; 3], radius: f64) -> Result<Body> {
    let o = point(center)?;
    if !(radius > 0.0 && radius.is_finite()) {
        return Err(Error::Argument);
    }
    run("create_sphere", |k| primitives::create_sphere(k, o, radius)).map(Body::new)
}

/// A cone or frustum about +Z (`SmApiCreateCone`; `smlib_make_cone`'s checks: base radius and
/// height positive, top radius at least 0 and not the base radius).
pub(crate) fn make_cone(origin: [f64; 3], base_radius: f64, top_radius: f64, height: f64) -> Result<Body> {
    let o = point(origin)?;
    let ok = base_radius > 0.0 && top_radius >= 0.0 && height > 0.0 && top_radius != base_radius;
    if !(ok && base_radius.is_finite() && top_radius.is_finite() && height.is_finite()) {
        return Err(Error::Argument);
    }
    run("create_cone", |k| primitives::create_cone(k, o, base_radius, top_radius, height)).map(Body::new)
}

/// A ring torus about +Z (`SmApiCreateTorus`; major radius > minor radius > 0).
pub(crate) fn make_torus(center: [f64; 3], major_radius: f64, minor_radius: f64) -> Result<Body> {
    let o = point(center)?;
    if !(minor_radius > 0.0 && minor_radius < major_radius && major_radius.is_finite()) {
        return Err(Error::Argument);
    }
    run("create_torus", |k| primitives::create_torus(k, o, major_radius, minor_radius)).map(Body::new)
}

/// The enclosed volume (`SmApiBrepComputeVolume`).
pub(crate) fn volume(body: &Body, relative_accuracy: f64) -> Result<f64> {
    if !relative_accuracy.is_finite() {
        return Err(Error::Argument);
    }
    with_body("brep_volume", body, |k, b| {
        let store = AttributeStore::default();
        let attrs = PropertyAttributes { store: &store, key_of: &|_| ObjectKey(0) };
        brep_volume(k, b, relative_accuracy, &attrs)
    })
}

/// Face, edge and vertex counts.
pub(crate) fn counts(body: &Body) -> Result<[usize; 3]> {
    with_body("counts", body, |_, b| Ok([b.face_list().len(), b.edge_list().len(), b.vertex_list().len()]))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_box_measures_exactly() {
        let b = make_box([1.0, -2.0, 3.0], [10.0, 20.0, 30.0]).unwrap();
        assert_eq!(counts(&b).unwrap(), [6, 12, 8]);
        let v = volume(&b, 1e-8).unwrap();
        assert!((v - 6000.0).abs() < 6000.0 * 1e-9, "{v}");
    }

    #[test]
    fn hostile_numbers_are_refused() {
        assert!(make_box([f64::NAN, 0.0, 0.0], [1.0; 3]).is_err());
        assert!(make_box([0.0; 3], [1.0, -1.0, 1.0]).is_err());
        let b = make_box([0.0; 3], [1.0; 3]).unwrap();
        assert!(volume(&b, f64::INFINITY).is_err());
    }

    #[test]
    fn a_panic_is_an_error_and_the_kernel_is_fresh_after() {
        let r: Result<()> = run("test", |_| panic!("boom"));
        assert!(matches!(r, Err(Error::Internal(ref m)) if m.contains("boom")), "{r:?}");
        assert!(make_box([0.0; 3], [1.0; 3]).is_ok());
    }
}
