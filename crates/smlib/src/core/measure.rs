//! The Measure and Check families on solidcore: what `capi/src/smlib_measure.cpp`,
//! `smlib_brep_solid_count` (`smlib_ops.cpp`), `smlib_brep_is_manifold_solid`
//! (`smlib_flat_api.cpp`) and `smlib_brep_check` (`smlib_check.cpp`) do on the C++ kernel, here
//! composed from solidcore's ports of the same SMLib routines.
//!
//! Two of them have no solidcore port yet and fail with "not supported yet":
//! `SmApiBrepMaterialCensus` ([`solid_count`]) and the topology-level `SmBrep::AssertValid`
//! ([`check`]).

use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::tol::{SmallTopoType, context_xsect_tol_3d};
use solidcore::heal::gap::GapFunction;
use solidcore::mass::integrator::properties::{MassPropertyBehavior, PropertyAttributes, brep_properties, face_properties};
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::query::api_queries::{brep_bounding_box, brep_is_manifold_solid, face_bounding_box};
use solidcore::topology::brep::p1 as bl;
use solidcore::topology::face::p2::{PPF_AREA, PPF_AREA_FIRST_MOMENTS};
use solidcore::{Kernel, Point3, SolidError, Vec3};

use super::{Body, with_body};
use crate::check::{CheckLevel, CheckReport};
use crate::measure::MassProperties;
use crate::{Error, Result};

const OP: &str = "smlib_measure";

/// The accuracy the C API passes on: finite, clamped to [1e-8, 1e-1].
fn accuracy(relative_accuracy: f64) -> Result<f64> {
    if !relative_accuracy.is_finite() {
        return Err(Error::Argument);
    }
    Ok(relative_accuracy.clamp(1e-8, 1e-1))
}

fn invalid(why: &'static str) -> SolidError {
    SolidError::invalid_input(OP, why)
}

fn finite(v: &[f64]) -> bool {
    v.iter().all(|x| x.is_finite())
}

/// `faces_area` of `smlib_measure.cpp`: every face counted once (SmApiBrepComputeArea, with the
/// [1e-8, 1e-1] envelope instead of its [1e-4, 1e-1]), the total area and, when `first` is set,
/// the area's first moments about `o`.
fn faces_area(k: &Kernel, b: &mut solidcore::Brep, accuracy: f64, o: Point3, first: bool) -> solidcore::Result<(f64, Vec3)> {
    let mut area = 0.0;
    let mut sum = Vec3::new(0.0, 0.0, 0.0);
    let flags = if first { PPF_AREA | PPF_AREA_FIRST_MOMENTS } else { PPF_AREA };
    for f in bl::faces(b, None)? {
        let size = face_bounding_box(k, b, f, false)?.size();
        let edge = size.z.max(size.x.max(size.y));
        let p = face_properties(k, b, f, OrientType::Same, accuracy, o, edge * edge / 100.0, 0.0, flags)?;
        // The area and its moments share one integrand, so a trim boundary integrated the other
        // way round flips both.
        let sign = if p.area < 0.0 { -1.0 } else { 1.0 };
        area += sign * p.area;
        if first {
            sum += p.moments[0] * sign;
        }
    }
    if !area.is_finite() {
        return Err(invalid("non-finite area"));
    }
    Ok((area, sum))
}

/// `smlib_brep_mass_properties`: `SmBrep::ComputePreciseProperties` directly, as the C API calls
/// it (not `SmApiBrepComputeMassProperties`: the [1e-8, 1e-1] accuracy envelope, no manifold
/// refusal, the sum over material regions), the face-by-face area and its centroid for a body
/// with no material, and moments about the middle of the box moved to the centroid.
pub(crate) fn mass_properties(body: &Body, relative_accuracy: f64) -> Result<MassProperties> {
    let accuracy = accuracy(relative_accuracy)?;
    with_body("mass_properties", body, |k, b| {
        let mut material = false;
        for r in bl::regions(b, None)? {
            // IsVoid's UNSURE (2) is true in C++: only FALSE is material.
            material |= b.region(r)?.is_void() == 0;
        }
        // The C API refuses a body with per-entity mass attributes (they would override the unit
        // density). Bodies here carry none: the attribute store passed in is empty.
        let store = AttributeStore::default();
        let attrs = PropertyAttributes { store: &store, key_of: &|_| ObjectKey(0) };
        // Moments about the middle of the box, moved to the centroid below: about the world
        // origin, a body far from it would lose its inertia to cancellation.
        let o = brep_bounding_box(k, b, false)?.mid();
        if !finite(&[o.x, o.y, o.z]) {
            return Err(invalid("non-finite box"));
        }
        if !material {
            let (area, first) = faces_area(k, b, accuracy, o, true)?;
            if !(area > 0.0) {
                return Err(invalid("no area"));
            }
            let c = [o.x + first.x / area, o.y + first.y / area, o.z + first.z / area];
            if !finite(&[area, c[0], c[1], c[2]]) {
                return Err(invalid("non-finite result"));
            }
            return Ok(MassProperties { area, volume: 0.0, centroid: c, moments: [0.0; 3], products: [0.0; 3] });
        }
        let p = brep_properties(k, b, accuracy, o, MassPropertyBehavior { density: 1.0, ..MassPropertyBehavior::default() }, &attrs)?;
        let (mass, volume) = (p.mass, p.volume);
        if !mass.is_finite() || !(mass > 0.0) || !(volume > 0.0) {
            return Err(invalid("no mass"));
        }
        // m[4]: first moments about o; m[7]: (y²+z², z²+x², x²+y²) about o; m[6]: (yz, zx, xy)
        // about o. Parallel axes from o to the centroid (unit density: mass = volume).
        let [_, _, _, _, m4, _, m6, m7] = p.moments;
        let d = [m4.x / mass, m4.y / mass, m4.z / mass];
        let r = MassProperties {
            area: p.area,
            volume,
            centroid: [o.x + d[0], o.y + d[1], o.z + d[2]],
            moments: [
                m7.x - mass * (d[1] * d[1] + d[2] * d[2]),
                m7.y - mass * (d[0] * d[0] + d[2] * d[2]),
                m7.z - mass * (d[0] * d[0] + d[1] * d[1]),
            ],
            products: [m6.x - mass * d[1] * d[2], m6.y - mass * d[2] * d[0], m6.z - mass * d[0] * d[1]],
        };
        if !finite(&[r.area, r.volume]) || !finite(&r.centroid) || !finite(&r.moments) || !finite(&r.products) {
            return Err(invalid("non-finite result"));
        }
        Ok(r)
    })
}

/// `smlib_brep_area`: every face counted once.
pub(crate) fn area(body: &Body, relative_accuracy: f64) -> Result<f64> {
    let accuracy = accuracy(relative_accuracy)?;
    with_body("area", body, |k, b| faces_area(k, b, accuracy, Point3::new(0.0, 0.0, 0.0), false).map(|(a, _)| a))
}

/// `smlib_brep_bounds`: `SmBrep::CalculateTightBoundingBox`.
pub(crate) fn bounds(body: &Body) -> Result<([f64; 3], [f64; 3])> {
    with_body("bounds", body, |k, b| {
        let x = brep_bounding_box(k, b, true)?;
        let (lo, hi) = (x.min(), x.max());
        let (lo, hi) = ([lo.x, lo.y, lo.z], [hi.x, hi.y, hi.z]);
        if !finite(&lo) || !finite(&hi) || lo.iter().zip(&hi).any(|(a, b)| a > b) {
            return Err(invalid("no box"));
        }
        Ok((lo, hi))
    })
}

/// `smlib_brep_edge_face_distance`: SMLib's own edge/face gap (`SmEdgeuse::CalcMaxEdgeFaceGap`'s
/// `SmCrvSrfGapFunction` with 20 samples, its largest sample). Indices in the body's edge and
/// face order.
pub(crate) fn edge_face_distance(body: &Body, edge: usize, face: usize) -> Result<f64> {
    let found = with_body("edge_face_distance", body, |k, b| {
        let (Some(&e), Some(&f)) = (bl::edges(b, None)?.get(edge), bl::faces(b, None)?.get(face)) else {
            return Ok(None);
        };
        let interval = b.edge(e)?.interval;
        let (Some(curve), Some(surface)) = (b.edge(e)?.curve.as_ref(), b.face(f)?.surface.as_ref()) else {
            return Err(invalid("edge without curve or face without surface"));
        };
        if !(interval.max() > interval.min()) {
            return Err(invalid("empty edge interval"));
        }
        // SmTol::GetXSectTol3d(brep's context): the process context. The tolerance only
        // classifies the samples; it does not change the distances.
        let tol = context_xsect_tol_3d(k, None, SmallTopoType::NormalSize);
        let mut gaps = GapFunction::crv_srf(tol, curve, &interval, surface, 20);
        let d = gaps.max_gap_sample(k, false)?.length();
        if !d.is_finite() {
            return Err(invalid("non-finite gap"));
        }
        Ok(Some(d))
    })?;
    found.ok_or(Error::Argument)
}

/// `smlib_brep_is_manifold_solid`: `SmApiBrepIsManifoldSolid`.
pub(crate) fn is_manifold_solid(body: &Body) -> Result<bool> {
    with_body("is_manifold_solid", body, |_, b| brep_is_manifold_solid(b))
}

/// `smlib_brep_solid_count` calls `SmApiBrepMaterialCensus`, which solidcore does not port yet.
pub(crate) fn solid_count(body: &Body) -> Result<usize> {
    let _ = body;
    Err(Error::Solidcore("SmApiBrepMaterialCensus: not supported yet: solidcore has no port of it".into()))
}

/// `smlib_brep_check` runs `SmBrep::ValidatePointers`, then `SmBrep::AssertValid` over the whole
/// topology graph. solidcore ports the first but not the second (nor the brep, face, loop, edge
/// and vertex `AssertValid` it walks), so the check cannot run.
pub(crate) fn check(body: &Body, level: CheckLevel) -> Result<CheckReport> {
    let _ = (body, level);
    Err(Error::Solidcore("SmBrep::AssertValid: not supported yet: solidcore has no topology-level AssertValid".into()))
}
