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

pub(crate) mod blend;
#[cfg(test)]
mod blend_tests;
pub(crate) mod boolean;
#[cfg(test)]
mod boolean_tests;
mod convert;
#[cfg(test)]
mod convert_tests;
mod measure;
#[cfg(test)]
mod measure_tests;
pub(crate) mod offset;
#[cfg(test)]
mod offset_tests;
pub(crate) mod sweep;
#[cfg(test)]
mod sweep_tests;

pub(crate) use convert::{from_arrays, to_arrays};
pub(crate) use measure::{area, bounds, check, edge_face_distance, is_manifold_solid, mass_properties, solid_count};

/// A solidcore body. Its own lock: solidcore's operations take `&mut Brep`, even to measure.
pub(crate) struct Body(pub(crate) Mutex<solidcore::topology::Brep>);

impl Body {
    pub(crate) fn new(b: solidcore::topology::Brep) -> Body {
        Body(Mutex::new(b))
    }
}

/// The fixes this backend runs with: every one of solidcore's opt-in fixes of upstream SMLib
/// defects (`UpstreamFixes`, each naming its record and issue). They are all fixes; the C++ fork
/// carries most of them in its own sources. A test checks none is left off, so moving the
/// solidcore pin to a rev with a new flag fails until it is added here.
pub(crate) fn fixes() -> UpstreamFixes {
    let mut f = UpstreamFixes::default();
    f.fillet_retry_radius_as_first = true;
    f.fillet_rail_rim_on_face = true;
    f.fillet_refuse_rails_leaving_faces = true;
    f.fillet_refuse_face_consuming_blends = true;
    f.fillet_refuse_non_manifold_result = true;
    f.fillet_refuse_crossing_rails = true;
    f.fillet_corner_ball_patch = true;
    f.fillet_chamfer_cone_on_arc = true;
    f.fillet_torus_center_on_axis = true;
    f.offset_excess_faces_after_merge = true;
    f.offset_clip_right_side_out = true;
    f.revolution_offset_axis_both_sides = true;
    f.offset_extended_edges_mated = true;
    f.remove_slit_loops_where_solids_touch = true;
    f.keep_operand_uv_trims = true;
    f.geometric_g2_continuity = true;
    f.proto_vertex_miss_is_false = true;
    f.check_loop_vertex_tolerance = true;
    f.check_loop_projected_seam_ends = true;
    f.trim_boundary_seam_wrap = true;
    f.bspline_check_min_radius = true;
    f.oscillating_skips_failed_guesses = true;
    f.refine_cc_range_ends = true;
    f.flat_corners_test_bits = true;
    f.cylinder_rebuild_mean_radius = true;
    f.planar_path_uncapped_open_profiles = true;
    f.revolve_analytic_profile_faces = true;
    f.revolve_arc_centre_near_axis_is_sphere = true;
    f.taper_pivot_nearest_corner = true;
    f.revolve_nested_shells_as_cavities = true;
    f.curve_sweep_caps_at_sweep_ends = true;
    f.join_touching_loops = true;
    f.uv_trim_reversal_on_surface = true;
    f.offset_resolve_crossing_loops = true;
    f.offset_walls_touch_refused = true;
    f.offset_analytic_mate_edges = true;
    f.precise_properties_split_at_surface_knots = true;
    f.nurb_cylinder_inside_out_frame = true;
    f.shell_inner_shell_of_statuses = true;
    f.local_operation_offset_status = true;
    f.torus_major_radius_from_origin = true;
    f.fillet_corner_sphere_from_four_arcs = true;
    f.fillet_nx1_opposite_continuation_side_edge = true;
    f.fillet_trace_rejects_short_step = true;
    f.torus_iso_circle_axis_from_midpoint = true;
    f.torus_keeps_inside_out = true;
    f.internal_pole_row_through_knot = true;
    f.transform_scales_each_tolerance = true;
    f.transform_model_size_from_body_box = true;
    f.offset_collapsed_rounds_mated = true;
    f.offset_missing_surface_fails = true;
    f.fillet_nx1_end_rib_global_merge = true;
    f.uv_trim_corner_end_points_only = true;
    f.bspline_trim_preserves_uv_spans = true;
    f.cone_parallel_line_midpoint = true;
    f.cone_parallel_line_domains = true;
    f.proto_edge_self_loop_on_vertex_degenerate = true;
    f.slice_coincident_faces_sheet_side = true;
    f.boolean_outer_shell_first = true;
    f.edge_reversal_keeps_failed_edge = true;
    f.topology_failed_evaluations_propagate = true;
    f.heal_stage3_failures_propagate = true;
    f.sweep_frame_on_planar_path_normal = true;
    f.cover_refuses_folded_continuation = true;
    f.boolean_vertex_by_best_crossing = true;
    f.boolean_cleanup_removes_slits = true;
    f.classify_edge_interval = true;
    f.ellipse_trim_seam_lines_span_cone_height = true;
    f.degenerate_proto_edge_in_one_vertex = true;
    f.new_edge_vertex_tolerance_covers_gap = true;
    f.torus_signed_gen_curve_interval = true;
    f.sheet_sweep_joins_profile_wires = true;
    f.point_solve_keeps_triangle_guess = true;
    f.fillet_setback_from_start_vertex = true;
    f.imprint_trims_edge_to_proto_vertex = true;
    f.imprint_extends_edge_to_proto_vertex = true;
    f.fillet_point_step_per_point = true;
    f.analytic_copy_keeps_parameterization = true;
    f.fillet_point_bigger_step_retry = true;
    f.fillet_through_extended_rail_end = true;
    f.precise_properties_body_length_scale = true;
    f.offset_extension_from_base = true;
    f.corner_blend_stays_inside_neighbours = true;
    f.fillet_cleanup_keeps_target_faces = true;
    f.fillet_nx1_tangent_side_edge = true;
    f.offset_closed_curve_trim_arc = true;
    f.offset_stitch_at_body_tolerance = true;
    f.curve_contacts_at_curve_ends = true;
    f.shell_empty_cavity_fails = true;
    f.fillet_refuse_ball_not_fitting = true;
    f.boolean_lists_virtual_surface_copies = true;
    f.sphere_small_circle_not_touching = true;
    f.ssi_tangent_contact_points = true;
    f.ssi_trace_tangent_curves_once = true;
    f.found_torus_sphere_keep_tested_nurb = true;
    f
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

/// A call the capi checks for `SM_SUCCESS`, treating any other status as no answer: an error
/// that stands for an SMLib status exactly ([`SolidError::sm_status`](solidcore::SolidError::sm_status),
/// solidcore#1452) is `None`. Every other solidcore error
/// (unported code, invalid input to an adapter, a missing record...) is not a status the C++
/// path returns, so it fails the call instead of hiding as a skipped entity.
pub(crate) fn answer<T>(r: std::result::Result<T, solidcore::SolidError>) -> std::result::Result<Option<T>, solidcore::SolidError> {
    match r {
        Ok(v) => Ok(Some(v)),
        Err(e) if e.sm_status().is_some() => Ok(None),
        Err(e) => Err(e),
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

/// An axis-aligned box from its lower corner (`SmApiCreateBox`, as `smlib_make_box` checks it:
/// every edge length positive).
pub(crate) fn make_box(origin: [f64; 3], size: [f64; 3]) -> Result<Body> {
    if !origin.iter().chain(&size).all(|x| x.is_finite()) || !size.iter().all(|&s| s > 0.0) {
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
    fn every_upstream_fix_is_on() {
        let f = format!("{:?}", fixes());
        assert!(!f.contains(": false"), "an UpstreamFixes flag is off: {f}");
    }

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
