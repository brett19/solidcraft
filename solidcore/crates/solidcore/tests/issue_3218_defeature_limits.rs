//! #3218: a defeature that keeps a feature on its base names why its patch is not built
//! ([`DefeatureFailure`], carried by the refusal's [`Reason::Defeature`] at the open loop's
//! edge, and by each [`KeptFeature::limit`]). Each limit is decided where the patch's
//! construction refuses, from what it found there:
//!
//! - `LoopNotPlanar`: a planar cap asked for a cylinder fillet's loop (two lines and two arcs
//!   on the cylinder), which no plane holds;
//! - `ExtensionRefused`: the faces along the loop asked to be extended by 0;
//! - `TangentNeighbours`: `fillet1019`'s fillet that ends on another, tangent, fillet: the
//!   patch's piece on the wall is cut from that fillet's extension only along an edge the
//!   kernel proves smooth, so nothing bounds it there;
//! - `NotEnclosed`: a box face, whose neighbours, extended, run past it;
//! - `ExtensionWithinTolerance`: the faces along a fillet's loop extended by less than the
//!   distance within which the loop's vertices and the patch's are one point.
//!
//! The models are the upstream test files, read by the Rust reader when the pinned upstream
//! checkout is present.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing
)] // test helpers

use solidcore::blend::feature_executive::{FeatureExecutive, KeptFeature, RebuildBrepType};
use solidcore::error::defeature::DefeatureFailure;
use solidcore::error::{Reason, SM_ERR};
use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::object::ObjectRef;
use solidcore::topology::brep::p1 as bp1;
use solidcore::topology::brep::p10::TopologyEnv;
use solidcore::topology::brep::p7::read_from_file;
use solidcore::topology::database_io::FileType;
use solidcore::topology::topology::NotifyRecords;
use solidcore::{Brep, EntityRef, Kernel, SolidError};

/// An upstream test file, read and arena-named as the goldens reader does.
fn read(k: &mut Kernel, file: &str) -> Option<Brep> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../upstream/TestFiles/pt_TestFiles")
        .join(file);
    let Ok(bytes) = std::fs::read(&path) else {
        assert!(
            std::env::var_os("SOLIDCORE_REQUIRE_UPSTREAM").is_none(),
            "SOLIDCORE_REQUIRE_UPSTREAM is set and the upstream test file is missing: {}",
            path.display()
        );
        return None;
    };
    let mut b = Brep::new_with_infinite_region().unwrap();
    let mut records = NotifyRecords::new();
    let mut attributes = AttributeStore::default();
    let key = |_: EntityRef| ObjectRef {
        key: ObjectKey(0),
        type_code: 0,
    };
    let mut env = TopologyEnv {
        all_attributes: &[],
        attributes: &mut attributes,
        key_of: &key,
        records: &mut records,
        next_key: 1 << 50,
    };
    read_from_file(k, &mut b, bytes, FileType::Ascii, false, &mut env).unwrap();
    b.use_arena_naming().unwrap();
    Some(b)
}

/// Face `face` of `file` defeatured alone with base rebuild `base_rb` (and the base extension
/// distance `ext`, when given): the refusal's limit, checked to be at an edge with the
/// status `SM_ERR`, and the base unchanged.
fn limit(
    file: &str,
    face: usize,
    base_rb: RebuildBrepType,
    ext: Option<f64>,
) -> Option<DefeatureFailure> {
    let mut k = Kernel::default();
    let mut b = read(&mut k, file)?;
    let faces = bp1::faces(&b, None).unwrap();
    let before = faces.len();
    let r = {
        let mut ex = FeatureExecutive::new(&mut k, &mut b).unwrap();
        ex.add_feature(
            &mut k,
            &[faces[face]],
            base_rb,
            RebuildBrepType::ExtendSurface,
        )
        .unwrap()
        .unwrap();
        if let Some(d) = ext {
            ex.set_base_ext_dist(d);
        }
        ex.do_defeature(&mut k)
    };
    assert_eq!(
        bp1::faces(&b, None).unwrap().len(),
        before,
        "{file} {face}: base changed"
    );
    match &r {
        Err(e @ SolidError::Failed(f)) => {
            assert!(matches!(f.entity(), Some(EntityRef::Edge(_))), "{r:?}");
            assert_eq!(e.sm_status(), Some(SM_ERR), "{r:?}");
            match f.reason {
                Reason::Defeature(d) => Some(d),
                _ => panic!("{file} {face}: {r:?}"),
            }
        }
        other => panic!("{file} {face}: a refusal naming its limit, got {other:?}"),
    }
}

/// What a defeature of `box_with_large_fillet`'s fillet (face 6) past the identity radius
/// builds (#16457), checked against the input's geometry: the input's vertices span the
/// box [0, 10]^3 and its faces are six planes and the fillet, so taking the fillet off
/// restores that box. The body's volume is the box's, 1000 (to 1e-9 relative); every face
/// lies on one of the box's six planes (all its vertices share that coordinate) and their
/// areas sum to the box's 600. The box's six planes are left in 10 faces (MEASURED: the
/// patch's planar pieces are not merged with the faces they extend). Level 2 reports the
/// defect as it is, 6 edges failing `ValidateGeometry`, so a fix shows.
fn assert_box_restored_with_its_defect(k: &mut Kernel, b: &mut Brep) {
    use solidcore::mass::integrator::properties::{brep_volume, face_area, PropertyAttributes};
    let store = AttributeStore::default();
    let attrs = PropertyAttributes {
        store: &store,
        key_of: &|_| ObjectKey(0),
    };
    let volume = brep_volume(k, b, 1e-9, &attrs).unwrap();
    assert!((volume - 1000.0).abs() <= 1e-9 * 1000.0, "volume {volume}");
    let faces = bp1::faces(b, None).unwrap();
    assert_eq!(faces.len(), 10, "the box's six planes in 10 faces");
    let mut area = 0.0;
    for &f in &faces {
        let points: Vec<[f64; 3]> =
            solidcore::topology::face::p4::vertices(b, f, None, false, None)
                .unwrap()
                .into_iter()
                .map(|v| {
                    let p = b.vertex(v).unwrap().point;
                    [p.x, p.y, p.z]
                })
                .collect();
        let on_a_box_plane = (0..3).any(|axis| {
            [0.0, 10.0]
                .iter()
                .any(|&c| points.iter().all(|p| (p[axis] - c).abs() < 1e-6))
        });
        assert!(
            on_a_box_plane,
            "face {f:?} off the box's planes: {points:?}"
        );
        area += face_area(k, b, f, 1e-9).unwrap();
    }
    assert!((area - 600.0).abs() <= 1e-6, "area {area}");
    let check = solidcore::topology::brep::validation::check_brep(
        k,
        b,
        solidcore::topology::brep::validation::CheckLevel::Level2,
    )
    .unwrap();
    let mut want =
        vec![
            "SmBrep::Test #61 Valid: Bad Edge fails pE->ValidateGeometry(FALSE) check [owner SmEdge]"
                .to_string();
            6
        ];
    want.push("AssertValid skipped: ValidatePointers failed".to_string());
    assert_eq!(check.lines(), want, "#16457: the body's Level 2 report");
}

#[test]
fn a_planar_cap_on_a_cylinder_fillet_s_loop_is_refused_as_not_planar() {
    let Some(l) = limit(
        "box_with_large_fillet.smb",
        6,
        RebuildBrepType::PlanarCap,
        None,
    ) else {
        return;
    };
    assert_eq!(l, DefeatureFailure::LoopNotPlanar);
}

#[test]
fn an_extension_by_nothing_is_refused_as_the_extension_s() {
    let Some(l) = limit(
        "box_with_large_fillet.smb",
        6,
        RebuildBrepType::ExtendSurface,
        Some(0.0),
    ) else {
        return;
    };
    assert_eq!(l, DefeatureFailure::ExtensionRefused { distance: 0.0 });
}

#[test]
fn a_fillet_ending_on_a_tangent_fillet_is_refused_as_tangent_neighbours() {
    let Some(l) = limit(
        "Fillet/fillet1019.smb",
        3,
        RebuildBrepType::ExtendSurface,
        None,
    ) else {
        return;
    };
    let within = Kernel::default().tolerances.ang_tol_deg;
    assert_eq!(
        l,
        DefeatureFailure::TangentNeighbours { within_deg: within }
    );
}

#[test]
fn a_box_face_is_refused_as_not_enclosed() {
    let Some(l) = limit(
        "box_with_large_fillet.smb",
        0,
        RebuildBrepType::ExtendSurface,
        None,
    ) else {
        return;
    };
    assert!(matches!(l, DefeatureFailure::NotEnclosed { .. }), "{l:?}");
}

/// `box_with_large_fillet`'s fillet (face 6), its faces extended by 0.001: the loop's
/// vertices carry tolerances of 2.374e-3, so a vertex of the extended boundary and the
/// loop's vertex are one point within their summed zone tolerances and the 0.001 extension
/// is no feature the kernel can resolve (the merged vertex's tolerance covers it). The
/// refusal names that identity radius, from the loop vertices' tolerances and the patch's:
/// a lower bound on the extension, not a size that builds a valid body (see the bracket test
/// below and #16457).
#[test]
fn an_extension_within_the_loop_s_tolerance_is_refused_naming_the_identity_radius() {
    let Some(l) = limit(
        "box_with_large_fillet.smb",
        6,
        RebuildBrepType::ExtendSurface,
        Some(0.001),
    ) else {
        return;
    };
    let DefeatureFailure::ExtensionWithinTolerance {
        extended_by,
        identity_radius,
    } = l
    else {
        panic!("a refusal naming the identity radius, got {l:?}");
    };
    assert_eq!(extended_by, 0.001);
    let mut k = Kernel::default();
    let mut b = read(&mut k, "box_with_large_fillet.smb").unwrap();
    let widest = bp1::vertices(&b, None)
        .unwrap()
        .into_iter()
        .map(|v| b.vertex(v).unwrap().tolerance)
        .fold(0.0_f64, f64::max);
    let patch = b.tolerance();
    assert!(
        identity_radius > widest && identity_radius <= (widest + patch) * (1.0 + 1e-12),
        "radius {identity_radius}, loop vertex tolerance {widest}, patch {patch}"
    );
    let faces = bp1::faces(&b, None).unwrap();
    let mut ex = FeatureExecutive::new(&mut k, &mut b).unwrap();
    ex.add_feature(
        &mut k,
        &[faces[6]],
        RebuildBrepType::ExtendSurface,
        RebuildBrepType::ExtendSurface,
    )
    .unwrap()
    .unwrap();
    ex.set_base_ext_dist(1.01 * identity_radius);
    let r = ex.do_defeature(&mut k);
    assert!(r.is_ok(), "extended by 1.01 x {identity_radius}: {r:?}");
    drop(ex);
    assert_box_restored_with_its_defect(&mut k, &mut b);
}

/// The same fillet bracketed at its identity radius: extended by 0.9999 x the radius the
/// patch still fails and is refused naming the radius; extended by 1.0001 x it builds. On
/// this model the construction's own failure ends at the radius to within 1e-4 (measured:
/// it fails at 2.3843e-3 and builds at 2.3845e-3, the radius being 2.38440e-3), so the
/// refusal is named over no extension that would have built. The body built past it is not
/// valid: Level 2 reports 6 x `SmBrep` test 61 (an edge fails `ValidateGeometry`), as main's
/// body does at every extension from 0.001 to 0.01 (#16457). The radius is a lower bound on
/// the extension, never a size that fits; this asserts the defect as it is, so a fix shows.
#[test]
fn the_identity_radius_brackets_where_the_patch_starts_to_build() {
    // (no upstream test file: nothing to check)
    let Some(l) = limit(
        "box_with_large_fillet.smb",
        6,
        RebuildBrepType::ExtendSurface,
        Some(0.001),
    ) else {
        return;
    };
    let DefeatureFailure::ExtensionWithinTolerance {
        identity_radius, ..
    } = l
    else {
        panic!("a refusal naming the identity radius, got {l:?}");
    };
    let below = 0.9999 * identity_radius;
    let l = limit(
        "box_with_large_fillet.smb",
        6,
        RebuildBrepType::ExtendSurface,
        Some(below),
    );
    assert_eq!(
        l,
        Some(DefeatureFailure::ExtensionWithinTolerance {
            extended_by: below,
            identity_radius,
        })
    );
    let mut k = Kernel::default();
    let mut b = read(&mut k, "box_with_large_fillet.smb").unwrap();
    let faces = bp1::faces(&b, None).unwrap();
    let mut ex = FeatureExecutive::new(&mut k, &mut b).unwrap();
    ex.add_feature(
        &mut k,
        &[faces[6]],
        RebuildBrepType::ExtendSurface,
        RebuildBrepType::ExtendSurface,
    )
    .unwrap()
    .unwrap();
    ex.set_base_ext_dist(1.0001 * identity_radius);
    let r = ex.do_defeature(&mut k);
    assert!(r.is_ok(), "extended by 1.0001 x {identity_radius}: {r:?}");
    drop(ex);
    assert_box_restored_with_its_defect(&mut k, &mut b);
}

/// A kept feature carries its limit too: of the two fillets of `cyl_on_box_w_fillet` one
/// comes off and the other is kept, naming why.
#[test]
fn a_kept_feature_names_its_limit() {
    let mut k = Kernel::default();
    let Some(mut b) = read(&mut k, "Fillet/cyl_on_box_w_fillet.smb") else {
        return;
    };
    let faces = bp1::faces(&b, None).unwrap();
    let kept: Vec<KeptFeature> = {
        let mut ex = FeatureExecutive::new(&mut k, &mut b).unwrap();
        for f in [3, 8] {
            ex.add_feature(
                &mut k,
                &[faces[f]],
                RebuildBrepType::ExtendSurface,
                RebuildBrepType::ExtendSurface,
            )
            .unwrap()
            .unwrap();
        }
        ex.do_defeature(&mut k).unwrap()
    };
    assert_eq!(kept.len(), 1, "{kept:?}");
    assert!(kept[0].limit.is_some(), "{kept:?}");
}
