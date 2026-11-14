//! Regression test (tangency-labs/solidcraft#15282 review): an offset whose mated edge is
//! off one of its extended surfaces is a closed solid once `MateEdges` installs the
//! intersection in its place.
//!
//! `Fillet/tilt_cyl_on_box.smb` (a box with a tilted cylinder, filleted) offset on its
//! face 1 by 11.0492, in and out, convex edges extended, as the goldens case
//! `pe_Fillet_tilt_cyl_on_box_f1_d0_{in,out}_ext` does. Upstream's shortcut kept the old
//! offset edge because its ends match the intersection's, though it lies at least 0.3 off
//! one of the two surfaces in between (certified:
//! `offset::curve_on_surfaces::tests::tilt_cyl_old_offset_edge_is_off_its_plane_and_the_intersection_is_on_both`);
//! its result (1 region, 1 shell, 8 faces, 9 loops, 16 edges, 10 vertices) is no solid:
//! `V - E + F - (L - F) = 1`, which no closed surface gives. `MateEdges` now keeps an
//! offset edge only where it is proved on both surfaces, and installs the intersection
//! (proved on both) otherwise.
//!
//! The result is one closed manifold solid: two regions (the infinite one and the solid's)
//! bounded by the two faceuse shells of one surface (8 faces, 9 loops, 15 edges, 10
//! vertices: Euler characteristic 2, genus 0), Level 2 check-clean.
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing
)] // test helpers

#[path = "support/offset_fixture.rs"]
mod offset_fixture;

use offset_fixture::Notifications;
use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::object::ObjectRef;
use solidcore::offset::offset_executive::p1::{OffsetExecutive, OffsetOperation};
use solidcore::offset::offset_executive::p2::SolidOffsetOptions;
use solidcore::query::api_queries::brep_is_manifold_solid;
use solidcore::topology::brep::p1 as brep_p1;
use solidcore::topology::brep::p10::TopologyEnv;
use solidcore::topology::brep::p7::read_from_file;
use solidcore::topology::brep::validation::{check_brep, CheckLevel};
use solidcore::topology::database_io::FileType;
use solidcore::topology::topology::NotifyRecords;
use solidcore::{Brep, EntityRef, Kernel};

fn tilt_cyl_on_box(k: &mut Kernel) -> Option<Brep> {
    let path = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("../../upstream/TestFiles/pt_TestFiles/Fillet/tilt_cyl_on_box.smb");
    let Ok(bytes) = std::fs::read(&path) else {
        assert!(
            std::env::var_os("SOLIDCORE_REQUIRE_UPSTREAM").is_none(),
            "SOLIDCORE_REQUIRE_UPSTREAM is set and the upstream test file is missing: {}",
            path.display()
        );
        eprintln!("unavailable upstream test file: {}", path.display());
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
        next_key: 1 << 40,
    };
    read_from_file(k, &mut b, bytes, FileType::Ascii, false, &mut env).unwrap();
    b.use_arena_naming().unwrap();
    Some(b)
}

/// `DoSolidOffset` of the part on its face 1 by 11.0492 (`inset` in or out), convex edges
/// extended, results not merged, as the goldens case; `None` without the pinned upstream
/// checkout.
fn offset(inset: bool) -> Option<(Kernel, Brep)> {
    let mut k = Kernel::default();
    let mut part = tilt_cyl_on_box(&mut k)?;
    let face = part.face_list()[1];
    let mut exec =
        OffsetExecutive::new(OffsetOperation::SolidOffset, 11.0492, 0.001_339_890_721_639);
    exec.set_inset(inset);
    exec.set_extend_convex_edges(true);
    exec.set_merge_results(false);
    exec.set_shell_faces(&[face]);
    let mut notifications = Notifications::new(&mut part);
    let result = exec
        .do_solid_offset(
            &mut k,
            Some(&mut part),
            SolidOffsetOptions::default(),
            &mut notifications.context(),
        )
        .expect("offset");
    Some((k, result))
}

/// One closed manifold solid: two regions (the infinite one and the solid's), two
/// faceuse shells (the two sides of its one surface), that surface of Euler characteristic
/// `V - E + F - (L - F) = 2` (genus 0), Level 2 clean.
fn assert_one_genus_0_solid(inset: bool) {
    let Some((mut k, mut b)) = offset(inset) else {
        return;
    };
    b.use_arena_naming().unwrap();
    assert!(brep_is_manifold_solid(&b).unwrap(), "inset {inset}");
    let regions = brep_p1::regions(&b, None).unwrap();
    let mut shells = std::collections::BTreeSet::new();
    for &r in &regions {
        shells.extend(b.region(r).unwrap().shells.iter().copied());
    }
    let faces = brep_p1::faces(&b, None).unwrap();
    let edges = brep_p1::edges(&b, None).unwrap();
    let vertices = brep_p1::vertices(&b, None).unwrap();
    let mut loops = 0usize;
    for &f in &faces {
        loops += solidcore::topology::face::p2::loops(&b, f, false, None)
            .unwrap()
            .len();
    }
    let (v, e, f, l) = (
        vertices.len() as i64,
        edges.len() as i64,
        faces.len() as i64,
        loops as i64,
    );
    assert_eq!(regions.len(), 2, "inset {inset}");
    assert_eq!(shells.len(), 2, "inset {inset}");
    assert_eq!(
        v - e + f - (l - f),
        2,
        "inset {inset}: V {v} E {e} F {f} L {l}"
    );
    let lines = check_brep(&mut k, &mut b, CheckLevel::Level2)
        .unwrap()
        .lines();
    assert!(lines.is_empty(), "inset {inset}: {lines:?}");
}

#[test]
fn tilt_cyl_on_box_offset_in_is_one_checked_solid() {
    assert_one_genus_0_solid(true);
}

#[test]
fn tilt_cyl_on_box_offset_out_is_one_checked_solid() {
    assert_one_genus_0_solid(false);
}
