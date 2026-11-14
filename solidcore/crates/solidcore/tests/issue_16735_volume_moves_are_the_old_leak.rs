//! #16735: every volume `brep_volume`'s closure of tolerant edges moved is the old
//! integrator's leak through the edges' gaps, not a change of shape (see
//! `support/volume_leak.rs` for the three checks).
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing
)] // test code

#[path = "support/volume_leak.rs"]
mod volume_leak;

use solidcore::math::Point3;
use solidcore::primitive::api_primitives::create_box;
use solidcore::Kernel;

/// The checks on a box, whose edges are exact: no gap, no leak, the closed form.
#[test]
fn a_box_has_no_leak() {
    let mut k = Kernel::default();
    let mut b = create_box(&mut k, Point3::new(1.0, -2.0, 3.0), 4.0, 5.0, 6.0).unwrap();
    let l = volume_leak::leak(&k, &mut b, 1e-10, 64);
    assert!((l.volume - 120.0).abs() <= 1e-9 * 120.0, "{l:?}");
    assert!(l.origin_spread <= 1e-8 * 120.0, "{l:?}");
    assert!(
        l.old_bound <= 1e-9 && l.worst_gap_over_tolerance <= 1.0,
        "{l:?}"
    );
}
