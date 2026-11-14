//! #16735: checks that a volume moved by `brep_volume`'s closure of tolerant edges is the
//! old integrator's leak, not a change of shape.
//!
//! The old integrator took each face over its own trims. Those trims' images lie a gap g
//! (at most the edge tolerance) from the edge curve. The field `(0, 0, z - o_z)` then
//! leaked through the gaps by at most sum over edge uses of `g L max|z - o_z|`
//! ([`Leak::old_bound`], with g sampled: MEASURED). The new integrator closes every face
//! onto the edge curves, so its volume does not depend on the origin
//! ([`Leak::origin_spread`]). It differs from the solid's only within the tolerance tubes
//! and balls, sum pi tol^2 L + sum 4/3 pi tol^3 ([`Leak::new_bound`]), provided every gap
//! is within its stored tolerance ([`Leak::worst_gap_over_tolerance`] <= 1).
#![allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::panic,
    clippy::indexing_slicing,
    dead_code
)] // test helpers; each caller uses a subset

use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::mass::integrator::properties::{
    brep_properties, edge_length, MassPropertyBehavior, PropertyAttributes,
};
use solidcore::math::{Point2, Point3, PI};
use solidcore::nurbs::b_spline_surface_n_lib::p2::EvaluateFlags;
use solidcore::query::brep::{calculate_bounding_box, BoundingBoxOptions};
use solidcore::topology::api_files::read_brep_from_file;
use solidcore::topology::brep::p1;
use solidcore::topology::brep::p10::TopologyEnv;
use solidcore::topology::edgeuse::p1 as eu1;
use solidcore::topology::topology::NotifyRecords;
use solidcore::topology::{edge, loopuse, vertex};
use solidcore::{Brep, Kernel};

/// A B-rep read from SMB bytes (`ascii` for the text form).
pub fn read_smb(k: &mut Kernel, bytes: &[u8], ascii: bool) -> Brep {
    let mut attributes = AttributeStore::default();
    let mut records = NotifyRecords::new();
    let mut env = TopologyEnv {
        all_attributes: &[],
        attributes: &mut attributes,
        key_of: &loopuse::notify_ref,
        records: &mut records,
        next_key: 1 << 50,
    };
    let mut b = read_brep_from_file(k, Some(bytes.to_vec()), ascii, false, &mut env).unwrap();
    b.refresh_cached_topology();
    b
}

/// The B-rep's volume about `origin` to `accuracy`.
pub fn volume_about(k: &Kernel, b: &mut Brep, origin: Point3, accuracy: f64) -> f64 {
    properties_about(k, b, origin, accuracy).volume
}

/// The B-rep's properties about `origin` to `accuracy`.
pub fn properties_about(
    k: &Kernel,
    b: &mut Brep,
    origin: Point3,
    accuracy: f64,
) -> solidcore::mass::integrator::properties::PreciseProperties {
    let store = AttributeStore::default();
    let attrs = PropertyAttributes {
        store: &store,
        key_of: &|_| ObjectKey(0),
    };
    brep_properties(
        k,
        b,
        accuracy,
        origin,
        MassPropertyBehavior::default(),
        &attrs,
    )
    .unwrap()
}

/// The face, edge and vertex counts.
pub fn counts(b: &Brep) -> [usize; 3] {
    [
        p1::faces(b, None).unwrap().len(),
        p1::edges(b, None).unwrap().len(),
        p1::vertices(b, None).unwrap().len(),
    ]
}

/// What [`leak`] measures.
#[derive(Debug, Clone)]
pub struct Leak {
    /// The volume about the bounding box's middle (`brep_volume`'s origin).
    pub volume: f64,
    /// The largest difference of the volume about two other origins, 70 and 100 away, from it.
    pub origin_spread: f64,
    /// What the quadrature alone may leave of that spread: 1e-9 of the volume plus the
    /// area times the origins' distance (each face's integrand grows with the distance;
    /// MEASURED: closed bodies stay 10x and more below it, leaking ones 10x and more above).
    pub origin_allowance: f64,
    /// sum over edge uses of `g L max|z - o_z|`, g the trim image's sampled gap (MEASURED).
    pub old_bound: f64,
    /// sum pi tol^2 L over edges + sum 4/3 pi tol^3 over vertices.
    pub new_bound: f64,
    /// [`Leak::new_bound`] with each edge's sampled gap where that is larger than its tolerance:
    /// the closure's strips span the actual gap (MEASURED).
    pub new_bound_measured: f64,
    /// The largest sampled gap over its edge's stored tolerance.
    pub worst_gap_over_tolerance: f64,
    /// The edges with a sampled gap over their tolerance, with that ratio.
    pub strayed: Vec<(solidcore::topology::EdgeId, f64)>,
}

/// The nearest point of `curve` on `[t0, t1]` to `p` by Gauss-Newton from `guess`.
fn nearest(
    k: &Kernel,
    c: &solidcore::curve::curve::Curve,
    t0: f64,
    t1: f64,
    p: Point3,
    guess: f64,
) -> Point3 {
    let mut t = guess.max(t0).min(t1);
    for _ in 0..64 {
        let e = c.evaluate(k, t, 1, true, false).unwrap();
        let df = e[1].dot(e[1]);
        if df <= 0.0 {
            break;
        }
        let next = (t - (e[0] - p).dot(e[1]) / df).max(t0).min(t1);
        let step = (next - t).abs();
        t = next;
        if step <= 1e-15 * (1.0 + t.abs()) {
            break;
        }
    }
    c.evaluate_point(k, t).unwrap()
}

/// The checks' quantities for `b` at `accuracy`, the trims sampled at `samples` points.
pub fn leak(k: &Kernel, b: &mut Brep, accuracy: f64, samples: u32) -> Leak {
    let o = calculate_bounding_box(k, &mut *b, BoundingBoxOptions::default())
        .unwrap()
        .mid();
    let here = properties_about(k, b, o, accuracy);
    let volume = here.volume;
    let origin_allowance = 1e-9 * (volume.abs() + here.area * 100.0);
    let mut origin_spread = 0.0_f64;
    for d in [
        Point3::new(100.0, 0.0, 0.0),
        Point3::new(-37.0, 20.0, -55.0),
    ] {
        origin_spread = origin_spread.max((volume_about(k, b, o + d, accuracy) - volume).abs());
    }
    let mut new_bound = 0.0;
    for e in p1::edges(b, None).unwrap() {
        let t = edge::tolerance(b, e).unwrap();
        new_bound += PI * t * t * edge_length(k, b, e, 1e-9).unwrap();
    }
    for v in p1::vertices(b, None).unwrap() {
        let t = vertex::tolerance(b, v).unwrap();
        new_bound += 4.0 / 3.0 * PI * t * t * t;
    }
    let (mut old_bound, mut worst) = (0.0, 0.0_f64);
    let mut gaps: std::collections::BTreeMap<solidcore::topology::EdgeId, f64> =
        std::collections::BTreeMap::new();
    let mut strayed: Vec<(solidcore::topology::EdgeId, f64)> = Vec::new();
    for f in p1::faces(b, None).unwrap() {
        let Some(s) = b.face(f).unwrap().surface.clone() else {
            continue;
        };
        let Some(fu) = solidcore::topology::face::p2::upward_faceuse(b, f).unwrap() else {
            continue;
        };
        for lu in eu1::faceuse_loopuses(b, fu, false, None).unwrap() {
            for eu in eu1::loopuse_edgeuses(b, lu).unwrap() {
                let e = eu1::edge(b, eu).unwrap().unwrap();
                let Some(c) = edge::curve(b, e).unwrap() else {
                    continue;
                };
                let Some(trim) = eu1::uv_trim_curve(b, eu).unwrap().0 else {
                    continue;
                };
                let ivl = b.edge(e).unwrap().interval;
                let ti = trim.natural_interval();
                let (mut g, mut mz) = (0.0_f64, 0.0_f64);
                for i in 0..=samples {
                    let l = f64::from(i) / f64::from(samples);
                    let uv = trim
                        .evaluate_point(k, ti.min() + (ti.max() - ti.min()) * l)
                        .unwrap();
                    let p = s
                        .evaluate(k, Point2::new(uv.x, uv.y), 0, 0, EvaluateFlags::default())
                        .unwrap()[0][0];
                    // the trims run along their edges (#16735's walk): the same fraction
                    // of the edge as a guess
                    let mut q = nearest(
                        k,
                        c,
                        ivl.min(),
                        ivl.max(),
                        p,
                        ivl.min() + (ivl.max() - ivl.min()) * l,
                    );
                    if (p - q).length() > edge::tolerance(b, e).unwrap() {
                        // a trim not parameterised as its edge: the best of a coarse scan
                        let mut best = (f64::INFINITY, ivl.min());
                        for j in 0..=256 {
                            let t = ivl.min() + (ivl.max() - ivl.min()) * f64::from(j) / 256.0;
                            let d = (c.evaluate_point(k, t).unwrap() - p).length();
                            if d < best.0 {
                                best = (d, t);
                            }
                        }
                        let r = nearest(k, c, ivl.min(), ivl.max(), p, best.1);
                        if (p - r).length() < (p - q).length() {
                            q = r;
                        }
                    }
                    g = g.max((p - q).length());
                    mz = mz.max((q.z - o.z).abs());
                }
                let t = edge::tolerance(b, e).unwrap();
                old_bound += g * edge_length(k, b, e, 1e-9).unwrap() * mz;
                worst = worst.max(g / t);
                let gap = gaps.entry(e).or_insert(0.0);
                *gap = gap.max(g);
                if g > t {
                    strayed.push((e, g / t));
                }
            }
        }
    }
    let mut new_bound_measured = 0.0;
    for e in p1::edges(b, None).unwrap() {
        let t = edge::tolerance(b, e)
            .unwrap()
            .max(gaps.get(&e).copied().unwrap_or(0.0));
        new_bound_measured += PI * t * t * edge_length(k, b, e, 1e-9).unwrap();
    }
    for v in p1::vertices(b, None).unwrap() {
        let t = vertex::tolerance(b, v).unwrap();
        new_bound_measured += 4.0 / 3.0 * PI * t * t * t;
    }
    Leak {
        volume,
        origin_spread,
        origin_allowance,
        new_bound_measured,
        old_bound,
        new_bound,
        worst_gap_over_tolerance: worst,
        strayed,
    }
}

/// The checks on one volume that moved from `old` to `new` (`brep_volume` at `accuracy`):
/// it reproduces (`brep_volume` gives `new` to 1e-9); the move is within the old leak bound
/// plus twice the quadrature's own share at `accuracy` (its difference from the volume at
/// 1e-10; MEASURED); the volume does not depend on the origin beyond
/// [`Leak::origin_allowance`]; and the new value's error bound with the sampled gaps
/// ([`Leak::new_bound_measured`]) is below the old leak bound, or else (gaps well inside
/// their tolerances, where the tolerance tube is wider than anything that leaked) the move
/// itself lies inside that tube, so no value it can take is better. `Ok` with the figures, or
/// `Err` with them and the failed checks. Gaps over their edge's tolerance are reported in
/// the figures (`worst gap / tol`), not failed: the input's stored tolerance is stale there,
/// and the measured bound covers it.
pub fn moved_by_the_old_leak(
    k: &Kernel,
    b: &mut Brep,
    old: f64,
    new: f64,
    accuracy: f64,
) -> Result<String, String> {
    let store = AttributeStore::new();
    let attrs = PropertyAttributes {
        store: &store,
        key_of: &|_| ObjectKey(0),
    };
    let at_accuracy =
        solidcore::mass::integrator::properties::brep_volume(k, b, accuracy, &attrs).unwrap();
    let l = leak(k, b, 1e-10, 64);
    let quadrature = 2.0 * (at_accuracy - l.volume).abs();
    let moved = (new - old).abs();
    let line = format!(
        "volume {new} (was {old}, moved {moved:.3e}): old leak bound {:.3e} + quadrature \
         {quadrature:.3e}; origin spread {:.3e} (allowance {:.3e}); new bound {:.3e} \
         (with the gaps {:.3e}); worst gap / tol {:.3}",
        l.old_bound,
        l.origin_spread,
        l.origin_allowance,
        l.new_bound,
        l.new_bound_measured,
        l.worst_gap_over_tolerance
    );
    let reproduces = (at_accuracy - new).abs() <= 1e-9 * new.abs();
    let within_old_leak = moved <= l.old_bound + quadrature;
    let origin_free = l.origin_spread <= l.origin_allowance;
    // the new value's error bound is below the old one's; where the gaps are far inside
    // their tolerances the tube's own width is the larger, and the move must lie inside it
    // (or be the quadrature's own)
    let tighter =
        l.new_bound_measured <= l.old_bound || moved <= l.new_bound_measured || moved <= quadrature;
    if reproduces && within_old_leak && origin_free && tighter {
        Ok(line)
    } else {
        Err(format!(
            "{line} [reproduces {reproduces} (got {at_accuracy}), within old leak \
             {within_old_leak}, origin-free {origin_free}, new bound tighter or move inside \
             the tube {tighter}]"
        ))
    }
}

/// [`moved_by_the_old_leak`], panicking with its figures when a check fails.
pub fn assert_moved_by_the_old_leak(k: &Kernel, b: &mut Brep, old: f64, new: f64, accuracy: f64) {
    match moved_by_the_old_leak(k, b, old, new, accuracy) {
        Ok(line) => eprintln!("#16735 move: {line}"),
        Err(line) => panic!("#16735 move: {line}"),
    }
}
