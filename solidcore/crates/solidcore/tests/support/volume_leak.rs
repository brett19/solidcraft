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
    .volume
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
#[derive(Debug, Clone, Copy)]
pub struct Leak {
    /// The volume about the bounding box's middle (`brep_volume`'s origin).
    pub volume: f64,
    /// The largest difference of the volume about two other origins from it.
    pub origin_spread: f64,
    /// sum over edge uses of `g L max|z - o_z|`, g the trim image's sampled gap (MEASURED).
    pub old_bound: f64,
    /// sum pi tol^2 L over edges + sum 4/3 pi tol^3 over vertices.
    pub new_bound: f64,
    /// The largest sampled gap over its edge's stored tolerance.
    pub worst_gap_over_tolerance: f64,
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
    let volume = volume_about(k, b, o, accuracy);
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
            }
        }
    }
    Leak {
        volume,
        origin_spread,
        old_bound,
        new_bound,
        worst_gap_over_tolerance: worst,
    }
}
