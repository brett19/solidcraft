//! Not upstream (#16669): a certificate that a traced span's UV image on an analytic
//! sphere stays within a tolerance of the span's 3D curve.
//!
//! Upstream: none. `TestSpanAccuracy` (`source/SMLib/src/SmSurfaceIntersector.cpp`) samples
//! five interior parameters, and evaluates each UV point clamped to the domain. Near a
//! sphere's pole a span's UV Hermite cubic can leave the domain far behind (a span 0.37
//! long began at the seam crossing 3.3e-5 from the pole, with the exact chain-rule tangent
//! `u' ≈ δ/ρ²`, and its second control point was at u = −18.5). The clamped samples landed
//! near the pole and passed, but the trim made from the span missed its 3D curve by up to
//! 0.66.
//!
//! | Rust                       | Upstream |
//! |----------------------------|----------|
//! | [`sphere_span_image`]      | none     |
//! | [`sphere_bezier_image`]    | none     |
//! | `net_box` (private)        | none     |
//!
//! Deviations: the bisection of the span's parameter is bounded at [`MAX_DEPTH`] levels (at
//! most `2^MAX_DEPTH` pieces, an explicit stack, no recursion). A span the bisection does not
//! certify by then is not certified, and the trace shortens its step.

use crate::blend::fillet_section::interval::Ivl;
use crate::blend::fillet_section::station::{ders_basis, Nurbs};
use crate::geom::vector3d::Vec3;
use crate::surface::surface::{Surface, SurfaceKind};

/// The deepest bisection of a span's parameter [`sphere_span_image`] makes: at most
/// `2^MAX_DEPTH` pieces.
pub const MAX_DEPTH: u32 = 8;

/// An enclosure of a point or vector of 3-space.
type Iv3 = [Ivl; 3];

/// What [`sphere_span_image`] found.
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum SpanImage {
    /// The surface is not an analytic sphere: no certificate is made.
    NotASphere,
    /// The image is within the returned bound of the 3D span, which is at most the tolerance.
    Certified(f64),
    /// No certificate within the tolerance at [`MAX_DEPTH`] (or the net's enclosure failed).
    NotCertified,
}

/// Not upstream (#16669): whether the image on `surface` of the UV Hermite span
/// `uv = [uv0, duv0, uv1, duv1]` (from `uv0` with derivative `duv0` to `uv1` with `duv1`,
/// its parameter in `[0, 1]`) lies within `tol` of the 3D Hermite span `xyz` (the same
/// layout) at every parameter, on an analytic sphere. The UV points are in `x`, `y` (`z`
/// zero); the derivatives are those the trace scales by its step, as its span test does.
///
/// Both spans are cubic Bezier curves (their control points enclosed in rounded intervals
/// from the Hermite data). On each piece `[a, b]` of the parameter the difference
/// `D(t) = S(N(t)) − H(t)` is enclosed by the mean value form `D(m) + D'([a, b]) [−r, r]`,
/// `m` the piece's middle and `r` its half width. `D(m)` evaluates the sphere's rational net
/// at an enclosure of `N(m)`, and `D'` is `S_u N_u' + S_v N_v' − H'` with the net's first
/// derivatives enclosed over the UV box `N([a, b])` and the spans' derivatives over the
/// piece. `Surface::evaluate` evaluates a sphere through that net (its `SmBSplineSurface`
/// base), and a box past the net's knots uses its end patches continued, as the evaluator's
/// span search does. The bound is the length of the enclosure's farthest corner, rounded up.
/// Pieces that do not certify are bisected, at most [`MAX_DEPTH`] times. PROVEN: the mean
/// value theorem per component, and outward rounding throughout.
///
/// [`SpanImage::NotASphere`] for any other surface kind; [`SpanImage::NotCertified`] when a
/// piece at the deepest level does not certify, or a weight's enclosure is not positive.
#[must_use]
pub fn sphere_span_image(surface: &Surface, uv: [Vec3; 4], xyz: [Vec3; 4], tol: f64) -> SpanImage {
    certify(surface, bezier(uv), bezier(xyz), tol)
}

/// Not upstream (#16669): [`sphere_span_image`] for spans given as cubic Bezier control
/// points (`uv` and `xyz`, the parameter in `[0, 1]`), as a curve's control polygon holds
/// them after knot insertion.
#[must_use]
pub fn sphere_bezier_image(
    surface: &Surface,
    uv: [Vec3; 4],
    xyz: [Vec3; 4],
    tol: f64,
) -> SpanImage {
    let iv = |v: Vec3| [Ivl::from(v.x), Ivl::from(v.y), Ivl::from(v.z)];
    certify(surface, uv.map(iv), xyz.map(iv), tol)
}

/// The certificate of [`sphere_span_image`] for enclosed Bezier control points.
fn certify(surface: &Surface, n: [Iv3; 4], h: [Iv3; 4], tol: f64) -> SpanImage {
    let SurfaceKind::Sphere(sphere) = surface.kind() else {
        return SpanImage::NotASphere;
    };
    let Ok(net) = Nurbs::of(sphere.base().base()) else {
        return SpanImage::NotCertified;
    };
    let dn = derivative(&n);
    let dh = derivative(&h);
    let mut worst = 0.0_f64;
    let mut stack = vec![(0.0_f64, 1.0_f64, 0_u32)];
    while let Some((a, b, depth)) = stack.pop() {
        // a missing bound (an enclosure that failed) does not certify
        if let Some(bound) = piece_bound(&net, &n, &h, &dn, &dh, a, b) {
            if bound <= tol {
                worst = worst.max(bound);
                continue;
            }
        }
        if depth >= MAX_DEPTH {
            return SpanImage::NotCertified;
        }
        let m = 0.5 * (a + b);
        stack.push((m, b, depth + 1));
        stack.push((a, m, depth + 1));
    }
    SpanImage::Certified(worst)
}

/// The cubic Bezier control points of the Hermite span `[p0, d0, p1, d1]` (from `p0` with
/// derivative `d0` to `p1` with `d1`, on `[0, 1]`), enclosed.
fn bezier([p0, d0, p1, d1]: [Vec3; 4]) -> [Iv3; 4] {
    let iv = |v: Vec3| [Ivl::from(v.x), Ivl::from(v.y), Ivl::from(v.z)];
    let third = |p: Iv3, d: Iv3, sign: f64| {
        let three = Ivl::from(3.0);
        let s = Ivl::from(sign);
        [
            p[0] + s * d[0] / three,
            p[1] + s * d[1] / three,
            p[2] + s * d[2] / three,
        ]
    };
    let (q0, q3) = (iv(p0), iv(p1));
    [q0, third(q0, iv(d0), 1.0), third(q3, iv(d1), -1.0), q3]
}

/// The control points of a cubic Bezier curve's derivative (a quadratic).
fn derivative(q: &[Iv3; 4]) -> [Iv3; 3] {
    let three = Ivl::from(3.0);
    let diff = |a: Iv3, b: Iv3| {
        [
            three * (b[0] - a[0]),
            three * (b[1] - a[1]),
            three * (b[2] - a[2]),
        ]
    };
    [diff(q[0], q[1]), diff(q[1], q[2]), diff(q[2], q[3])]
}

/// The blossom (polar form) of the Bezier curve `q` at the arguments `ts` (as many as its
/// degree), enclosed: de Casteljau's algorithm with argument `ts[l]` at level `l`. With
/// every argument `t` it is the curve's point at `t`.
fn blossom(q: &[Iv3], ts: &[f64]) -> Option<Iv3> {
    let mut work = q.to_vec();
    for level in 1..work.len() {
        let t = Ivl::from(*ts.get(level - 1)?);
        let s = Ivl::from(1.0) - t;
        for i in 0..work.len() - level {
            let (a, b) = (*work.get(i)?, *work.get(i + 1)?);
            let slot = work.get_mut(i)?;
            *slot = [
                s * a[0] + t * b[0],
                s * a[1] + t * b[1],
                s * a[2] + t * b[2],
            ];
        }
    }
    work.first().copied()
}

/// An enclosure of the Bezier curve `q` over `[a, b]`: the hull of the control points of
/// its piece there (the blossoms at `a` and `b` repeated), which hold the piece (the convex
/// hull property). The arguments are exact, so only rounding widens the enclosure, not
/// the size of the coordinates (de Casteljau's algorithm with an interval argument widens
/// it by their size).
fn piece_hull(q: &[Iv3], a: f64, b: f64) -> Option<Iv3> {
    let degree = q.len().checked_sub(1)?;
    let mut out: Option<Iv3> = None;
    for j in 0..=degree {
        let ts: Vec<f64> = (0..degree).map(|i| if i < j { b } else { a }).collect();
        let c = blossom(q, &ts)?;
        out = Some(match out {
            None => c,
            Some(o) => hull3(o, c),
        });
    }
    out
}

/// The bound of `|S(N(t)) − H(t)|` over `[a, b]` ([`sphere_span_image`]), rounded up.
fn piece_bound(
    net: &Nurbs,
    n: &[Iv3; 4],
    h: &[Iv3; 4],
    dn: &[Iv3; 3],
    dh: &[Iv3; 3],
    a: f64,
    b: f64,
) -> Option<f64> {
    let m = 0.5 * (a + b);
    let r = Ivl::from(b - m).hull(Ivl::from(m - a)).hi;
    let nm = blossom(n, &[m; 3])?;
    let hm = blossom(h, &[m; 3])?;
    let (sm, _, _) = net_box(net, nm[0], nm[1])?;
    let nbox = piece_hull(n, a, b)?;
    let dnbox = piece_hull(dn, a, b)?;
    let dhbox = piece_hull(dh, a, b)?;
    let (_, su, sv) = net_box(net, nbox[0], nbox[1])?;
    let spread = Ivl::new(-r, r);
    let mut sum = Ivl::from(0.0);
    for i in 0..3 {
        let d0 = *sm.get(i)? - *hm.get(i)?;
        let d1 = *su.get(i)? * dnbox[0] + *sv.get(i)? * dnbox[1] - *dhbox.get(i)?;
        let c = Ivl::from((d0 + d1 * spread).mag());
        sum = sum + c * c;
    }
    let bound = sum.sqrt().hi;
    bound.is_finite().then_some(bound)
}

/// The spans `k` of the knot vector `knots` (degree `deg`, `count` control points) whose
/// pieces meet `x`, the end spans continued past the knots' range.
fn spans_meeting(knots: &[f64], deg: usize, count: usize, x: Ivl) -> Vec<usize> {
    let mut out = Vec::new();
    for k in deg..count {
        let (Some(&lo), Some(&hi)) = (knots.get(k), knots.get(k + 1)) else {
            continue;
        };
        let first = k == deg;
        let last = k + 1 == count;
        let meets = hi > lo && (first || x.hi >= lo) && (last || x.lo <= hi);
        if meets {
            out.push(k);
        }
    }
    out
}

/// Enclosures of the sphere net's point and first derivatives over the UV box `u` × `v`:
/// the hull over every polynomial patch the box meets, each evaluated over the box's part
/// in it (the end patches continued past the knots' range). `None` where a weight's
/// enclosure is not positive or no patch meets the box.
fn net_box(net: &Nurbs, u: Ivl, v: Ivl) -> Option<(Iv3, Iv3, Iv3)> {
    let mut out: Option<(Iv3, Iv3, Iv3)> = None;
    for ku in spans_meeting(&net.u, net.p, net.nu, u) {
        let uu = clip(&net.u, net.p, net.nu, ku, u)?;
        for kv in spans_meeting(&net.v, net.q, net.nv, v) {
            let vv = clip(&net.v, net.q, net.nv, kv, v)?;
            let patch = patch_box(net, ku, kv, uu, vv)?;
            out = Some(match out {
                None => patch,
                Some(o) => (
                    hull3(o.0, patch.0),
                    hull3(o.1, patch.1),
                    hull3(o.2, patch.2),
                ),
            });
        }
    }
    out
}

/// The part of `x` in span `k` (all of it past the knots' range at an end span).
fn clip(knots: &[f64], deg: usize, count: usize, k: usize, x: Ivl) -> Option<Ivl> {
    let lo = if k == deg {
        x.lo
    } else {
        x.lo.max(*knots.get(k)?)
    };
    let hi = if k + 1 == count {
        x.hi
    } else {
        x.hi.min(*knots.get(k + 1)?)
    };
    let ordered = lo <= hi;
    ordered.then(|| Ivl::new(lo, hi))
}

/// The componentwise hull of two enclosures.
fn hull3(a: Iv3, b: Iv3) -> Iv3 {
    [a[0].hull(b[0]), a[1].hull(b[1]), a[2].hull(b[2])]
}

/// The point and first derivatives of the net's patch `(ku, kv)` over `u` × `v`, enclosed:
/// the homogeneous sums and the quotient rule `S_u = (A_u − w_u S) / w`.
fn patch_box(net: &Nurbs, ku: usize, kv: usize, u: Ivl, v: Ivl) -> Option<(Iv3, Iv3, Iv3)> {
    let nu = ders_basis(&net.u, ku, net.p, u, 1);
    let nv = ders_basis(&net.v, kv, net.q, v, 1);
    let basis = |m: &Vec<Vec<Ivl>>, a: usize, i: usize| m.get(a).and_then(|r| r.get(i)).copied();
    let mut h = [[[Ivl::from(0.0); 4]; 2]; 2];
    for (a, row) in h.iter_mut().enumerate() {
        for (b, hab) in row.iter_mut().enumerate() {
            for i in 0..=net.p {
                for j in 0..=net.q {
                    let li = 4
                        * ((ku + i).checked_sub(net.p)? * net.nv + (kv + j).checked_sub(net.q)?);
                    let c = basis(&nu, a, i)? * basis(&nv, b, j)?;
                    for (e, he) in hab.iter_mut().enumerate() {
                        *he = *he + c * Ivl::from(*net.pw.get(li + e)?);
                    }
                }
            }
        }
    }
    let [[h00, h01], [h10, _]] = h;
    let w = h00[3];
    let positive = w.lo > 0.0;
    if !positive {
        return None;
    }
    let point = [h00[0] / w, h00[1] / w, h00[2] / w];
    let rate = |d: [Ivl; 4]| {
        [
            (d[0] - d[3] * point[0]) / w,
            (d[1] - d[3] * point[1]) / w,
            (d[2] - d[3] * point[2]) / w,
        ]
    };
    Some((point, rate(h10), rate(h01)))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::geom::axis2_placement::Axis2Placement;
    use crate::geom::vector2d::Point2;
    use crate::kernel::Kernel;
    use crate::nurbs::b_spline_surface::BSplineSurface;
    use crate::nurbs::b_spline_surface_n_lib::p2::EvaluateFlags;
    use crate::surface::sphere::Sphere;

    fn ball(k: &Kernel) -> Surface {
        Surface::from(
            Sphere::create_canonical(
                k,
                &Axis2Placement::new(
                    Vec3::new(1.0, -2.0, 3.0),
                    Vec3::new(0.0, 0.6, 0.8),
                    Vec3::new(1.0, 0.0, 0.0),
                ),
                6.0,
            )
            .unwrap(),
        )
    }

    /// The point at `t` of the Hermite span `[p0, d0, p1, d1]` on `[0, 1]`.
    fn hermite([p0, d0, p1, d1]: [Vec3; 4], t: f64) -> Vec3 {
        let (t2, t3) = (t * t, t * t * t);
        p0 * (2.0 * t3 - 3.0 * t2 + 1.0)
            + d0 * (t3 - 2.0 * t2 + t)
            + p1 * (-2.0 * t3 + 3.0 * t2)
            + d1 * (t3 - t2)
    }

    /// A UV span on the ball from `a` to `b` (its chord as both derivatives) and the 3D
    /// Hermite span through the image's end points with the chain-rule derivatives.
    fn spans(k: &Kernel, s: &Surface, a: Point2, b: Point2) -> ([Vec3; 4], [Vec3; 4]) {
        let d = b - a;
        let image = |p: Point2| {
            let m = s.evaluate(k, p, 1, 1, EvaluateFlags::default()).unwrap();
            (m[0][0], m[1][0] * d.x + m[0][1] * d.y)
        };
        let ((p0, d0), (p1, d1)) = (image(a), image(b));
        let lift = |p: Point2| Vec3::new(p.x, p.y, 0.0);
        let dv = Vec3::new(d.x, d.y, 0.0);
        ([lift(a), dv, lift(b), dv], [p0, d0, p1, d1])
    }

    /// The largest distance between the image and the 3D span at `n + 1` parameters.
    fn sampled(k: &Kernel, s: &Surface, uv: [Vec3; 4], xyz: [Vec3; 4], n: u32) -> f64 {
        (0..=n)
            .map(|i| {
                let t = f64::from(i) / f64::from(n);
                let q = hermite(uv, t);
                let p = s.evaluate_point(k, Point2::new(q.x, q.y)).unwrap();
                p.distance(hermite(xyz, t))
            })
            .fold(0.0, f64::max)
    }

    /// The bound holds the sampled distances and is at most the tolerance, on spans inside
    /// one patch, across a knot, and near a pole.
    #[test]
    fn the_bound_holds_every_sampled_distance() {
        let k = &Kernel::default();
        let s = ball(k);
        let cases = [
            (Point2::new(0.05, 0.3), Point2::new(0.1, 0.4)),
            (Point2::new(0.23, 0.48), Point2::new(0.27, 0.52)),
            (Point2::new(0.6, 0.97), Point2::new(0.62, 0.999)),
            (Point2::new(0.77, 0.15), Point2::new(0.73, 0.17)),
        ];
        for (a, b) in cases {
            let (uv, xyz) = spans(k, &s, a, b);
            let seen = sampled(k, &s, uv, xyz, 2000);
            let tol = 1e-2;
            assert!(seen < tol / 4.0, "{a:?} {b:?}: sampled {seen}");
            match sphere_span_image(&s, uv, xyz, tol) {
                SpanImage::Certified(bound) => {
                    assert!(seen <= bound && bound <= tol, "{a:?} {b:?}: {seen} {bound}");
                }
                other => panic!("{a:?} {b:?}: {other:?}, sampled {seen}"),
            }
        }
    }

    /// A UV span whose second control point leaves the domain far behind (as the #16669
    /// span's did, u = −18.5) is not certified, though its ends are on the 3D span.
    #[test]
    fn a_span_whose_uv_image_leaves_the_curve_is_not_certified() {
        let k = &Kernel::default();
        let s = ball(k);
        let (mut uv, xyz) = spans(k, &s, Point2::new(0.97, 0.9), Point2::new(0.99, 0.99));
        uv[1] = Vec3::new(-30.0, uv[1].y, 0.0);
        assert!(sampled(k, &s, uv, xyz, 2000) > 0.1);
        assert_eq!(
            sphere_span_image(&s, uv, xyz, 1e-2),
            SpanImage::NotCertified
        );
    }

    /// A tolerance of zero is not reached by the bisection: not certified at the depth
    /// bound.
    #[test]
    fn a_zero_tolerance_is_not_certified() {
        let k = &Kernel::default();
        let s = ball(k);
        let (uv, xyz) = spans(k, &s, Point2::new(0.2, 0.3), Point2::new(0.21, 0.31));
        assert_eq!(sphere_span_image(&s, uv, xyz, 0.0), SpanImage::NotCertified);
    }

    /// Other surface kinds get no certificate.
    #[test]
    fn a_plane_is_not_a_sphere() {
        let s = Surface::from(
            BSplineSurface::create_bilinear_surface(
                Vec3::new(0.0, 0.0, 0.0),
                Vec3::new(1.0, 0.0, 0.0),
                Vec3::new(0.0, 1.0, 0.0),
                Vec3::new(1.0, 1.0, 0.0),
            )
            .unwrap(),
        );
        let z = [Vec3::ZERO; 4];
        assert_eq!(sphere_span_image(&s, z, z, 1.0), SpanImage::NotASphere);
    }
}
