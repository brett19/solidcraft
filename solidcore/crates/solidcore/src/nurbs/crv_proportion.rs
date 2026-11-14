//! A curve made longer keeping its control points' proportions: the same knots and count, the
//! control polygon stretched along itself from the far end.
//!
//! Not upstream. To lengthen a curve at one end by `dist`, the control polygon is read from the
//! other end (pinned) by its cumulative chord `s_i` (`s_0 = 0`, `S` its length), and continued
//! past the moving end along that end's last leg. The polygon is stretched to `S + d`: control
//! point `i` goes to the point at `s_i (S + d) / S` along the continued polygon, so each keeps
//! its share of the polygon from the pinned end, the pinned end stays, and the moving end goes
//! out along its leg by `d`. The weights stay.
//!
//! `d` is found so the curve's arc length grows by `dist`. The arc length is not monotone in
//! `d`: where the polygon turns back (a hook at the moving end), points slide round the turn
//! and the curve can get shorter as `d` grows. It is continuous in `d` (each point moves
//! continuously along the polygon), it is the curve's own length at `d = 0`, and it grows
//! without bound (every point but the pinned one runs out along the end leg, so the moving end
//! leaves the pinned one). So a `d` whose length is the one asked exists past any bracket
//! `[d_lo, d_hi]` with the measured length short at `d_lo` and not short at `d_hi`: `d_hi` is
//! found by doubling from `dist` ([`MAX_DOUBLINGS`]), then the root by regula falsi with the
//! Illinois step (at most [`MAX_STEPS`]), on the measured length ([`Curve::arc_length`]).
//!
//! The result is accepted only on a certified bound: enclosures of the arc lengths of the curve
//! and of the result (`length_enclosure`: per Bézier piece, the chord below and the control
//! polygon above, or for a rational piece the bound from its derivative, rounding counted,
//! pieces halved until the enclosure is narrow) whose difference lies within [`ACCEPT_TOL`]
//! of `dist`, relative to the length asked. Otherwise the extension is refused
//! ([`ExtendRefusal::LengthNotMet`]); it never returns a curve short of, or past, the length.
//!
//! What is kept: the knots, the degree, the count and weights of the control points, and each
//! control point's share of the polygon. What is not: the curve over its old domain (the points
//! slide along the polygon, so the shape near the pinned end moves a little as the ones near the
//! moving end move more).
//!
//! Refused: a length not finite and above 0, a curve of fewer than two control points, a
//! polygon of no length, or an end leg of no length behind the moving end (every leg from it
//! back of no length), as invalid input; a length not met or not certified as above.
//!
//! Deviations: none (not upstream); no recursion (the enclosure's pieces are refined in rounds
//! over a vector, at most [`MAX_PIECES`] pieces, each halved at most [`MAX_DEPTH`] times).

use crate::error::{Result, SolidError};
use crate::math::Vec3;
use crate::nurbs::crv_adv::p1::CurveEnd;
use crate::nurbs::crv_basic::Curve;
use crate::nurbs::crv_geom::ToleranceKind;
use crate::nurbs::math::{gamma, CPoint, NOW, NOZ};
use crate::surface::extend::ExtendRefusal;

const OP: &str = "nurbs::crv_proportion::extend_keeping_proportion";

/// The most regula falsi steps on the stretch, once it is bracketed.
pub const MAX_STEPS: usize = 60;

/// The most doublings of the stretch to bracket the length asked.
pub const MAX_DOUBLINGS: usize = 64;

/// The relative tolerance the arc lengths are measured to in the search.
const LENGTH_TOL: f64 = 1e-10;

/// How close, relative to the length asked, the certified growth in arc length must be to the
/// length asked for the extension to be accepted.
pub const ACCEPT_TOL: f64 = 1e-7;

/// The most Bézier pieces an arc-length enclosure refines to.
pub const MAX_PIECES: usize = 1 << 15;

/// The most times one piece of an enclosure is halved.
pub const MAX_DEPTH: u32 = 40;

/// The most refinement rounds of an enclosure.
const MAX_ROUNDS: usize = 64;

fn invalid(reason: &'static str) -> SolidError {
    SolidError::invalid_input(OP, reason)
}

fn not_met() -> SolidError {
    SolidError::ExtendRefused {
        operation: OP,
        refusal: ExtendRefusal::LengthNotMet,
        distance: None,
    }
}

/// The control polygon from the pinned end, its cumulative chord and the moving end's unit leg
/// direction.
struct Polygon {
    points: Vec<Vec3>,
    chord: Vec<f64>,
    out: Vec3,
}

impl Polygon {
    fn of(points: Vec<Vec3>) -> Result<Polygon> {
        let mut chord = Vec::new();
        let mut s = 0.0;
        chord.push(0.0);
        for w in points.windows(2) {
            if let [a, b] = w {
                s += (*b - *a).length();
            }
            chord.push(s);
        }
        if !(s > 0.0 && s.is_finite()) {
            return Err(invalid("a control polygon of no length"));
        }
        // the last leg of some length, from the moving end back
        let out = points
            .windows(2)
            .rev()
            .find_map(|w| match w {
                [a, b] => (*b - *a).unitize().ok(),
                _ => None,
            })
            .ok_or_else(|| invalid("no leg behind the moving end"))?;
        Ok(Polygon { points, chord, out })
    }

    fn length(&self) -> f64 {
        self.chord.last().copied().unwrap_or(0.0)
    }

    /// The point at chord length `t` along the polygon, continued past its moving end.
    fn at(&self, t: f64) -> Vec3 {
        let (Some(&last), Some(&end)) = (self.points.last(), self.chord.last()) else {
            return Vec3::ZERO;
        };
        if t >= end {
            return last + self.out * (t - end);
        }
        for (k, w) in self.chord.windows(2).enumerate() {
            if let ([s0, s1], Some(&a), Some(&b)) = (w, self.points.get(k), self.points.get(k + 1))
            {
                if t <= *s1 {
                    let span = s1 - s0;
                    return if span > 0.0 {
                        a + (b - a) * ((t - s0) / span)
                    } else {
                        a
                    };
                }
            }
        }
        last
    }

    /// Each point moved to its share of the polygon stretched by `d`.
    fn stretched(&self, d: f64) -> Vec<Vec3> {
        let scale = (self.length() + d) / self.length();
        self.chord.iter().map(|s| self.at(s * scale)).collect()
    }
}

impl Curve {
    /// This curve made `dist` longer at `end`, its control polygon stretched along itself from
    /// the other end (module notes), and the curve's measured arc length reached. The growth in
    /// arc length is certified within [`ACCEPT_TOL`] of `dist`, relative to the length asked.
    ///
    /// # Errors
    ///
    /// `InvalidInput` for the refusals the module names; [`SolidError::ExtendRefused`] with
    /// [`ExtendRefusal::LengthNotMet`] where the length is not bracketed, not found, or not
    /// certified; those of [`Curve::arc_length`] and the Bézier decomposition.
    pub fn extend_keeping_proportion(&self, dist: f64, end: CurveEnd) -> Result<(Curve, f64)> {
        if !(dist.is_finite() && dist > 0.0) {
            return Err(invalid("a length not finite and above 0"));
        }
        let given: Vec<(Vec3, f64)> = self.cpts().iter().map(|c| c.to_euclid_and_w()).collect();
        if given.len() < 2 {
            return Err(invalid("fewer than two control points"));
        }
        let weights: Vec<f64> = given.iter().map(|(_, w)| *w).collect();
        let mut points: Vec<Vec3> = given.iter().map(|(p, _)| *p).collect();
        if end == CurveEnd::Start {
            points.reverse();
        }
        let polygon = Polygon::of(points)?;
        let (Some(&lo), Some(&hi)) = (self.knots().first(), self.knots().last()) else {
            return Err(invalid("a curve without knots"));
        };
        // a length the quadrature cannot measure to its stop is not met
        let length = |c: &Curve| match c.arc_length(lo, hi, LENGTH_TOL, ToleranceKind::Relative) {
            Err(SolidError::NotConverged { .. }) => Err(not_met()),
            other => other,
        };
        let make = |d: f64| -> Curve {
            let mut moved = polygon.stretched(d);
            if end == CurveEnd::Start {
                moved.reverse();
            }
            let mut c = self.clone();
            for ((slot, p), w) in c.cpts_mut().iter_mut().zip(moved).zip(&weights) {
                *slot = CPoint::weighted(p, *w);
            }
            c
        };
        let before = length(self)?;
        let target = before + dist;
        if !target.is_finite() {
            return Err(not_met());
        }
        // the bracket: short at d_lo (the curve itself), not short at d_hi
        let (mut d_lo, mut g_lo) = (0.0_f64, -dist);
        let mut d_hi = dist;
        let mut g_hi = length(&make(d_hi))? - target;
        let mut doublings = 0;
        // (a NaN length is not finite, and refuses)
        while !crate::geom::certified::above(g_hi, 0.0) && g_hi != 0.0 {
            if doublings >= MAX_DOUBLINGS || !g_hi.is_finite() {
                return Err(not_met());
            }
            (d_lo, g_lo) = (d_hi, g_hi);
            d_hi *= 2.0;
            if !d_hi.is_finite() {
                return Err(not_met());
            }
            g_hi = length(&make(d_hi))? - target;
            doublings += 1;
        }
        // regula falsi with the Illinois step, keeping the bracket
        let stop = LENGTH_TOL * target.max(1.0);
        let (mut best_d, mut best_g) = if g_hi.abs() < g_lo.abs() {
            (d_hi, g_hi)
        } else {
            (d_lo, g_lo)
        };
        let mut side = 0_i8;
        for _ in 0..MAX_STEPS {
            if best_g.abs() <= stop {
                break;
            }
            let d = (d_lo * g_hi - d_hi * g_lo) / (g_hi - g_lo);
            let d = if d.is_finite() && d > d_lo && d < d_hi {
                d
            } else {
                0.5 * (d_lo + d_hi)
            };
            if !(d > d_lo && d < d_hi) {
                break;
            }
            let g = length(&make(d))? - target;
            if !g.is_finite() {
                return Err(not_met());
            }
            if g.abs() < best_g.abs() {
                (best_d, best_g) = (d, g);
            }
            if g < 0.0 {
                (d_lo, g_lo) = (d, g);
                if side == -1 {
                    g_hi *= 0.5;
                }
                side = -1;
            } else {
                (d_hi, g_hi) = (d, g);
                if side == 1 {
                    g_lo *= 0.5;
                }
                side = 1;
            }
        }
        let best = make(best_d);
        let reached = target + best_g;
        // accepted on the certified growth only
        let width = 0.25 * ACCEPT_TOL * target;
        let (Some(old), Some(new)) = (
            length_enclosure(self, width)?,
            length_enclosure(&best, width)?,
        ) else {
            return Err(not_met());
        };
        let grew = (new.0 - old.1, new.1 - old.0);
        let margin = ACCEPT_TOL * target;
        // the differences' rounding, outward
        let pad = gamma(2) * (new.1 + old.1);
        if grew.0 - pad >= dist - margin && grew.1 + pad <= dist + margin {
            Ok((best, reached))
        } else {
            Err(not_met())
        }
    }
}

/// A Bézier piece of a curve: its homogeneous control points (`[w x, w y, w z, w]`), the bound
/// on each coefficient's rounding, and how many times it was halved.
#[derive(Debug, Clone)]
struct Piece {
    h: Vec<[f64; 4]>,
    err: f64,
    depth: u32,
    polynomial: bool,
    lo: f64,
    hi: f64,
}

/// The largest absolute coefficient of `h`.
fn largest(h: &[[f64; 4]]) -> f64 {
    h.iter().flatten().fold(0.0_f64, |m, v| m.max(v.abs()))
}

impl Piece {
    fn new(h: Vec<[f64; 4]>, err: f64, depth: u32, polynomial: bool) -> Option<Piece> {
        let (lo, hi) = bounds(&h, err, polynomial)?;
        Some(Piece {
            h,
            err,
            depth,
            polynomial,
            lo,
            hi,
        })
    }

    /// Its halves, by de Casteljau at the middle: each level a convex combination, so each
    /// coefficient within `gamma(2 n + 2)` of the largest more (as the rail certificate of the
    /// tangent rollover counts a subdivision).
    fn halves(&self) -> Option<(Piece, Piece)> {
        let n = self.h.len();
        let mut work = self.h.clone();
        let mut left = Vec::new();
        let mut right = vec![[0.0; 4]; n];
        for level in 0..n {
            left.push(*work.first()?);
            *right.get_mut(n - 1 - level)? = *work.get(n - 1 - level)?;
            for i in 0..n - 1 - level {
                let (a, b) = (*work.get(i)?, *work.get(i + 1)?);
                *work.get_mut(i)? = [
                    0.5 * (a[0] + b[0]),
                    0.5 * (a[1] + b[1]),
                    0.5 * (a[2] + b[2]),
                    0.5 * (a[3] + b[3]),
                ];
            }
        }
        let err = self.err + gamma(2 * n + 2) * largest(&self.h);
        Some((
            Piece::new(left, err, self.depth + 1, self.polynomial)?,
            Piece::new(right, err, self.depth + 1, self.polynomial)?,
        ))
    }
}

/// The certified enclosure `(lo, hi)` of a Bézier piece's arc length, from its homogeneous
/// control points `h`, each coefficient within `err` of exact, `polynomial` for a piece of a
/// curve that is not rational (every weight exactly 1, the NURB's own marker). `None` where
/// the weights are not proved positive or a value is not finite.
///
/// The Euclidean points `P_i` are within `widen` of exact in each coordinate (the quotient of
/// coefficients within `err` by a weight at least `w_min - err`, and its own rounding), so a
/// computed distance between two of them is within `4 widen` of exact (`2 sqrt 3 < 4`), and
/// the sums carry `gamma(8)` relative.
/// - Below: the chord `|P_n - P_0|`, which no curve between the two ends is shorter than.
/// - Above, polynomial (every weight 1): the control polygon's length, which the curve, the
///   image of the polygon by de Casteljau's corner cutting, is no longer than.
/// - Above, rational: with `W = sum w_i B_i`, `C' = (n / W) sum_i B^{n-1}_i (w_{i+1} dP_i +
///   dw_i (P_i - C))` (`dP_i = P_{i+1} - P_i`, `dw_i = w_{i+1} - w_i`), and `C` in the hull of
///   the `P_i`, so `|P_i - C| <= D`, the diagonal of their box. With `int n B^{n-1}_i = 1`,
///   the length is at most `(w_max / w_min) sum |dP_i| + D sum |dw_i| / w_min`.
fn bounds(h: &[[f64; 4]], err: f64, polynomial: bool) -> Option<(f64, f64)> {
    if !err.is_finite() || !h.iter().flatten().all(|v| v.is_finite()) {
        return None;
    }
    let w_min = h.iter().map(|c| c[3]).fold(f64::INFINITY, f64::min);
    let w_max = h.iter().map(|c| c[3]).fold(f64::NEG_INFINITY, f64::max);
    if !crate::geom::certified::above(w_min - err, 0.0) {
        return None;
    }
    let points: Vec<Vec3> = h
        .iter()
        .map(|c| Vec3::new(c[0] / c[3], c[1] / c[3], c[2] / c[3]))
        .collect();
    let size = points
        .iter()
        .flat_map(|p| [p.x.abs(), p.y.abs(), p.z.abs()])
        .fold(0.0_f64, f64::max);
    let widen = err * (1.0 + size) / (w_min - err) + gamma(2) * size;
    let slack = 4.0 * widen;
    let (Some(&first), Some(&last)) = (points.first(), points.last()) else {
        return None;
    };
    let chord = (last - first).length();
    let lo = (chord * (1.0 - gamma(8)) - slack).max(0.0);
    let legs = points.len().saturating_sub(1);
    #[allow(clippy::cast_precision_loss)] // a piece has a few points
    let legs_f = legs as f64;
    let polygon: f64 = points
        .windows(2)
        .map(|w| match w {
            [a, b] => (*b - *a).length(),
            _ => 0.0,
        })
        .sum();
    let polygon_hi = polygon * (1.0 + gamma(8)) + legs_f * slack;
    let hi = if polynomial {
        polygon_hi
    } else {
        let (mut bmin, mut bmax) = (
            Vec3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
            Vec3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
        );
        for p in &points {
            bmin = Vec3::new(bmin.x.min(p.x), bmin.y.min(p.y), bmin.z.min(p.z));
            bmax = Vec3::new(bmax.x.max(p.x), bmax.y.max(p.y), bmax.z.max(p.z));
        }
        let diagonal = (bmax - bmin).length() * (1.0 + gamma(8)) + slack;
        let dw: f64 = h
            .windows(2)
            .map(|w| match w {
                [a, b] => (b[3] - a[3]).abs() + 2.0 * err,
                _ => 0.0,
            })
            .sum();
        let low_w = w_min - err;
        ((w_max + err) / low_w * polygon_hi + diagonal * dw / low_w) * (1.0 + gamma(8))
    };
    (lo.is_finite() && hi.is_finite()).then_some((lo, hi))
}

/// A certified enclosure `(lo, hi)` of `curve`'s arc length no wider than `width`, from its
/// Bézier pieces, the pieces whose enclosure is at least half the average width halved in
/// rounds;
/// `None` where it is not reached within [`MAX_PIECES`] pieces, [`MAX_DEPTH`] halvings or
/// `MAX_ROUNDS` rounds, or a piece is not bounded (weights not proved positive, a value not
/// finite, a 2D net). The decomposition's knot insertions leave each coefficient within
/// `gamma(16 (p + 1))` of the curve's largest (as the rail certificate of the tangent rollover
/// counts them); the sums of the pieces carry `gamma(count)` relative.
///
/// # Errors
/// Those of the decomposition.
fn length_enclosure(curve: &Curve, width: f64) -> Result<Option<(f64, f64)>> {
    if !(width.is_finite() && width > 0.0) {
        return Ok(None);
    }
    let homogeneous = |c: &CPoint| -> Option<[f64; 4]> {
        if c.z == NOZ {
            return None;
        }
        if c.w == NOW {
            Some([c.x, c.y, c.z, 1.0])
        } else {
            Some([c.x, c.y, c.z, c.w])
        }
    };
    let all: Option<Vec<[f64; 4]>> = curve.cpts().iter().map(homogeneous).collect();
    let Some(all) = all else {
        return Ok(None);
    };
    let polynomial = !curve.is_rational();
    let decomposed = curve.knots().len() > 2 * (curve.degree() + 1);
    let err0 = if decomposed {
        gamma(16 * (curve.degree() + 1)) * largest(&all)
    } else {
        0.0
    };
    let mut pieces = Vec::new();
    for bezier in curve.decompose_bez()? {
        let h: Option<Vec<[f64; 4]>> = bezier.cpts().iter().map(homogeneous).collect();
        let Some(piece) = h.and_then(|h| Piece::new(h, err0, 0, polynomial)) else {
            return Ok(None);
        };
        pieces.push(piece);
    }
    for _ in 0..MAX_ROUNDS {
        #[allow(clippy::cast_precision_loss)] // the count is at most MAX_PIECES
        let count = pieces.len() as f64;
        let sum = |f: &dyn Fn(&Piece) -> f64| pieces.iter().map(f).sum::<f64>();
        let rounding = gamma(pieces.len());
        let lo = sum(&|p| p.lo) * (1.0 - rounding);
        let hi = sum(&|p| p.hi) * (1.0 + rounding);
        if !(lo.is_finite() && hi.is_finite()) {
            return Ok(None);
        }
        if hi - lo <= width {
            return Ok(Some((lo, hi)));
        }
        let average = (hi - lo) / count;
        let mut next = Vec::new();
        for piece in pieces {
            if piece.hi - piece.lo >= 0.5 * average {
                if piece.depth >= MAX_DEPTH {
                    return Ok(None);
                }
                let Some((a, b)) = piece.halves() else {
                    return Ok(None);
                };
                next.push(a);
                next.push(b);
            } else {
                next.push(piece);
            }
            if next.len() > MAX_PIECES {
                return Ok(None);
            }
        }
        pieces = next;
    }
    Ok(None)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn curve(points: &[[f64; 3]], degree: usize) -> Curve {
        let pw: Vec<CPoint> = points
            .iter()
            .map(|p| CPoint::from_point(Vec3::new(p[0], p[1], p[2])))
            .collect();
        let n = pw.len();
        let mut knots = vec![0.0; degree + 1];
        for i in 1..n - degree {
            knots.push(i as f64 / (n - degree) as f64);
        }
        knots.extend(vec![1.0; degree + 1]);
        Curve::new(pw, degree, knots).unwrap()
    }

    fn cvs(c: &Curve) -> Vec<Vec3> {
        c.cpts().iter().map(|p| p.to_euclid()).collect()
    }

    /// A straight line 10 long made 5 longer at its end: the end at 15, the start where it was.
    #[test]
    fn a_line_grows_along_itself_from_its_pinned_start() {
        let line = curve(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], 1);
        let (out, reached) = line.extend_keeping_proportion(5.0, CurveEnd::End).unwrap();
        assert!((reached - 15.0).abs() < 1e-6, "{reached}");
        let p = cvs(&out);
        assert!(
            (p[0] - Vec3::ZERO).length() < 1e-12
                && (p[1] - Vec3::new(15.0, 0.0, 0.0)).length() < 1e-6,
            "{p:?}"
        );
    }

    /// A straight cubic with CVs at x = 0, 1, 3 and 6 made 3 longer at its end: the CVs at
    /// 0, 1.5, 4.5 and 9, each at its share of the polygon as before, the knots kept.
    #[test]
    fn a_straight_cubic_keeps_each_cvs_share_of_its_polygon() {
        let c = curve(
            &[
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
                [6.0, 0.0, 0.0],
            ],
            3,
        );
        let (out, reached) = c.extend_keeping_proportion(3.0, CurveEnd::End).unwrap();
        assert!((reached - 9.0).abs() < 1e-6, "{reached}");
        assert_eq!(out.knots(), c.knots());
        let p = cvs(&out);
        for (got, x) in p.iter().zip([0.0, 1.5, 4.5, 9.0]) {
            assert!((*got - Vec3::new(x, 0.0, 0.0)).length() < 1e-6, "{p:?}");
        }
    }

    /// A curved cubic made 3 longer: the same count and knots, the start CV fixed, the arc
    /// length 3 more.
    #[test]
    fn a_curved_cubic_grows_by_the_length_asked() {
        let c = curve(
            &[
                [0.0, 0.0, 0.0],
                [2.0, 3.0, 0.0],
                [6.0, 3.0, 0.0],
                [9.0, 0.0, 0.0],
            ],
            3,
        );
        let before = c
            .arc_length(0.0, 1.0, 1e-9, ToleranceKind::Relative)
            .unwrap();
        let (out, reached) = c.extend_keeping_proportion(3.0, CurveEnd::End).unwrap();
        assert!((reached - before - 3.0).abs() < 1e-6, "{before} {reached}");
        assert_eq!(out.knots(), c.knots());
        let (old, new) = (cvs(&c), cvs(&out));
        assert_eq!(old.len(), new.len());
        assert!((old[0] - new[0]).length() < 1e-12, "the start stays");
    }

    /// Grown at its start, the end stays.
    #[test]
    fn grown_at_its_start_the_end_stays() {
        let c = curve(
            &[
                [0.0, 0.0, 0.0],
                [2.0, 3.0, 0.0],
                [6.0, 3.0, 0.0],
                [9.0, 0.0, 0.0],
            ],
            3,
        );
        let (out, _) = c.extend_keeping_proportion(2.0, CurveEnd::Start).unwrap();
        let (old, new) = (cvs(&c), cvs(&out));
        assert!((old[3] - new[3]).length() < 1e-12, "the end stays");
        assert!((new[0] - old[0]).length() > 1.0, "the start moves out");
    }

    #[test]
    fn bad_lengths_and_polygons_are_refused() {
        let c = curve(&[[0.0, 0.0, 0.0], [10.0, 0.0, 0.0]], 1);
        assert!(c.extend_keeping_proportion(0.0, CurveEnd::End).is_err());
        assert!(c
            .extend_keeping_proportion(f64::NAN, CurveEnd::End)
            .is_err());
        let point = curve(&[[1.0, 1.0, 1.0], [1.0, 1.0, 1.0]], 1);
        assert!(point.extend_keeping_proportion(1.0, CurveEnd::End).is_err());
    }

    fn length_of(c: &Curve) -> f64 {
        let (lo, hi) = (c.knots()[0], *c.knots().last().unwrap());
        c.arc_length(lo, hi, 1e-10, ToleranceKind::Relative)
            .unwrap()
    }

    /// The outcome of growing `c` by `dist` at `end`: an extension whose growth in arc length is
    /// the length asked, by a certified enclosure of both curves' lengths within the acceptance
    /// tolerance and by the measured lengths, or the named refusal. Panics on anything else.
    fn check(c: &Curve, dist: f64, end: CurveEnd, what: &str) -> bool {
        match c.extend_keeping_proportion(dist, end) {
            Ok((out, reached)) => {
                let before = length_of(c);
                let target = before + dist;
                let tol = ACCEPT_TOL * target;
                let after = length_of(&out);
                assert!(
                    (after - target).abs() <= tol,
                    "{what}: measured {after}, asked {target} (reported {reached})"
                );
                assert!(
                    (reached - target).abs() <= tol,
                    "{what}: reported {reached}"
                );
                let width = 0.25 * tol;
                let old = length_enclosure(c, width).unwrap().unwrap();
                let new = length_enclosure(&out, width).unwrap().unwrap();
                assert!(
                    new.0 - old.1 >= dist - tol && new.1 - old.0 <= dist + tol,
                    "{what}: certified growth [{}, {}] for {dist}",
                    new.0 - old.1,
                    new.1 - old.0
                );
                assert_eq!(out.knots(), c.knots(), "{what}");
                true
            }
            Err(SolidError::ExtendRefused {
                refusal: ExtendRefusal::LengthNotMet,
                ..
            }) => false,
            Err(e) => panic!("{what}: {e:?}"),
        }
    }

    /// #16635: a hooked cubic (the polygon turns back at its end) grown by 0.5 and by 3. The
    /// stretch slides the points round the hook, so the length first falls as the stretch grows;
    /// it was returned as Ok at 8.77 and 7.53, short of the 10.35 and 12.85 asked (even shorter
    /// than the curve). Now each reaches the length asked, certified.
    #[test]
    fn a_hooked_cubic_grows_by_the_length_asked() {
        let c = curve(
            &[
                [0.0, 0.0, 0.0],
                [10.0, 0.0, 0.0],
                [10.0, 0.1, 0.0],
                [9.0, 0.1, 0.0],
            ],
            3,
        );
        assert!((length_of(&c) - 9.848_846_342_765_029).abs() < 1e-9);
        for dist in [0.5, 3.0] {
            assert!(
                check(&c, dist, CurveEnd::End, &format!("hook {dist}")),
                "hook {dist}: refused"
            );
        }
    }

    /// A deterministic sweep (fixed seed): straight, curved and nearly degenerate polygons of
    /// degree 1 to 3, grown at either end by lengths from a hundredth of the curve to three
    /// times it. Every Ok has the length asked (certified and measured); every other outcome is
    /// the named refusal. On main a quarter of the Ok results were short or long.
    #[test]
    fn every_extension_reaches_the_length_asked_or_is_refused() {
        let mut state: u64 = 0x0005_eed1_6635_u64;
        let mut next = || {
            state = state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1_442_695_040_888_963_407);
            ((state >> 11) as f64) / ((1u64 << 53) as f64)
        };
        let mut polygons: Vec<(Vec<[f64; 3]>, usize, &str)> = vec![
            (
                vec![[0.0, 0.0, 0.0], [5.0, 0.0, 0.0], [7.0, 0.0, 0.0]],
                2,
                "straight",
            ),
            (
                vec![
                    [0.0, 0.0, 0.0],
                    [2.0, 3.0, 0.0],
                    [6.0, 3.0, 1.0],
                    [9.0, 0.0, 0.0],
                ],
                3,
                "curved",
            ),
            (
                vec![
                    [0.0, 0.0, 0.0],
                    [1.0, 1e-9, 0.0],
                    [2.0, 0.0, 0.0],
                    [3.0, 1e-9, 0.0],
                ],
                3,
                "nearly straight",
            ),
            (
                vec![
                    [0.0, 0.0, 0.0],
                    [1e-6, 0.0, 0.0],
                    [1e-6, 1e-6, 0.0],
                    [5.0, 0.0, 0.0],
                ],
                3,
                "a tiny first leg",
            ),
        ];
        for k in 0..40 {
            let degree = 1 + k % 3;
            let n = degree + 1 + (k % 4);
            let pts: Vec<[f64; 3]> = (0..n)
                .map(|_| {
                    [
                        20.0 * next() - 10.0,
                        20.0 * next() - 10.0,
                        4.0 * next() - 2.0,
                    ]
                })
                .collect();
            polygons.push((pts, degree, "random"));
        }
        let (mut built, mut refused) = (0, 0);
        for (i, (pts, degree, kind)) in polygons.iter().enumerate() {
            let c = curve(pts, *degree);
            let l = length_of(&c);
            for share in [0.01, 0.3, 3.0] {
                for end in [CurveEnd::Start, CurveEnd::End] {
                    let what = format!("{kind} {i} share {share} {end:?}");
                    if check(&c, share * l, end, &what) {
                        built += 1;
                    } else {
                        refused += 1;
                    }
                }
            }
        }
        eprintln!("sweep: built {built}, refused {refused}");
        // every case is decided; most build
        assert!(built > 10 * refused, "built {built}, refused {refused}");
    }

    /// A rational quarter circle grown by 1: certified and the length asked, or refused by name
    /// (a rational piece's upper bound converges only to first order).
    #[test]
    fn a_rational_arc_reaches_the_length_or_is_refused_by_name() {
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let pw = vec![
            CPoint::weighted(Vec3::new(1.0, 0.0, 0.0), 1.0),
            CPoint::weighted(Vec3::new(1.0, 1.0, 0.0), h),
            CPoint::weighted(Vec3::new(0.0, 1.0, 0.0), 1.0),
        ];
        let arc = Curve::new(pw, 2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
        let built = check(&arc, 1.0, CurveEnd::End, "arc");
        eprintln!("arc: built {built}");
    }

    /// The enclosure is the closed form: a straight cubic of length 6 (CVs at x 0, 1, 3, 6) and
    /// a quarter circle of radius 1 lie inside their enclosures.
    #[test]
    fn the_length_enclosure_holds_closed_forms() {
        let c = curve(
            &[
                [0.0, 0.0, 0.0],
                [1.0, 0.0, 0.0],
                [3.0, 0.0, 0.0],
                [6.0, 0.0, 0.0],
            ],
            3,
        );
        let (lo, hi) = length_enclosure(&c, 1e-9).unwrap().unwrap();
        assert!(lo <= 6.0 && 6.0 <= hi && hi - lo <= 1e-9, "{lo} {hi}");
        let h = std::f64::consts::FRAC_1_SQRT_2;
        let pw = vec![
            CPoint::weighted(Vec3::new(1.0, 0.0, 0.0), 1.0),
            CPoint::weighted(Vec3::new(1.0, 1.0, 0.0), h),
            CPoint::weighted(Vec3::new(0.0, 1.0, 0.0), 1.0),
        ];
        let arc = Curve::new(pw, 2, vec![0.0, 0.0, 0.0, 1.0, 1.0, 1.0]).unwrap();
        let quarter = std::f64::consts::FRAC_PI_2;
        let (lo, hi) = length_enclosure(&arc, 1e-3).unwrap().unwrap();
        assert!(lo <= quarter && quarter <= hi, "{lo} {hi}");
    }
}
