//! Filling a closed loop with one height field that meets the faces beside the loop with G1 or
//! G2: the surface of [`crate::fill::fill_loop`] (a bicubic B-spline height field over the loop's
//! plane, so its parameters are plane coordinates and every edge's pcurve is the edge projected
//! onto that plane, exactly), fitted to the faces' derivatives as well as to the loop.
//!
//! The formulation (Piegl & Tiller, *The NURBS Book*, §9.4.1 least squares; do Carmo,
//! *Differential Geometry of Curves and Surfaces*, §3-3 for the second fundamental form of a
//! graph):
//!
//! - **The plane and grid** are [`crate::fill::fill_loop`]'s: the surface is
//!   `S(u, v) = o + u·x + v·y + h(u, v)·n` with `h` a bicubic B-spline on a uniform grid over the
//!   projected loop's box, the heights its unknowns.
//! - **The conditions** at each station (a point of the loop with the face beside it) are linear
//!   in the heights, since every quantity of a graph is a derivative of `h`:
//!   - G0: `h(u, v)` is the station's height.
//!   - G1: the face's unit normal `N` (turned to `n`'s side) is the graph's when
//!     `h_u = −N·x / N·n` and `h_v = −N·y / N·n`.
//!   - G2: with G1, the surfaces share the curve and its tangent plane, so they agree to the
//!     second order exactly when their normal curvatures across the curve agree: along the
//!     direction `w` the face leaves the loop in (`a = w·x`, `b = w·y`),
//!     `(a² h_uu + 2ab h_uv + b² h_vv) · N·n = κ`, `κ` the face's normal curvature along `w`
//!     (its curvature vector `bend` against `N`).
//!
//!   Each row is weighed to a length on the grid's scale (the G1 rows times a cell, the G2 rows
//!   times a cell squared), and the heights minimise their squared misfit plus a small multiple
//!   of the thin-plate energy ([`FAIRNESS`]), which decides what the conditions leave free.
//! - **At a corner** of the loop the corner's own station holds the position only, and the G2
//!   rows fade in over [`FADE_SHARE`] of the run from each corner to the next ([`fade`]).
//! - **Grids** of 4 spans across the box's longer side, doubling to 128, are tried in turn, with
//!   denser sampling when a grid's cells are finer than twice the stations' step. The fill is the
//!   first whose surface is within `tol` of every station, whose normal is within `angle_tol` of
//!   the face's at every G1/G2 station, and whose normal curvature across the loop is within
//!   `curvature_tol` of the face's at every G2 station clear of the corners' fades. Otherwise
//!   "not supported yet", with how close the best grid came.
//! - **Certified path** ([`fill_matched_certified`]), only for a loop the grids above refuse and
//!   whose caller bounds every side ([`SideBounds`]): the rows near a corner ask for a curvature
//!   passing from the corner's own ([`corner_hessian`]) to the face's ([`ramp`]), the ladder ends
//!   at [`MAX_MATCHED_SPANS`], and a grid is accepted only if, besides the station checks, it
//!   stays within `tol` of every planar neighbour's plane by its Bézier hulls ([`past_planes`])
//!   and the gap, angle and curvature are bounded within their tolerances between the stations
//!   ([`certify`]).
//!
//! A loop that is not the boundary of a height field over its plane, or a face beside it that
//! stands along the plane's normal, is refused, never filled with wrong geometry. Input is
//! hostile: sizes are capped and every evaluation checked; nothing here panics.

use crate::bspline::{HPoint, SplineError, SplineResult, basis_bezier, bezier_segment, invalid, segment, subdivide};
use crate::fill::{Band, Grid, LoopFill, LoopFrame, MAX_LOOP_POINTS, SMOOTHNESS, check_projectable, inside_loop, loop_frame, segment_meets_rect};
use crate::fill_general::Station;
use crate::prelude::*;
use solidcore_record::Vec3;

/// Most spans across the box's longer side: the last grid, √2 past the doubling's 128, for a
/// face's side short against the loop (0.9 of 5 for r3 rounds set back 3.5595), whose fades 128
/// spans cross in 3.5 cells. Its band takes 150 MB and four times the 128 grid's solve.
pub const MAX_MATCHED_SPANS: usize = 181;
/// The last grid of the uncertified path.
const PLAIN_SPANS: usize = 128;
/// Most stations the certified path samples: twice [`MAX_LOOP_POINTS`] (the other path's cap),
/// as the last grid's cells, finer than the 128 grid's, need the stations twice as close.
pub const MAX_MATCHED_POINTS: usize = 2 * MAX_LOOP_POINTS;
/// Fewest spans (the first grid tried).
const MIN_SPANS: usize = 4;
/// The share of the run from a corner to the next over which the G2 rows fade in ([`fade`]) and
/// [`ramp`] from the corner's curvature to the face's.
pub const FADE_SHARE: f64 = 0.15;
/// The thin-plate energy's weight per fitted station: a thousand times
/// [`crate::fill::fill_loop`]'s. Its fill only passes through points, which leave the inside to
/// the energy alone; here the slope and curvature rows also act a little way inside the loop,
/// and with the weaker energy they pull the surface up past a plane it is G2 to, a millimetre or
/// two in from the plane's curve (by 0.06 at a box corner's r5 rounds set back 8). This weight
/// keeps it under the plane but where it passes from a round's curvature to the plane's, and
/// costs the loop little (2e-5 mm, 0.025° and 4e-4 per mm there).
const FAIRNESS: f64 = 1e3 * SMOOTHNESS;
/// A face beside the loop whose normal is closer than this cosine to square with the plane's
/// stands along the plane's normal: the height field cannot be tangent to it.
const MIN_NORMAL_COS: f64 = 0.05;

/// What [`fill_matched`] is asked for.
#[derive(Clone, Copy, Debug)]
pub struct MatchedSpec {
    /// How far the surface may be from the loop.
    pub tol: f64,
    /// How far (radians) its normal may turn from a G1/G2 neighbour's.
    pub angle_tol: f64,
    /// How far (1/length) its normal curvature across the loop may be from a G2 neighbour's,
    /// clear of the corners' fades.
    pub curvature_tol: f64,
}

/// The fill and what it achieved.
#[derive(Clone, Debug, PartialEq)]
pub struct MatchedFill {
    /// The height field, its plane and its deviation from the loop (no crown).
    pub fill: LoopFill,
    /// The largest angle (radians) between the surface's normal and a neighbour's at the G1/G2
    /// stations (both fields alike: every such station is held to the angle tolerance).
    pub normal_deviation: f64,
    pub normal_deviation_all: f64,
    /// The largest difference (1/length) between the surface's normal curvature across the
    /// loop and the neighbour's, at the G2 stations clear of the corners' fades, and at any G2
    /// station.
    pub curvature_mismatch: f64,
    pub curvature_mismatch_all: f64,
    /// Spans of the grid each way, and stations sampled.
    pub spans: [usize; 2],
    pub stations: usize,
}

/// Bounds on one side of the loop over its station parameter `t` (0 to 1), from the caller, who
/// knows the edge and the face beside it: they certify a fill between the stations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SideBounds {
    /// `|p'(t)|` and `|p''(t)|` of the loop's point.
    pub speed: f64,
    pub accel: f64,
    /// `|N''(t)|` of the face's unit normal along the side.
    pub normal_accel: f64,
    /// `|w'(t)|` of the leave direction.
    pub leave_rate: f64,
    /// `|κ'(t)|` of the face's normal curvature along the leave direction.
    pub curvature_rate: f64,
}

/// Fill the closed loop of `sides` sides with one height field meeting each station's face as
/// its `across` asks (see the module documentation). `stations(k, ts)` samples side `k` at the
/// parameters `ts` (0 at its start, which is where side `k − 1` ends, towards 1 at its end); a
/// station at `t = 0` marked `corner` is a corner of the loop. Accepted on the station checks
/// alone, with each face's own curvature asked near the corners and grids up to 128 spans.
pub fn fill_matched(
    sides: usize,
    stations: &mut dyn FnMut(usize, &[f64]) -> Result<Vec<Station>, String>,
    spec: &MatchedSpec,
) -> SplineResult<MatchedFill> {
    ladder(sides, stations, spec, None)
}

/// [`fill_matched`], and where it refuses and `bounds` holds one [`SideBounds`] per side, the
/// certified path: the corners' own curvature asked near them ([`corner_hessian`], [`ramp`]),
/// grids up to [`MAX_MATCHED_SPANS`] over up to [`MAX_MATCHED_POINTS`] stations, and a grid
/// accepted only when, besides the station checks, its Bézier hulls stay within `tol` of every
/// planar neighbour ([`past_planes`]) and the gap to the loop, the angle and the curvature are
/// bounded within their tolerances between the stations ([`certify`]). If that path refuses too,
/// the refusal is [`fill_matched`]'s.
pub fn fill_matched_certified(
    sides: usize,
    stations: &mut dyn FnMut(usize, &[f64]) -> Result<Vec<Station>, String>,
    spec: &MatchedSpec,
    bounds: &[SideBounds],
) -> SplineResult<MatchedFill> {
    match ladder(sides, stations, spec, None) {
        Err(SplineError::Unsupported(why)) if bounds.len() == sides => {
            ladder(sides, stations, spec, Some(bounds)).map_err(|_| SplineError::Unsupported(why))
        }
        other => other,
    }
}

/// The grid ladder, uncertified (`bounds` `None`) or certified.
fn ladder(
    sides: usize,
    stations: &mut dyn FnMut(usize, &[f64]) -> Result<Vec<Station>, String>,
    spec: &MatchedSpec,
    bounds: Option<&[SideBounds]>,
) -> SplineResult<MatchedFill> {
    if sides == 0 || sides.saturating_mul(16) > MAX_LOOP_POINTS / 2 {
        return Err(SplineError::TooLarge(format!("a loop of {sides} sides (at most {})", MAX_LOOP_POINTS / 32)));
    }
    let positive = |x: f64| x.is_finite() && x > 0.0;
    if !(positive(spec.tol) && positive(spec.angle_tol) && positive(spec.curvature_tol)) {
        return invalid("the fill tolerances must be positive");
    }
    let mut sample = |ts: &[f64]| -> SplineResult<Vec<Vec<Station>>> {
        (0..sides)
            .map(|k| {
                let got = stations(k, ts).map_err(SplineError::Invalid)?;
                if got.iter().any(|s| s.clash) {
                    return Err(SplineError::Unsupported("a matched fill at a corner whose faces' tangent planes disagree".into()));
                }
                if got.len() == ts.len() && got.iter().all(finite_station) {
                    Ok(got)
                } else {
                    Err(SplineError::Invalid(format!("side {k} has no finite stations")))
                }
            })
            .collect()
    };
    // Each side's end, where the next one starts: at a corner, the face before it.
    let ends: Vec<Station> = sample(&[1.0])?
        .into_iter()
        .map(|side| side.into_iter().next())
        .collect::<Option<_>>()
        .ok_or_else(|| SplineError::Invalid("a side has no end".into()))?;
    let mut per = 16usize;
    let mut fit = sample(&(0..per).map(|i| i as f64 / per as f64).collect::<Vec<_>>())?;
    let mut best = Shortfall::default();
    let mut level = MIN_SPANS;
    loop {
        let check = sample(&(0..per).map(|i| (i as f64 + 0.5) / per as f64).collect::<Vec<_>>())?;
        // The loop in order: fitted and checked stations alternating, side by side.
        let mut all: Vec<(Station, bool)> = Vec::with_capacity(2 * per * sides);
        for (f, c) in fit.iter().zip(&check) {
            for (a, b) in f.iter().zip(c) {
                all.push((*a, true));
                all.push((*b, false));
            }
        }
        match fit_round(&all, &ends, spec, level, bounds)? {
            Round::Done(fill) => return Ok(*fill),
            Round::Denser(s, at) => {
                best = best.min(s);
                level = at;
            }
            Round::Exhausted(s) => {
                best = best.min(s);
                break;
            }
        }
        if all.len() * 2 > if bounds.is_some() { MAX_MATCHED_POINTS } else { MAX_LOOP_POINTS } {
            break;
        }
        fit = fit.iter().zip(&check).map(|(f, c)| f.iter().zip(c).flat_map(|(a, b)| [*a, *b]).collect()).collect();
        per *= 2;
    }
    Err(SplineError::Unsupported(format!(
        "filling this loop with one surface meeting its neighbours: the closest fit stays {:e} off the loop (tolerance {:e}), turns {:.4}° from \
         its neighbours (tolerance {:.4}°) and misses their curvature by {:e} (tolerance {:e})",
        best.deviation,
        spec.tol,
        best.angle.to_degrees(),
        spec.angle_tol.to_degrees(),
        best.curvature,
        spec.curvature_tol
    )))
}

fn finite_station(s: &Station) -> bool {
    s.p.is_finite() && s.across.is_none_or(|a| a.leave.is_finite() && a.normal.is_finite() && a.bend.is_none_or(|b| b.is_finite()))
}

/// How close a grid came.
#[derive(Clone, Copy, Debug)]
struct Shortfall {
    deviation: f64,
    angle: f64,
    curvature: f64,
}

impl Default for Shortfall {
    fn default() -> Shortfall {
        Shortfall { deviation: f64::INFINITY, angle: f64::INFINITY, curvature: f64::INFINITY }
    }
}

impl Shortfall {
    /// The closer of two, by how many times its tolerance each misses.
    fn min(self, other: Shortfall) -> Shortfall {
        let worst = |s: &Shortfall| s.deviation + s.angle + s.curvature;
        if worst(&other) < worst(&self) { other } else { self }
    }
}

enum Round {
    Done(Box<MatchedFill>),
    /// Denser sampling is needed for the grid of this level (the coarser ones fell short).
    Denser(Shortfall, usize),
    Exhausted(Shortfall),
}

/// A station as the fit takes it: its place over the plane, its height, and what the face beside
/// it asks there, in the plane's terms.
struct Row {
    q: [f64; 2],
    h: f64,
    fitted: bool,
    /// G1: the height field's gradient there, and the face's unit normal on `n`'s side.
    gradient: Option<([f64; 2], Vec3)>,
    /// G2: the leave direction's plane components `(a, b)`, the second derivative of `h` asked
    /// along them (`κ / N·n`, or within a corner's fade passing to it from the corner's), and `κ`
    /// itself.
    curvature: Option<([f64; 2], f64, f64)>,
    /// How near the station is to a corner: its distance along the loop to the nearer of the two
    /// around it, as a share of the run between them (0 at a corner, ½ midway; 1 with no corner).
    to_corner: f64,
    corner: bool,
}

/// The stations in the plane `f`'s terms, with their distances to the corners along the loop.
/// `ends` are the sides' ends (each where the next side starts), `all` the sides' stations in
/// order, as many on each.
fn rows(all: &[(Station, bool)], ends: &[Station], f: &LoopFrame, corner_curvature: Option<f64>) -> SplineResult<Vec<Row>> {
    let m = all.len();
    let sides = ends.len();
    if sides == 0 || !m.is_multiple_of(sides) {
        return invalid("stations");
    }
    let per_side = m / sides;
    // Arc length along the loop (chords), and where its corners are.
    let mut s = vec![0.0f64; m + 1];
    for k in 0..m {
        let (Some((a, _)), Some((b, _))) = (all.get(k), all.get((k + 1) % m)) else { return invalid("stations") };
        let next = s.get(k).copied().unwrap_or(0.0) + a.p.dist(b.p);
        if let Some(slot) = s.get_mut(k + 1) {
            *slot = next;
        }
    }
    let total = s.get(m).copied().unwrap_or(0.0);
    if !(total > 0.0 && total.is_finite()) {
        return invalid("the loop has no length");
    }
    let at = |k: usize| s.get(k).copied().unwrap_or(0.0);
    let at_corner: Vec<usize> = (0..m).filter(|&k| all.get(k).is_some_and(|(st, _)| st.corner)).collect();
    let corners: Vec<f64> = at_corner.iter().map(|&k| at(k)).collect();
    // Each corner's own curvature: from the face before it (the previous side's end) and the face
    // after it (its station's), where both give it.
    let mut hessians: Vec<Option<[f64; 3]>> = Vec::with_capacity(at_corner.len());
    for &k in &at_corner {
        let before = ends.get((k / per_side + sides - 1) % sides);
        let (Some((after, _)), Some(before)) = (all.get(k), before) else { return invalid("stations") };
        hessians.push(match corner_curvature {
            Some(tol) => corner_hessian(before, after, f, tol)?,
            None => None,
        });
    }
    // The share of the run between the two corners around `x` from the nearer one, and that one.
    let nearest = |list: &[f64], x: f64| -> (f64, usize) {
        let (mut back, mut behind) = (f64::INFINITY, 0usize);
        let (mut ahead, mut before) = (total, 0usize);
        for (i, c) in list.iter().enumerate() {
            let b = (x - c).rem_euclid(total);
            if b < back {
                (back, behind) = (b, i);
            }
            let a = (c - x).rem_euclid(total);
            if a > 0.0 && a < ahead {
                (ahead, before) = (a, i);
            }
        }
        if list.is_empty() {
            return (1.0, 0);
        }
        let run = back + ahead;
        (if run > 0.0 { back.min(ahead) / run } else { 0.0 }, if back <= ahead { behind } else { before })
    };
    let mut out = Vec::with_capacity(m);
    for (k, (st, fitted)) in all.iter().enumerate() {
        let (to_corner, near) = nearest(&corners, at(k));
        let own = hessians.get(near).copied().flatten();
        let d = st.p - f.o;
        let q = [d.dot(f.x), d.dot(f.y)];
        let (gradient, bend) = face_terms(st, f)?;
        let curvature = bend.map(|(ab, kappa, c)| {
            let second = match own {
                Some([huu, huv, hvv]) => {
                    let blend = ramp(to_corner);
                    let from = ab[0] * ab[0] * huu + 2.0 * ab[0] * ab[1] * huv + ab[1] * ab[1] * hvv;
                    (1.0 - blend) * from + blend * (kappa / c)
                }
                None => kappa / c,
            };
            (ab, second, kappa)
        });
        out.push(Row { q, h: d.dot(f.n), fitted: *fitted, gradient, curvature, to_corner, corner: st.corner });
    }
    Ok(out)
}

type FaceTerms = (Option<([f64; 2], Vec3)>, Option<([f64; 2], f64, f64)>);

/// What the face beside station `st` asks, in the plane `f`'s terms: the gradient the height
/// field needs for G1 and the face's unit normal on `n`'s side, and for G2 the leave direction's
/// plane components, the face's normal curvature along it and `N·n`.
fn face_terms(st: &Station, f: &LoopFrame) -> SplineResult<FaceTerms> {
    let Some(a) = st.across else { return Ok((None, None)) };
    let normal = a.normal.normalized().ok_or_else(|| SplineError::Invalid("a neighbour's normal has no direction".into()))?;
    let nn = if normal.dot(f.n) >= 0.0 { normal } else { normal * -1.0 };
    let c = nn.dot(f.n);
    if !(c > MIN_NORMAL_COS) {
        return Err(SplineError::Unsupported("filling a loop beside a face that stands along the fill's normal".into()));
    }
    let bend = a.bend.map(|b| ([a.leave.dot(f.x), a.leave.dot(f.y)], b.dot(nn), c));
    Ok((Some(([-nn.dot(f.x) / c, -nn.dot(f.y) / c], nn)), bend))
}

/// The second derivatives `[h_uu, h_uv, h_vv]` of the height field over the plane `f` at a
/// corner of the loop between the face `before` it and the face `after` (module docs): a
/// surface G1 to a face along a curve has the face's `II(t, t)` and `II(t, w)` along the curve's
/// tangent `t` (`w` the leave direction; the jet's `t²` coefficient and half its `t·w` one), and
/// with the graph's `II(X, Y) = (N·n) X_pᵀ H Y_p` (`X_p` the plane components) each is linear in
/// `H`. Two faces give four conditions on three unknowns: their least-squares solution. `None`
/// where a face gives no jet, where the two curves leave the corner so nearly the same way that
/// the conditions leave `H` undecided (the normal equations' determinant under [`UNDECIDED`] of
/// its largest, `(trace / 3)³`), or where the faces disagree: a condition the solution misses by
/// more than `curvature_tol` (two rounds of different radii meeting G1 at a vertex where the loop
/// barely turns), as no surface holds both.
fn corner_hessian(before: &Station, after: &Station, f: &LoopFrame, curvature_tol: f64) -> SplineResult<Option<[f64; 3]>> {
    // The four conditions `row · H = rhs`, each with the `N·n` that turns its miss back into `II`.
    let mut conditions: Vec<([f64; 3], f64, f64)> = Vec::with_capacity(4);
    for st in [before, after] {
        let Some(a) = st.across else { return Ok(None) };
        let Some(jet) = a.jet else { return Ok(None) };
        let Some(&[ctt, ctw, _]) = jet.forms.first().and_then(|form| form.get(..3)) else { return Ok(None) };
        let normal = a.normal.normalized().ok_or_else(|| SplineError::Invalid("a neighbour's normal has no direction".into()))?;
        // The jet is the face's height along `normal`; the rows take it along `n`'s side.
        let side = if normal.dot(f.n) >= 0.0 { 1.0 } else { -1.0 };
        let c = normal.dot(f.n) * side;
        if !(c > MIN_NORMAL_COS) {
            return Err(SplineError::Unsupported("filling a loop beside a face that stands along the fill's normal".into()));
        }
        let w = a.leave;
        let tp = [normal.cross(w).dot(f.x), normal.cross(w).dot(f.y)];
        for (q, value) in [(tp, ctt), ([w.dot(f.x), w.dot(f.y)], 0.5 * ctw)] {
            conditions.push(([tp[0] * q[0], tp[0] * q[1] + tp[1] * q[0], tp[1] * q[1]], value * side / c, c));
        }
    }
    let mut m = [[0.0f64; 3]; 3];
    let mut r = [0.0f64; 3];
    for (row, rhs, _) in &conditions {
        for (mi, ri) in m.iter_mut().zip(row) {
            for (mij, rj) in mi.iter_mut().zip(row) {
                *mij += ri * rj;
            }
        }
        for (x, ri) in r.iter_mut().zip(row) {
            *x += ri * rhs;
        }
    }
    let det = |a: &[[f64; 3]; 3]| {
        let [[a00, a01, a02], [a10, a11, a12], [a20, a21, a22]] = *a;
        a00 * (a11 * a22 - a12 * a21) - a01 * (a10 * a22 - a12 * a20) + a02 * (a10 * a21 - a11 * a20)
    };
    let d = det(&m);
    let scale = (m[0][0] + m[1][1] + m[2][2]) / 3.0;
    if !(d > UNDECIDED * scale * scale * scale) {
        return Ok(None);
    }
    let mut out = [0.0; 3];
    for (col, o) in out.iter_mut().enumerate() {
        let mut a = m;
        for (row, x) in a.iter_mut().zip(r) {
            if let Some(slot) = row.get_mut(col) {
                *slot = x;
            }
        }
        *o = det(&a) / d;
    }
    let held = conditions.iter().all(|(row, rhs, c)| (c * (row.iter().zip(out).map(|(x, h)| x * h).sum::<f64>() - rhs)).abs() <= curvature_tol);
    Ok((held && out.iter().all(|x| x.is_finite())).then_some(out))
}

/// How small the determinant of [`corner_hessian`]'s normal equations may be against its largest
/// for a positive semidefinite matrix of that trace, `(trace / 3)³`, before the conditions count
/// as undecided: two curves a degree apart give about 3e-4 (it goes as the sine squared of the
/// angle between them), so only curves that leave the same way to rounding fall under it.
const UNDECIDED: f64 = 1e-12;

/// The C2 smootherstep `6t⁵ − 15t⁴ + 10t³` of `share / FADE_SHARE` (1 from [`FADE_SHARE`] on):
/// how far the curvature a G2 row near a corner with a curvature of its own asks has passed from
/// the corner's to its face's. Its first and second derivatives vanish at both ends, so the
/// curvature asked along the loop has a continuous slope and bend where the ramp meets the
/// corner's and the face's.
fn ramp(share: f64) -> f64 {
    let t = (share / FADE_SHARE).clamp(0.0, 1.0);
    t * t * t * (10.0 + t * (6.0 * t - 15.0))
}

/// The smoothstep `3t² − 2t³` of `share / FADE_SHARE` (1 from [`FADE_SHARE`] on).
fn fade(share: f64) -> f64 {
    let t = (share / FADE_SHARE).clamp(0.0, 1.0);
    t * t * (3.0 - 2.0 * t)
}

/// A planar neighbour's plane: a point on it and its unit normal, turned away from the loop.
struct Plane {
    p: Vec3,
    n: Vec3,
}

/// The planes of the loop's planar neighbours: each side whose stations all carry one unit
/// normal (to [`FLAT`]) and no curvature across (`bend` and every jet coefficient, to [`FLAT`] at
/// the loop's size; a side with no `bend` counts as flat, so it is checked rather than skipped).
/// Such a side whose stations leave its plane (by more than [`FLAT`] of the loop's size) is
/// refused. Each plane is turned to the side the whole loop lies under, to `tol`; a plane the
/// loop reaches past both ways bounds nothing and is left out.
fn planes(all: &[(Station, bool)], ends: &[Station], tol: f64) -> SplineResult<Vec<Plane>> {
    let per_side = all.len() / ends.len().max(1);
    let first = all.first().map_or(Vec3::ZERO, |(s, _)| s.p);
    let size = all.iter().map(|(s, _)| s.p.dist(first)).fold(0.0, f64::max);
    let mut out = Vec::new();
    for (k, end) in ends.iter().enumerate() {
        let side: Vec<&Station> = all.get(k * per_side..(k + 1) * per_side).unwrap_or(&[]).iter().map(|(s, _)| s).chain([end]).collect();
        let Some((s0, n0)) = side.first().and_then(|s| Some((*s, s.across?.normal.normalized()?))) else { continue };
        let flat = |s: &&Station| {
            s.across.is_some_and(|a| {
                a.normal.normalized().is_some_and(|n| n.cross(n0).len() <= FLAT)
                    && a.bend.is_none_or(|b| b.len() * size <= FLAT)
                    && a.jet.is_none_or(|j| j.forms.iter().zip(1..).all(|(form, e)| form.iter().all(|c| c.abs() * size.powi(e) <= FLAT)))
            })
        };
        if !side.iter().all(flat) {
            continue;
        }
        if !side.iter().all(|s| (s.p - s0.p).dot(n0).abs() <= FLAT * size) {
            return Err(SplineError::Unsupported("a matched fill beside a flat face whose edge leaves its plane".into()));
        }
        let reach = |n: Vec3| all.iter().map(|(s, _)| (s.p - s0.p).dot(n)).fold(f64::NEG_INFINITY, f64::max);
        if let Some(n) = [n0, n0 * -1.0].into_iter().find(|n| reach(*n) <= tol) {
            out.push(Plane { p: s0.p, n });
        }
    }
    Ok(out)
}

/// How nearly flat (relative to the loop's size) a side's face must be to count as a plane.
const FLAT: f64 = 1e-12;
/// Most patches [`past_planes`] splits per plane before it counts the patch as reaching past.
const MAX_SPLITS: usize = 1 << 16;

/// An upper bound on how far the height field `heights` on `grid` over `f` reaches past any of
/// `planes` within the loop `poly` (the stations in the plane's terms, in order), from its Bézier
/// hulls (NURBS Book §5.4: each span is a bicubic Bézier patch, inside the hull of its control
/// points, and the distance to a plane is affine, so its 16 values at the control points bound
/// it there). A patch whose hull is over `tol` is split in four (de Casteljau, exact but for
/// rounding) while it meets the loop: a side of the stations' polygon `poly` crosses it grown by
/// twice that side's sagitta (the circle through its ends' neighbours; none at a `corner`), or
/// its middle is inside.
/// Rounding is counted (`γ` of each coefficient's terms, and of each split). Past `tol` once a
/// patch's corner (a point of the surface) inside the loop is, or after [`MAX_SPLITS`] splits.
fn past_planes(grid: &Grid, heights: &[f64], f: &LoopFrame, planes: &[Plane], poly: &[[f64; 2]], corner: &[bool], tol: f64) -> SplineResult<f64> {
    let [cu, cv] = grid.count;
    let spans = [cu - 3, cv - 3];
    let (Some(&lo0), Some(&lo1)) = (grid.knots[0].first(), grid.knots[1].first()) else { return invalid("grid") };
    let (lo, cell) = ([lo0, lo1], [grid.cell(0), grid.cell(1)]);
    let bezier = span_bezier(grid)?;
    // The loop's sides (chords between stations), each with how far the loop can bow off it
    // (twice the sagitta `c²κ/8` of the circle through its end's neighbours, at an end not a
    // corner), by the spans their grown boxes reach, for the meet test.
    let m = poly.len();
    let at = |k: usize| poly.get(k % m).copied().unwrap_or([0.0; 2]);
    let gap = |p: [f64; 2], q: [f64; 2]| ((q[0] - p[0]) * (q[0] - p[0]) + (q[1] - p[1]) * (q[1] - p[1])).sqrt();
    let bend = |k: usize| {
        let (a, b, c) = (at(k + m - 1), at(k), at(k + 1));
        let cross = (b[0] - a[0]) * (c[1] - b[1]) - (b[1] - a[1]) * (c[0] - b[0]);
        let kappa = 2.0 * cross.abs() / (gap(a, b) * gap(b, c) * gap(a, c));
        if corner.get(k % m).copied().unwrap_or(true) || !kappa.is_finite() { 0.0 } else { kappa }
    };
    let mut near: Vec<Vec<([f64; 2], [f64; 2], f64)>> = vec![Vec::new(); spans[0] * spans[1]];
    for k in 0..m {
        let (p, q) = (at(k), at(k + 1));
        let bow = gap(p, q).powi(2) * bend(k).max(bend(k + 1)) / 4.0;
        let range = |d: usize| {
            let at = |x: f64| (((x - lo[d]) / cell[d]).floor().max(0.0) as usize).min(spans[d] - 1);
            at(p[d].min(q[d]) - bow)..=at(p[d].max(q[d]) + bow)
        };
        for j in range(1) {
            for i in range(0) {
                if let Some(list) = near.get_mut(j * spans[0] + i) {
                    list.push((p, q, bow));
                }
            }
        }
    }
    let gamma = 64.0 * f64::EPSILON;
    let mut worst = f64::NEG_INFINITY;
    for plane in planes {
        let (ax, ay, an, a0) = (f.x.dot(plane.n), f.y.dot(plane.n), f.n.dot(plane.n), (f.o - plane.p).dot(plane.n));
        let offset = (f.o - plane.p).len();
        let mut todo: Vec<([[f64; 4]; 4], [f64; 2], [f64; 2], f64, usize)> = Vec::new();
        let mut splits = 0usize;
        for sj in 0..spans[1] {
            for si in 0..spans[0] {
                let (u0, v0) = (lo[0] + cell[0] * si as f64, lo[1] + cell[1] * sj as f64);
                let (hs, mags) = cell_net(grid, heights, &bezier, si, sj)?;
                // net[l][k]: the distance's Bézier coefficient k along u, l along v.
                let mut net = [[0.0; 4]; 4];
                let mut slack = 0.0f64;
                for (l, (row, (hrow, mrow))) in net.iter_mut().zip(hs.iter().zip(&mags)).enumerate() {
                    for (k, (slot, (&h, &mag))) in row.iter_mut().zip(hrow.iter().zip(mrow)).enumerate() {
                        let (u, v) = (u0 + cell[0] * k as f64 / 3.0, v0 + cell[1] * l as f64 / 3.0);
                        *slot = a0 + u * ax + v * ay + h * an;
                        // The terms' rounding, and that of `ax`, `ay`, `an`, `a0` (dot products of unit vectors).
                        slack = slack.max(gamma * (a0.abs() + (u * ax).abs() + (v * ay).abs() + mag * an.abs() + u.abs() + v.abs() + mag + offset));
                    }
                }
                todo.push((net, [u0, v0], [u0 + cell[0], v0 + cell[1]], slack, sj * spans[0] + si));
            }
        }
        while let Some((net, a, b, slack, at)) = todo.pop() {
            let hull = net.iter().flatten().fold(f64::NEG_INFINITY, |m, &x| if x > m || x.is_nan() { x } else { m }) + slack;
            if !hull.is_finite() {
                return Ok(f64::INFINITY);
            }
            if hull <= tol {
                worst = worst.max(hull);
                continue;
            }
            let list = near.get(at).map_or(&[][..], Vec::as_slice);
            let middle = [0.5 * (a[0] + b[0]), 0.5 * (a[1] + b[1])];
            let meets =
                list.iter().any(|&(p, q, w)| segment_meets_rect(p, q, [a[0] - w, a[1] - w], [b[0] + w, b[1] + w])) || inside_loop(middle, poly);
            if !meets {
                continue;
            }
            let [[c00, _, _, c03], _, _, [c30, _, _, c33]] = net;
            for (value, q) in [(c00, a), (c03, [b[0], a[1]]), (c30, [a[0], b[1]]), (c33, b)] {
                if value - slack > tol && inside_loop(q, poly) {
                    return Ok(value);
                }
            }
            splits += 1;
            if splits > MAX_SPLITS {
                return Ok(hull);
            }
            let grow = slack + 8.0 * f64::EPSILON * net.iter().flatten().fold(0.0, |m: f64, x| m.max(x.abs()));
            let (left, right) = split_rows(net);
            let um = middle[0];
            for (half, ua, ub) in [(left, a[0], um), (right, um, b[0])] {
                let (low, high) = split_rows(transpose(half));
                todo.push((transpose(low), [ua, a[1]], [ub, middle[1]], grow, at));
                todo.push((transpose(high), [ua, middle[1]], [ub, b[1]], grow, at));
            }
        }
    }
    Ok(worst)
}

/// Each span's four basis functions in Bernstein form, per direction (`[span][basis][k]`).
fn span_bezier(grid: &Grid) -> SplineResult<[Vec<[[f64; 4]; 4]>; 2]> {
    let mut bezier: [Vec<[[f64; 4]; 4]>; 2] = [Vec::new(), Vec::new()];
    for (out, (kn, count)) in bezier.iter_mut().zip(grid.knots.iter().zip(grid.count)) {
        for s in 0..count.saturating_sub(3) {
            let (Some(&a), Some(&b)) = (kn.get(s + 3), kn.get(s + 4)) else { return invalid("grid") };
            let mut m = [[0.0; 4]; 4];
            for (i, row) in m.iter_mut().enumerate() {
                let support = kn.get(s + i..s + i + 5).ok_or_else(|| SplineError::Invalid("grid".into()))?;
                for (slot, c) in row.iter_mut().zip(basis_bezier(support, a, b)?) {
                    *slot = c;
                }
            }
            out.push(m);
        }
    }
    Ok(bezier)
}

/// The height field's Bézier net on span `(si, sj)` (`[l][k]`, k along u), and each
/// coefficient's sum of absolute terms (for its rounding).
fn cell_net(grid: &Grid, heights: &[f64], bezier: &[Vec<[[f64; 4]; 4]>; 2], si: usize, sj: usize) -> SplineResult<([[f64; 4]; 4], [[f64; 4]; 4])> {
    let (Some(mu), Some(mv)) = (bezier[0].get(si), bezier[1].get(sj)) else { return invalid("grid") };
    let (mut net, mut mags) = ([[0.0; 4]; 4], [[0.0; 4]; 4]);
    for (l, (row, mrow)) in net.iter_mut().zip(mags.iter_mut()).enumerate() {
        for (k, (slot, mag)) in row.iter_mut().zip(mrow.iter_mut()).enumerate() {
            for (b, wv) in mv.iter().enumerate() {
                for (a, wu) in mu.iter().enumerate() {
                    let c = wu.get(k).copied().unwrap_or(0.0) * wv.get(l).copied().unwrap_or(0.0);
                    let x = heights.get((sj + b) * grid.count[0] + si + a).copied().unwrap_or(f64::NAN);
                    *slot += c * x;
                    *mag += (c * x).abs();
                }
            }
        }
    }
    Ok((net, mags))
}

/// Bounds over the part `[u0, u1] × [v0, v1]` (shares of the span, 0 to 1) of the Bézier net
/// `net` of a span `du × dv` of `|∇h|`, of the Hessian's norm and of the third derivatives'
/// (`|h_u| + |h_v|`, `|h_uu| + 2|h_uv| + |h_vv|`, `|h_uuu| + 3|h_uuv| + 3|h_uvv| + |h_vvv|`): the
/// part's net (de Casteljau), then the Bernstein coefficients of each partial (its hodographs).
fn net_bounds(net: &[[f64; 4]; 4], [du, dv]: [f64; 2], [u0, u1]: [f64; 2], [v0, v1]: [f64; 2]) -> SplineResult<[f64; 3]> {
    let piece = |row: &[f64; 4], a: f64, b: f64| -> SplineResult<[f64; 4]> {
        let h: Vec<HPoint> = row.iter().map(|&x| HPoint::weighted(Vec3::new(x, 0.0, 0.0), 1.0)).collect();
        let mut out = [0.0; 4];
        out.iter_mut().zip(bezier_segment(&h, a, b)?).for_each(|(o, p)| *o = p.x);
        Ok(out)
    };
    let mut part = [[0.0; 4]; 4];
    for (o, row) in part.iter_mut().zip(net) {
        *o = piece(row, u0, u1)?;
    }
    let mut cols = transpose(part);
    for c in cols.iter_mut() {
        *c = piece(c, v0, v1)?;
    }
    let part = transpose(cols);
    let (du, dv) = (du * (u1 - u0), dv * (v1 - v0));
    // The largest |Δ^i_u Δ^j_v| of the part's net, times the partial's factor.
    let top = |i: usize, j: usize| {
        let diff = |row: &[f64], n: usize| {
            (0..n).fold(row.to_vec(), |r, _| {
                r.windows(2).map(|w| w.get(1).copied().unwrap_or(0.0) - w.first().copied().unwrap_or(0.0)).collect::<Vec<f64>>()
            })
        };
        let rows: Vec<Vec<f64>> = part.iter().map(|r| diff(r, i)).collect();
        let cols = (0..4 - i).map(|k| diff(&rows.iter().map(|r| r.get(k).copied().unwrap_or(0.0)).collect::<Vec<_>>(), j));
        let falling = |n: usize| (0..n).fold(1.0, |f, m| f * (3 - m) as f64);
        cols.flatten().fold(0.0f64, |m, x| m.max(x.abs())) * falling(i) * falling(j) / (du.powi(i as i32) * dv.powi(j as i32))
    };
    let gradient = top(1, 0) + top(0, 1);
    let hessian = top(2, 0) + 2.0 * top(1, 1) + top(0, 2);
    let third = top(3, 0) + 3.0 * top(2, 1) + 3.0 * top(1, 2) + top(0, 3);
    let inflate = 1.0 + 1e-9;
    let out = [gradient * inflate, hessian * inflate, third * inflate];
    if out.iter().all(|x| x.is_finite()) { Ok(out) } else { invalid("the fit is not finite") }
}

/// Upper bounds on the gap between the height field and the loop, on the angle between their
/// normals, and on the curvature miss clear of the corners' fades, between the stations, from
/// the side `bounds` and the span bounds ([`net_bounds`]) over where the loop can be between two
/// stations (within `speed·δ/2` of one, `δ` their parameter step). The gap `f = h(q) − H` and the
/// normals' difference `D` are `W^{2,∞}` along a side, so each is at most the larger of its
/// values at the two stations plus `M₂ δ²/8`, with `M₂` a bound on its second derivative:
/// `|f''| ≤ ‖∇²h‖ S² + (|∇h| + 1) A`; for the graph's normal `N = m/|m|`, `m = (−h_u, −h_v, 1)`,
/// `|N''| ≤ 2|m''| + 3|m'|²` with `|m'| ≤ ‖∇²h‖ S`, `|m''| ≤ ‖∇³h‖ S² + ‖∇²h‖ A`, plus the face's
/// `|N''|`. The angle is `2 asin(|D|/2)`. The curvature `k = aᵀ∇²h a / (W B)` (`W = |m|`,
/// `B = |a|² + (a·∇h)²`) is only Lipschitz (the third derivatives jump at knots): its miss is at
/// most the larger station value plus `L δ/2` (`L δ` from the one station clear of a fade), with
/// `L ≥ |k'| + |κ'|` from the quotient rule. Rounding: every span bound is raised by 1e-9 of it.
fn certify(grid: &Grid, heights: &[f64], f: &LoopFrame, rows: &[Row], ends: &[Station], bounds: &[SideBounds]) -> SplineResult<[f64; 3]> {
    let sides = ends.len();
    let per = rows.len() / sides.max(1);
    if sides == 0 || per == 0 || bounds.len() != sides {
        return invalid("stations");
    }
    let delta = 1.0 / per as f64;
    let spans = [grid.count[0] - 3, grid.count[1] - 3];
    let (Some(&lo0), Some(&lo1)) = (grid.knots[0].first(), grid.knots[1].first()) else { return invalid("grid") };
    let bezier = span_bezier(grid)?;
    let mut nets: Vec<Option<[[f64; 4]; 4]>> = vec![None; spans[0] * spans[1]];
    // One station's gap, normals' difference, curvature miss and `B`, and whether it is clear.
    let sample = |q: [f64; 2],
                  h: f64,
                  nn: Option<Vec3>,
                  bend: Option<([f64; 2], f64)>,
                  clear: bool|
     -> SplineResult<(f64, Option<f64>, Option<(f64, f64)>, bool)> {
        let Some([z, hu, hv, huu, huv, hvv]) = derivatives(grid, heights, q) else { return invalid("the fit is not finite") };
        let up = (1.0 + hu * hu + hv * hv).sqrt();
        let normal = (f.n - f.x * hu - f.y * hv) * (1.0 / up);
        let curvature = bend.map(|([a, b], kappa)| {
            let big_b = a * a + b * b + (a * hu + b * hv) * (a * hu + b * hv);
            ((a * a * huu + 2.0 * a * b * huv + b * b * hvv) / up / big_b - kappa, big_b)
        });
        Ok((z - h, nn.map(|n| (normal - n).len()), curvature, clear))
    };
    let mut out = [0.0f64; 3];
    for (k, b) in bounds.iter().enumerate() {
        let side = rows.get(k * per..(k + 1) * per).ok_or_else(|| SplineError::Invalid("stations".into()))?;
        let mut pts = Vec::with_capacity(per + 1);
        for r in side {
            pts.push((r.q, sample(r.q, r.h, r.gradient.map(|g| g.1), r.curvature.map(|c| (c.0, c.2)), r.to_corner >= FADE_SHARE)?));
        }
        let end = ends.get(k).ok_or_else(|| SplineError::Invalid("stations".into()))?;
        let d = end.p - f.o;
        let q = [d.dot(f.x), d.dot(f.y)];
        let (gradient, bend) = face_terms(end, f)?;
        let clear = rows.get(((k + 1) % sides) * per).is_some_and(|r| r.to_corner >= FADE_SHARE);
        pts.push((q, sample(q, d.dot(f.n), gradient.map(|g| g.1), bend.map(|c| (c.0, c.1)), clear)?));
        let (s, a) = (b.speed, b.accel);
        let reach = s * delta / 2.0;
        for pair in pts.windows(2) {
            let [(q0, (f0, d0, e0, c0)), (q1, (f1, d1, e1, c1))] = pair else { continue };
            // The spans the loop can cross between the two stations.
            let mut patch = [0.0f64; 3];
            let range = |dim: usize, lo: f64| {
                let at = |x: f64| (((x - lo) / grid.cell(dim)).floor().max(0.0) as usize).min(spans[dim] - 1);
                at(q0[dim].min(q1[dim]) - reach)..=at(q0[dim].max(q1[dim]) + reach)
            };
            let cells = [grid.cell(0), grid.cell(1)];
            let (lo, hi) = ([q0[0].min(q1[0]) - reach, q0[1].min(q1[1]) - reach], [q0[0].max(q1[0]) + reach, q0[1].max(q1[1]) + reach]);
            for sj in range(1, lo1) {
                for si in range(0, lo0) {
                    let slot = nets.get_mut(sj * spans[0] + si).ok_or_else(|| SplineError::Invalid("grid".into()))?;
                    let net = match slot {
                        Some(x) => *x,
                        None => *slot.insert(cell_net(grid, heights, &bezier, si, sj)?.0),
                    };
                    // The box's share of the span each way.
                    let share = |d: usize, base: f64, i: usize| {
                        let start = base + cells[d] * i as f64;
                        [((lo[d] - start) / cells[d]).clamp(0.0, 1.0), ((hi[d] - start) / cells[d]).clamp(0.0, 1.0)]
                    };
                    let (su, sv) = (share(0, lo0, si), share(1, lo1, sj));
                    let got = if su[0] < su[1] && sv[0] < sv[1] { net_bounds(&net, cells, su, sv)? } else { [0.0; 3] };
                    for (p, g) in patch.iter_mut().zip(got) {
                        *p = p.max(g);
                    }
                }
            }
            let [g, h2, t3] = patch;
            let second = delta * delta / 8.0;
            out[0] = out[0].max(f0.abs().max(f1.abs()) + (h2 * s * s + (g + 1.0) * a) * second);
            if let (Some(d0), Some(d1)) = (d0, d1) {
                let m2 = 2.0 * (t3 * s * s + h2 * a) + 3.0 * (h2 * s).powi(2) + b.normal_accel;
                out[1] = out[1].max(2.0 * ((d0.max(*d1) + m2 * second) / 2.0).min(1.0).asin());
            }
            if let (Some((e0, b0)), Some((e1, b1))) = (e0, e1) {
                if !(*c0 || *c1) {
                    continue;
                }
                let lr = b.leave_rate;
                let db = 2.0 * lr + 2.0 * g * (g * lr + h2 * s);
                let least = b0.min(*b1) - db * delta / 2.0;
                if !(least > 0.0) {
                    return Ok([out[0], out[1], f64::INFINITY]);
                }
                let lip = (2.0 * lr * h2 + t3 * s + h2 * (g * h2 * s + db / least)) / least + b.curvature_rate;
                let miss = match (c0, c1) {
                    (true, true) => e0.abs().max(e1.abs()) + lip * delta / 2.0,
                    (true, false) => e0.abs() + lip * delta,
                    _ => e1.abs() + lip * delta,
                };
                out[2] = out[2].max(miss);
            }
        }
    }
    let out = out.map(|x| x * (1.0 + 1e-9));
    Ok(if out.iter().all(|x| x.is_finite()) { out } else { [f64::INFINITY; 3] })
}

/// The pieces [`curve_bounds`] cuts each span of a curve into.
const CURVE_PIECES: usize = 16;

/// Bounds on a B-spline curve (degree, knots, homogeneous control points) over its domain, per
/// unit of its parameter: `[largest speed, largest acceleration, least speed]`, from each span's
/// Bézier hodographs: with `P` the weighted points less the first point times the weight and `w`
/// the weights, `p' = (P' − (p − c) w')/w` and `p'' = (P'' − 2 p' w' − (p − c) w'')/w`, each
/// hodograph's coefficients bounding it; the least speed from `p'·d` along the span's chord
/// direction `d`. `None` where a weight is not positive or a span's speed has no positive lower
/// bound so (a hull that turns too far). Rounding: raised (lowered) by 1e-9 of each.
pub fn curve_bounds(degree: usize, knots: &[f64], ctrl: &[HPoint]) -> Option<[f64; 3]> {
    let (&first, &last) = (knots.get(degree)?, knots.get(ctrl.len())?);
    let mut breaks: Vec<f64> = knots.iter().copied().filter(|k| *k >= first && *k <= last).collect();
    breaks.dedup();
    let (mut speed, mut accel, mut least) = (0.0f64, 0.0f64, f64::INFINITY);
    let p = degree as f64;
    // Each span in `CURVE_PIECES` pieces, whose hulls hug the curve closer.
    let pieces = breaks.windows(2).flat_map(|w| {
        let (lo, hi) = (w.first().copied().unwrap_or(0.0), w.last().copied().unwrap_or(0.0));
        (0..CURVE_PIECES).map(move |i| {
            (
                lo + (hi - lo) * i as f64 / CURVE_PIECES as f64,
                if i + 1 == CURVE_PIECES { hi } else { lo + (hi - lo) * (i + 1) as f64 / CURVE_PIECES as f64 },
            )
        })
    });
    for (a, b) in pieces {
        let (_, bez) = segment(degree, knots, ctrl, a, b).ok()?;
        if !bez.iter().all(|h| h.w > 0.0) {
            return None;
        }
        let c = bez.first()?.project()?;
        let pts: Vec<Vec3> = bez.iter().map(|h| h.project()).collect::<Option<_>>()?;
        let hod = |v: &[Vec3], f: f64| v.windows(2).map(|w| if let [x, y] = w { (*y - *x) * (f / (b - a)) } else { Vec3::ZERO }).collect::<Vec<_>>();
        let hods = |v: &[f64], f: f64| v.windows(2).map(|w| if let [x, y] = w { (y - x) * f / (b - a) } else { 0.0 }).collect::<Vec<_>>();
        let num: Vec<Vec3> = bez.iter().map(|h| h.xyz() - c * h.w).collect();
        let wts: Vec<f64> = bez.iter().map(|h| h.w).collect();
        let (n1, w1) = (hod(&num, p), hods(&wts, p));
        let (n2, w2) = (hod(&n1, p - 1.0), hods(&w1, p - 1.0));
        let big = |v: &[Vec3]| v.iter().fold(0.0f64, |m, x| m.max(x.len()));
        let bigs = |v: &[f64]| v.iter().fold(0.0f64, |m, x| m.max(x.abs()));
        let reach = pts.iter().fold(0.0f64, |m, x| m.max((*x - c).len()));
        let (wmin, wmax) = wts.iter().fold((f64::INFINITY, 0.0f64), |(lo, hi), w| (lo.min(*w), hi.max(*w)));
        let s = (big(&n1) + reach * bigs(&w1)) / wmin;
        let acc = (big(&n2) + 2.0 * s * bigs(&w1) + reach * bigs(&w2)) / wmin;
        let dir = (*pts.last()? - *pts.first()?).normalized()?;
        let low = (n1.iter().fold(f64::INFINITY, |m, x| m.min(x.dot(dir))) - reach * bigs(&w1)) / wmax;
        if !(low > 0.0 && s.is_finite() && acc.is_finite()) {
            return None;
        }
        (speed, accel, least) = (speed.max(s), accel.max(acc), least.min(low));
    }
    (least.is_finite()).then_some([speed * (1.0 + 1e-9), accel * (1.0 + 1e-9), least * (1.0 - 1e-9)])
}

/// Each row of a Bézier net split at its middle (de Casteljau, [`subdivide`]): the halves' nets.
fn split_rows(net: [[f64; 4]; 4]) -> ([[f64; 4]; 4], [[f64; 4]; 4]) {
    let halves = net.map(|row| {
        let (l, r) = subdivide(&row.map(|x| HPoint::weighted(Vec3::new(x, 0.0, 0.0), 1.0)), 0.5);
        let back = |h: Vec<HPoint>| {
            let mut out = [0.0; 4];
            out.iter_mut().zip(h).for_each(|(o, p)| *o = p.x);
            out
        };
        (back(l), back(r))
    });
    (halves.map(|h| h.0), halves.map(|h| h.1))
}

fn transpose(m: [[f64; 4]; 4]) -> [[f64; 4]; 4] {
    let [[a, b, c, d], [e, f, g, h], [i, j, k, l], [m, n, o, p]] = m;
    [[a, e, i, m], [b, f, j, n], [c, g, k, o], [d, h, l, p]]
}

/// The height field's derivatives at `q`: `[h, h_u, h_v, h_uu, h_uv, h_vv]`.
fn derivatives(grid: &Grid, heights: &[f64], q: [f64; 2]) -> Option<[f64; 6]> {
    let cu = *grid.count.first()?;
    let mut out = [0.0; 6];
    for (slot, (ru, rv)) in out.iter_mut().zip([(0, 0), (1, 0), (0, 1), (2, 0), (1, 1), (0, 2)]) {
        let (i0, bu) = grid.basis_derivative(0, q[0], ru)?;
        let (j0, bv) = grid.basis_derivative(1, q[1], rv)?;
        let mut s = 0.0;
        for (b, wv) in bv.iter().enumerate() {
            for (a, wu) in bu.iter().enumerate() {
                s += wu * wv * heights.get((j0 + b) * cu + i0 + a)?;
            }
        }
        *slot = s;
    }
    out.iter().all(|x| x.is_finite()).then_some(out)
}

/// One grid's fit of the rows: the normal equations of the weighed rows and the energy.
fn solve(grid: &Grid, rows: &[Row], cell: f64) -> SplineResult<Vec<f64>> {
    let [cu, cv] = grid.count;
    let n = cu * cv;
    let mut band = Band::new(n, 3 * cu + 3);
    let mut rhs = vec![0.0; n];
    let add = |terms: &[(usize, f64)], value: f64, weight: f64, band: &mut Band, rhs: &mut Vec<f64>| {
        let w2 = weight * weight;
        for &(r, cr) in terms {
            if let Some(slot) = rhs.get_mut(r) {
                *slot += w2 * cr * value;
            }
            for &(s, cs) in terms {
                band.add(r, s, w2 * cr * cs);
            }
        }
    };
    let mut fitted = 0usize;
    for row in rows.iter().filter(|r| r.fitted) {
        let basis = |r: [usize; 2]| -> SplineResult<(usize, [f64; 4], usize, [f64; 4])> {
            let (Some((i0, bu)), Some((j0, bv))) = (grid.basis_derivative(0, row.q[0], r[0]), grid.basis_derivative(1, row.q[1], r[1])) else {
                return invalid("a loop point is off the grid");
            };
            Ok((i0, bu, j0, bv))
        };
        // The 16 control heights a derivative of order `r` at the station draws on, scaled.
        let terms = |r: [usize; 2], scale: f64| -> SplineResult<Vec<(usize, f64)>> {
            let (i0, bu, j0, bv) = basis(r)?;
            let mut t = Vec::with_capacity(16);
            for (b, wv) in bv.iter().enumerate() {
                for (a, wu) in bu.iter().enumerate() {
                    t.push(((j0 + b) * cu + i0 + a, wu * wv * scale));
                }
            }
            Ok(t)
        };
        add(&terms([0, 0], 1.0)?, row.h, 1.0, &mut band, &mut rhs);
        fitted += 1;
        if row.corner {
            continue;
        }
        if let Some((g, _)) = row.gradient {
            add(&terms([1, 0], 1.0)?, g[0], cell, &mut band, &mut rhs);
            add(&terms([0, 1], 1.0)?, g[1], cell, &mut band, &mut rhs);
        }
        if let Some(([a, b], second, _)) = row.curvature {
            let w = cell * cell * fade(row.to_corner);
            if w > 0.0 {
                // a² h_uu + 2ab h_uv + b² h_vv, the three derivatives' terms summed per control
                // height.
                let mut sum: Vec<(usize, f64)> = Vec::with_capacity(48);
                for (r, s) in [([2, 0], a * a), ([1, 1], 2.0 * a * b), ([0, 2], b * b)] {
                    for (idx, v) in terms(r, s)? {
                        match sum.iter_mut().find(|(i, _)| *i == idx) {
                            Some(slot) => slot.1 += v,
                            None => sum.push((idx, v)),
                        }
                    }
                }
                add(&sum, second, w, &mut band, &mut rhs);
            }
        }
    }
    let lambda = FAIRNESS * fitted as f64 / n as f64 * grid.cell(0) * grid.cell(1);
    grid.add_energy(&mut band, lambda);
    band.solve(rhs)
}

/// What a fitted grid achieves at the rows: the deviation from the loop, the normal's angle to
/// the neighbours' (twice), the curvature mismatch (held stations, all).
fn achieved(grid: &Grid, heights: &[f64], rows: &[Row], f: &LoopFrame) -> SplineResult<[f64; 5]> {
    let reach = FADE_SHARE;
    let (mut dev, mut ang, mut cur, mut cur_all) = (0.0f64, 0.0f64, 0.0f64, 0.0f64);
    for row in rows {
        let Some([h, hu, hv, huu, huv, hvv]) = derivatives(grid, heights, row.q) else { return invalid("the fit is not finite") };
        dev = dev.max((h - row.h).abs());
        if row.corner {
            continue;
        }
        let up = (1.0 + hu * hu + hv * hv).sqrt();
        let normal = (f.n - f.x * hu - f.y * hv) * (1.0 / up);
        if let Some((_, nn)) = row.gradient {
            ang = ang.max(normal.dot(nn).clamp(-1.0, 1.0).acos());
        }
        if let Some(([a, b], _, kappa)) = row.curvature {
            // The normal curvature along the unit tangent of the surface over (a, b): the second
            // fundamental form over the first.
            let t = f.x * a + f.y * b + f.n * (a * hu + b * hv);
            let ii = (a * a * huu + 2.0 * a * b * huv + b * b * hvv) / up;
            let k = ii / t.len2().max(1e-300);
            let miss = (k - kappa).abs();
            cur_all = cur_all.max(miss);
            if row.to_corner >= reach {
                cur = cur.max(miss);
            }
        }
    }
    Ok([dev, ang, ang, cur, cur_all])
}

/// One round: the grids from `level` up, on these stations.
fn fit_round(all: &[(Station, bool)], ends: &[Station], spec: &MatchedSpec, level: usize, bounds: Option<&[SideBounds]>) -> SplineResult<Round> {
    let last = if bounds.is_some() { MAX_MATCHED_SPANS } else { PLAIN_SPANS };
    let fitted: Vec<Vec3> = all.iter().filter(|(_, f)| *f).map(|(s, _)| s.p).collect();
    let frame = loop_frame(&fitted)?;
    let points: Vec<Vec3> = all.iter().map(|(s, _)| s.p).collect();
    let rows = rows(all, ends, &frame, bounds.map(|_| spec.curvature_tol))?;
    let planes = if bounds.is_some() { planes(all, ends, spec.tol)? } else { Vec::new() };
    let corners: Vec<bool> = rows.iter().map(|r| r.corner).collect();
    let uv: Vec<[f64; 2]> = rows.iter().map(|r| r.q).collect();
    check_projectable(&points, &uv)?;
    let (mut lo, mut hi) = ([f64::INFINITY; 2], [f64::NEG_INFINITY; 2]);
    for q in &uv {
        for d in 0..2 {
            lo[d] = lo[d].min(q[d]);
            hi[d] = hi[d].max(q[d]);
        }
    }
    for d in 0..2 {
        let pad = 0.01 * (hi[d] - lo[d]);
        lo[d] -= pad;
        hi[d] += pad;
    }
    if !(lo[0] < hi[0] && lo[1] < hi[1] && (hi[0] - lo[0]).is_finite() && (hi[1] - lo[1]).is_finite()) {
        return invalid("the loop's extent is not finite");
    }
    // The fitted stations' largest step each way: a grid finer than half of it waits for denser
    // sampling.
    let fq: Vec<[f64; 2]> = rows.iter().filter(|r| r.fitted).map(|r| r.q).collect();
    let mut step = [0.0f64; 2];
    for k in 0..fq.len() {
        let (p, q) = (fq.get(k).copied().unwrap_or([0.0; 2]), fq.get((k + 1) % fq.len()).copied().unwrap_or([0.0; 2]));
        for d in 0..2 {
            step[d] = step[d].max((q[d] - p[d]).abs());
        }
    }
    let extent = [hi[0] - lo[0], hi[1] - lo[1]];
    let longer = extent[0].max(extent[1]);
    let mut best = Shortfall::default();
    let mut level = level;
    while level <= last {
        let spans = [0, 1].map(|d| ((extent[d] / longer * level as f64).ceil() as usize).clamp(1, level));
        let holds = (0..2).all(|d| extent[d] / spans[d] as f64 >= 2.0 * step[d]);
        if !holds && level > MIN_SPANS {
            return Ok(Round::Denser(best, level));
        }
        let grid = Grid::new(spans, lo, hi);
        let cell = grid.cell(0).max(grid.cell(1));
        let heights = solve(&grid, &rows, cell)?;
        let [dev, ang, ang_all, cur, cur_all] = achieved(&grid, &heights, &rows, &frame)?;
        let stations_hold = dev <= spec.tol && ang <= spec.angle_tol && cur <= spec.curvature_tol && holds;
        best = best.min(Shortfall { deviation: dev, angle: ang, curvature: cur });
        let accepted = stations_hold
            && match bounds {
                None => true,
                Some(b) => {
                    past_planes(&grid, &heights, &frame, &planes, &uv, &corners, spec.tol)? <= spec.tol && {
                        let [gap, angle, curvature] = certify(&grid, &heights, &frame, &rows, ends, b)?;
                        gap <= spec.tol && angle <= spec.angle_tol && curvature <= spec.curvature_tol
                    }
                }
            };
        if accepted {
            let surface = grid.surface(&heights, frame.o, frame.x, frame.y, frame.n)?;
            let fill = LoopFill {
                surface,
                origin: frame.o,
                axes: [frame.x, frame.y],
                normal: frame.n,
                counter_clockwise: frame.counter_clockwise,
                deviation: dev,
                evaluations: all.len(),
                crown: 0.0,
                crowned: 0,
                crown_drift: 0.0,
            };
            return Ok(Round::Done(Box::new(MatchedFill {
                fill,
                normal_deviation: ang,
                normal_deviation_all: ang_all,
                curvature_mismatch: cur,
                curvature_mismatch_all: cur_all,
                spans,
                stations: all.len(),
            })));
        }
        if !holds {
            return Ok(Round::Denser(best, level));
        }
        if level == last {
            break;
        }
        level = (level * 2).min(last);
    }
    Ok(Round::Exhausted(best))
}

#[cfg(test)]
#[path = "fill_matched_tests.rs"]
mod tests;
