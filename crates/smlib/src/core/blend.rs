//! Fillets and chamfers on solidcore: the C API's composition round SMLib's fillet executive,
//! ported from the fork's `capi/src/smlib_ops.cpp` (`blend_call`, `round_edges`,
//! `chamfer_offsets`, `chamfer_edges`, `chamfer_radius`, `blend_failure`, `setbacks_fit`,
//! `size_ok`, `points_ok`, `tolerance_ok`, `law_ok`, `law_pinches`, `law_along`, `finish`,
//! `smlib_fillet`, `smlib_chamfer`, `smlib_blend`, `smlib_blend_setback`,
//! `smlib_blend_setback_open`, `smlib_fillet_variable`, `smlib_chamfer_sides`) and
//! `capi/src/smlib_pick.h` (`pick`, `pick_edges`, `pick_vertices`).
//!
//! As there, every operation works on a copy of its body (`SmApiBrepCopy`), picks edges and
//! corners by point, and drives the executive step by step (`SetFilletParameters`,
//! `CreateFilletCorners`, the corners' bevels and setbacks, `DoFilleting`, `StitchAndOrient`).
//!
//! What the capi asks of fork-only SMLib code that solidcore does not have is
//! [`Error::Unsupported`], refused where the capi would first reach it, so every error before
//! it is the C++ path's:
//! - a curvature-continuous blend (`SetBlendCurveAtThirds`, fork fd3b6a7);
//! - a two-distance or distance-and-angle chamfer (`SetFilletParameters`' per-edge second
//!   offsets, fork f653ac9);
//! - a corner left open (`SetMarkCornerPatches`, fork 60af574);
//! - a variable radius whose law has a corner (`SplitFilletFacesAtRadiusCorners`, fork 6eecf04).
//!
//! - a blend whose rail leaves its face, which the fork may build over the cliff
//!   (`SmFilletCliffOverflow.cpp`, fork 8ba4667) and solidcore always refuses ([`blend_failure`]).
//!
//! solidcore's own refusals of code it has not ported (`SolidError::Unported`) are
//! [`Error::Unsupported`] too.

use std::collections::HashSet;
use std::sync::PoisonError;

use solidcore::SolidError;
use solidcore::blend::fillet_executive::p1::{FilletLaw, FilletLawKind};
use solidcore::blend::fillet_executive::session::FilletSession;
use solidcore::blend::fillet_geom::p1::{FilletCornerType, FilletErrorType};
use solidcore::blend::fillet_solver::p1::FilletSurfaceGeneratorType;
use solidcore::geom::vector3d::{Point3, Vec3};
use solidcore::kernel::Kernel;
use solidcore::mass::integrator::properties::edge_length;
use solidcore::nurbs::b_spline_curve::p1::{BSplineCurve, BSplineCurveForm, CanonicalData, KnotType};
use solidcore::query::api_queries::edge_closest_point;
use solidcore::topology::brep::p1 as brep_p1;
use solidcore::topology::brep::p2 as brep_p2;
use solidcore::topology::brep::p5::{StitchAndOrientOptions, stitch_and_orient};
use solidcore::topology::edgeuse::p1 as eu_p1;
use solidcore::topology::edgeuse::p3::{BinormalOptions, evaluate_binormal};
use solidcore::topology::{Brep, EdgeId, FaceId, VertexId, edge, vertex};

use super::boolean::copy_of;
use super::{Body, run};
use crate::{Blend, ChamferSecond, Error, MAX_POINTS, Result};

/// The capi's statuses as SolidCraft errors: an operation's own result.
type Out<T> = std::result::Result<T, Error>;

/// `SM_ERR_INVALID_INPUT`.
const INVALID_INPUT: i32 = 1007;

/// An SMLib status as the capi passes it on (`status`, smlib_shim.h): solidcore's upstream
/// status codes as they are; its refusals of code it has not ported as not supported.
fn status(e: SolidError) -> Error {
    match e {
        SolidError::Upstream { code, .. } => Error::Status(code),
        SolidError::InvalidInput { .. } => Error::Status(INVALID_INPUT),
        e if e.is_unported() => Error::Unsupported,
        e => Error::Solidcore(e.to_string()),
    }
}

/// `size_ok`.
fn size_ok(x: f64) -> bool {
    x.is_finite() && x > 0.0 && x <= 1e6
}

/// `tolerance_ok`.
fn tolerance_ok(t: f64) -> bool {
    t.is_finite() && t > 0.0 && t < 1e6
}

/// `points_ok`.
fn points_ok(points: &[[f64; 3]], allow_none: bool) -> bool {
    points.len() <= MAX_POINTS && (allow_none || !points.is_empty()) && points.iter().flatten().all(|x| x.is_finite())
}

fn p3(p: [f64; 3]) -> Point3 {
    Vec3::new(p[0], p[1], p[2])
}

/// Why a pick failed: `SMLIB_ERR_PICK_NONE` or `SMLIB_ERR_PICK_AMBIGUOUS`, and the point.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Miss {
    None(usize),
    Ambiguous(usize),
}

/// `smlib_pick::pick`: the entity nearest each point, each once, with the index of the point that
/// first picked it. A point picks none when the nearest is farther than `tol`, and is ambiguous
/// when the second nearest is within `tol` too and nearer than the first by at most `model_tol`.
/// An entity whose distance cannot be measured is skipped, as the capi skips a failed query
/// (solidcore's refusal of unported code is not skipped: it fails the pick).
fn pick<T: Copy + PartialEq>(
    all: &[T],
    points: &[[f64; 3]],
    tol: f64,
    model_tol: f64,
    mut dist: impl FnMut(T, Point3) -> solidcore::Result<Option<f64>>,
) -> solidcore::Result<std::result::Result<(Vec<T>, Vec<usize>), Miss>> {
    let (mut picked, mut first) = (Vec::new(), Vec::new());
    for (i, p) in points.iter().enumerate() {
        let p = p3(*p);
        let (mut best, mut d0, mut d1) = (None, f64::INFINITY, f64::INFINITY);
        for &x in all {
            let Some(d) = dist(x, p)? else { continue };
            if !d.is_finite() {
                continue;
            }
            if d < d0 {
                d1 = d0;
                d0 = d;
                best = Some(x);
            } else if d < d1 {
                d1 = d;
            }
        }
        let Some(best) = best.filter(|_| d0 <= tol) else {
            return Ok(Err(Miss::None(i)));
        };
        if d1 <= tol && d1 - d0 <= model_tol {
            return Ok(Err(Miss::Ambiguous(i)));
        }
        if !picked.contains(&best) {
            picked.push(best);
            first.push(i);
        }
    }
    Ok(Ok((picked, first)))
}

/// A query's failure the capi treats as no answer; solidcore's unported code still fails.
fn answer<T>(r: solidcore::Result<T>) -> solidcore::Result<Option<T>> {
    match r {
        Ok(v) => Ok(Some(v)),
        Err(e) if e.is_unported() => Err(e),
        Err(_) => Ok(None),
    }
}

/// `pick_edges` (`edge_distance`: `SmApiEdgeClosestPoint`).
fn pick_edges(k: &Kernel, b: &Brep, points: &[[f64; 3]], tol: f64) -> solidcore::Result<std::result::Result<(Vec<EdgeId>, Vec<usize>), Miss>> {
    let all = brep_p1::edges(b, None)?;
    pick(&all, points, tol, b.tolerance(), |e, p| Ok(answer(edge_closest_point(k, b, e, p))?.map(|c| c.closest.distance)))
}

/// `pick_vertices` (`vertex_distance`).
fn pick_vertices(b: &Brep, points: &[[f64; 3]], tol: f64) -> solidcore::Result<std::result::Result<(Vec<VertexId>, Vec<usize>), Miss>> {
    let all = brep_p1::vertices(b, None)?;
    pick(&all, points, tol, b.tolerance(), |v, p| Ok(Some((b.vertex(v)?.point - p).length())))
}

/// An edge pick's failure as `pick_error` (ops.rs) reads it.
fn edge_miss(m: Miss) -> Error {
    match m {
        Miss::None(point) => Error::NoMatch { point },
        Miss::Ambiguous(point) => Error::Ambiguous { point },
    }
}

/// A corner pick's failure as `setback_error` (ops.rs) reads it: the capi writes the corner
/// point's index to `out_failed_corner` only, so the status comes back bare.
fn corner_miss(m: Miss) -> Error {
    match m {
        Miss::None(_) => Error::Status(crate::ffi::ops::ERR_PICK_NONE),
        Miss::Ambiguous(_) => Error::Status(crate::ffi::ops::ERR_PICK_AMBIGUOUS),
    }
}

/// A corner set back: its vertex and the setback along each of its edges (the capi's
/// `SetBack`; the routed entry points take no ease, which is 0).
struct SetBack {
    vertex: VertexId,
    distance: f64,
}

/// `setbacks_fit`: each setback shorter than every edge at its corner, and where an edge's
/// other end is a setback corner too, the two together shorter than the edge.
fn setbacks_fit(k: &Kernel, b: &Brep, setbacks: &[SetBack]) -> solidcore::Result<bool> {
    for sb in setbacks {
        for e in vertex::edges(b, sb.vertex, None)? {
            let Some(length) = answer(edge_length(k, b, e, 1e-6))? else {
                return Ok(false);
            };
            let far = edge::other_vertex(b, e, Some(sb.vertex))?;
            let mut other = 0.0;
            for o in setbacks {
                if Some(o.vertex) == far && far != Some(sb.vertex) {
                    other = o.distance;
                }
            }
            if sb.distance + other >= length {
                return Ok(false);
            }
        }
    }
    Ok(true)
}

/// The midpoint parameter of `e` (`GetInterval().Evaluate(0.5)`).
fn mid(b: &Brep, e: EdgeId) -> solidcore::Result<f64> {
    Ok(b.edge(e)?.interval.evaluate(0.5))
}

/// `chamfer_radius`: the radius of the ball whose tangent points on the two faces of `e` lie
/// `distance` back from the edge at its midpoint, `distance * tan(a / 2)` with `a` the angle
/// between the directions into the faces. `None` where the capi's returns false.
fn chamfer_radius(k: &Kernel, b: &mut Brep, e: EdgeId, distance: f64) -> solidcore::Result<Option<f64>> {
    let Some(u0) = edge::blend_edgeuse(k, b, e)? else { return Ok(None) };
    let Some(u1) = eu_p1::radial(b, u0)? else { return Ok(None) };
    let t = mid(b, e)?;
    let options = BinormalOptions::default();
    let Some(e0) = answer(evaluate_binormal(k, b, u0, t, false, &options))? else { return Ok(None) };
    let Some(e1) = answer(evaluate_binormal(k, b, u1, t, false, &options))? else { return Ok(None) };
    let c = e0.binormal.dot(e1.binormal).clamp(-1.0, 1.0);
    let radius = distance * ((1.0 - c) / (1.0 + c)).sqrt();
    Ok((radius.is_finite() && radius > 0.0).then_some(radius))
}

/// What `round_edges` makes: the capi's kinds less the chamfer, which `chamfer_offsets` makes.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Section {
    Round,
    Chord,
    Chamfer,
}

/// The faces of `b`, for `made_faces`: a face whose id is new after the operation was made by
/// it. (The capi marks faces with an attribute SMLib copies through every split; here a face
/// split by the operation counts as made, which only matters to a blend that made no blend
/// face but split one, never seen.)
fn face_set(b: &Brep) -> solidcore::Result<HashSet<FaceId>> {
    Ok(brep_p1::faces(b, None)?.into_iter().collect())
}

/// solidcore's refusal of a blend whose rail leaves the face it rolls on (its port of fork
/// fc43974, `refuse_blends_larger_than_faces`).
const RAIL_LEAVES_FACE: &str = "Fillet Error: a rail leaves the face it rolls on (the blend is larger than the faces allow)";

/// `blend_failure`: a blend the executive found larger than its faces allow
/// (`SM_FILERR_NO_ROOM`) is [`Error::NoRoom`]; any other failure its status.
///
/// Except a blend whose rail leaves its face: there the fork decides between building the
/// blend over the cliff (`SmFilletCliffOverflow.cpp`, fork 8ba4667) and refusing it, which
/// solidcore does not port, refusing every such blend. Its answer is the fork's only by chance,
/// so it is [`Error::Unsupported`].
fn blend_failure(s: &FilletSession<'_>, e: SolidError) -> Error {
    let info = s.error_info();
    if info.code != FilletErrorType::NoRoom as i32 || e.is_unported() {
        status(e)
    } else if info.comment == RAIL_LEAVES_FACE {
        Error::Unsupported
    } else {
        Error::NoRoom
    }
}

/// `round_edges` and `chamfer_offsets` (with equal offsets): `edges` of `b` blended in place,
/// edge i at `sizes[i]`, the executive driven as `SmApiFilletEdgesPerEdge` drives it with the
/// corners' setbacks (`SmFilletCorner::SetSetBackDist`) set between making the corners and
/// filleting, or (a chamfer) every corner bevelled; then `StitchAndOrient`. A blend that made
/// no face is [`Error::NoEffect`]. A setback corner that is not SMLib's same-convexity N x N
/// corner is refused with its index in `setbacks` (`Err(Ok(j))`).
fn blend_edges(
    k: &mut Kernel,
    b: &mut Brep,
    edges: &[EdgeId],
    sizes: &[f64],
    section: Section,
    law: Option<FilletLaw>,
    setbacks: &[SetBack],
) -> solidcore::Result<std::result::Result<(), std::result::Result<usize, Error>>> {
    let before = face_set(b)?;
    let done = {
        let mut s = FilletSession::new(k, b)?;
        // `SetFilletParameters(b, es, sections, solvers, values, laws, -1, -1, -1, -1,
        // continuity 1, thumbweight 1)`: the session sets the executive's global merge first, as
        // `SmApiFilletEdges` does, and `SetFilletParameters` resets it, as in the capi.
        let prepared = match (section, law) {
            (Section::Round, None) => s.prepare_per_edge_constant_radius(k, edges, sizes, &[FilletSurfaceGeneratorType::Circular], 1, 1.0),
            // The size is the start radius (`smlib_fillet_variable`).
            (Section::Round, Some(law)) => match sizes.first() {
                Some(&size) => s.prepare_variable_radius_edges(k, edges, size, law, 1, 1),
                None => return Ok(Err(Err(Error::Argument))),
            },
            (Section::Chord, _) => s.prepare_per_edge_constant_distance(k, edges, sizes, &[FilletSurfaceGeneratorType::Circular], 1, 1.0),
            (Section::Chamfer, _) => s.prepare_per_edge_constant_radius(k, edges, sizes, &[FilletSurfaceGeneratorType::Linear], 1, 1.0),
        };
        let mut step = prepared.and_then(|()| s.create_corners(k, None)).map(|()| Ok(()));
        if let Ok(Ok(())) = step {
            step = corners(&mut s, section, setbacks);
        }
        match step {
            Ok(Ok(())) => match s.do_filleting(k, None) {
                Ok(()) => Ok(Ok(())),
                Err(e) => Ok(Err(Err(blend_failure(&s, e)))),
            },
            Ok(Err(refusal)) => Ok(Err(refusal)),
            Err(e) => Ok(Err(Err(blend_failure(&s, e)))),
        }
    };
    let done: solidcore::Result<std::result::Result<(), std::result::Result<usize, Error>>> = done;
    let stitched = match done? {
        Ok(()) => stitch_and_orient(k, b, &StitchAndOrientOptions::default()).map_err(status),
        Err(refusal) => {
            b.refresh_cached_topology();
            return Ok(Err(refusal));
        }
    };
    // The port's cached topology, true again for what reads the body next (as solidcore's own
    // fillet entry points do).
    b.refresh_cached_topology();
    if let Err(e) = stitched {
        return Ok(Err(Err(e)));
    }
    let made = face_set(b)?.difference(&before).count();
    Ok(if made > 0 { Ok(()) } else { Err(Err(Error::NoEffect)) })
}

/// The corners' part of `round_edges` / `chamfer_offsets`: a chamfer's corners bevelled; each
/// setback set on its vertex's corner (`GetFilletCornerOfVertex`), which must be an
/// `SM_FCR_N_x_N_CONVEX` corner.
fn corners(
    s: &mut FilletSession<'_>,
    section: Section,
    setbacks: &[SetBack],
) -> solidcore::Result<std::result::Result<(), std::result::Result<usize, Error>>> {
    let ids = s.fillet_corners()?;
    if section == Section::Chamfer {
        for id in ids {
            s.set_corner_bevel(id, true)?;
        }
        return Ok(Ok(()));
    }
    for (j, sb) in setbacks.iter().enumerate() {
        let mut found = None;
        for &id in &ids {
            if s.corner(id)?.vertex == Some(sb.vertex) {
                found = Some(id);
                break;
            }
        }
        let Some(id) = found else {
            return Ok(Err(Err(Error::Argument)));
        };
        if s.corner(id)?.corner_type() != FilletCornerType::NByNConvex {
            return Ok(Err(Ok(j)));
        }
        s.set_corner_setback(id, sb.distance)?;
    }
    Ok(Ok(()))
}

/// `finish`: the result must have faces, and a solid input must give a closed manifold solid.
fn finish(b: Brep, solid_in: bool) -> solidcore::Result<Out<Brep>> {
    if b.face_list().is_empty() {
        return Ok(Err(Error::Empty));
    }
    if solid_in && !brep_p2::is_manifold_solid(&b)? {
        return Ok(Err(Error::NotSolid));
    }
    Ok(Ok(b))
}

/// Runs `f` on a copy of `body` (`copy_of`) under the kernel's lock.
fn on_copy(call: &'static str, body: &Body, f: impl FnOnce(&mut Kernel, Brep) -> solidcore::Result<Out<Brep>>) -> Result<Body> {
    if body.0.is_poisoned() {
        return Err(Error::Internal(format!("{call}: the body was left unfinished by an earlier panic")));
    }
    run(call, |k| {
        let g = body.0.lock().unwrap_or_else(PoisonError::into_inner);
        let c = copy_of(k, &g, 1)?;
        drop(g);
        f(k, c)
    })?
    .map(Body::new)
}

/// What [`blend`] does with the picked corners' patches.
#[derive(Clone, Copy, PartialEq, Eq)]
pub(crate) enum Corners {
    /// Filled, as SMLib builds them (`smlib_blend`, `smlib_blend_setback`).
    Filled,
    /// Removed, the holes listed (`smlib_blend_setback_open`): not supported on solidcore.
    Open,
}

/// `blend_call` (as `smlib_fillet`, `smlib_chamfer`, `smlib_blend`, `smlib_blend_setback` and
/// `smlib_blend_setback_open` call it): `sizes` one per point, or one for all; `corners` and
/// `setbacks` the corner setbacks.
#[allow(clippy::too_many_arguments)] // the capi's blend_call
pub(crate) fn blend(
    body: &Body,
    kind: Blend,
    points: &[[f64; 3]],
    sizes: &[f64],
    tolerance: f64,
    corners: &[[f64; 3]],
    setbacks: &[f64],
    open: Corners,
) -> Result<Body> {
    let one_size = sizes.len() == 1;
    if !points_ok(points, false)
        || !tolerance_ok(tolerance)
        || !(one_size || sizes.len() == points.len())
        || !points_ok(corners, true)
        || setbacks.len() != corners.len()
    {
        return Err(Error::Argument);
    }
    if !sizes.iter().all(|&s| size_ok(s)) || !setbacks.iter().all(|&s| size_ok(s)) {
        return Err(Error::Argument);
    }
    // A chamfer's corners are bevelled, which builds no setback.
    if !corners.is_empty() && kind == Blend::Chamfer {
        return Err(Error::Unsupported);
    }
    on_copy("blend_call", body, |k, mut w| {
        let (picked, first) = match pick_edges(k, &w, points, tolerance)? {
            Ok(p) => p,
            Err(m) => return Ok(Err(edge_miss(m))),
        };
        let each: Vec<f64> = first.iter().map(|&i| sizes.get(if one_size { 0 } else { i }).copied().unwrap_or(0.0)).collect();
        // Each corner point picks a vertex; a vertex picked twice takes the first point's setback.
        let (at, first_corner) = match pick_vertices(&w, corners, tolerance)? {
            Ok(p) => p,
            Err(m) => return Ok(Err(corner_miss(m))),
        };
        let back: Vec<SetBack> =
            at.iter().zip(&first_corner).map(|(&vertex, &j)| SetBack { vertex, distance: setbacks.get(j).copied().unwrap_or(0.0) }).collect();
        if !setbacks_fit(k, &w, &back)? {
            return Ok(Err(Error::NoRoom));
        }
        let solid_in = brep_p2::is_manifold_solid(&w)?;
        let section = match kind {
            Blend::Round => Section::Round,
            Blend::Chord => Section::Chord,
            // `chamfer_edges`: each edge's offsets from its setback at its midpoint.
            Blend::Chamfer => {
                let mut radii = Vec::with_capacity(picked.len());
                for (&e, &d) in picked.iter().zip(&each) {
                    match chamfer_radius(k, &mut w, e, d)? {
                        Some(r) => radii.push(r),
                        None => return Ok(Err(Error::Status(INVALID_INPUT))),
                    }
                }
                return Ok(match blend_edges(k, &mut w, &picked, &radii, Section::Chamfer, None, &[])? {
                    Ok(()) => finish(w, solid_in)?,
                    Err(Ok(_)) => Err(Error::Unsupported),
                    Err(Err(e)) => Err(e),
                });
            }
            // The curvature-continuous section sits at thirds (`SetBlendCurveAtThirds`, fork
            // fd3b6a7), which solidcore does not have.
            Blend::Curvature => return Ok(Err(Error::Unsupported)),
        };
        // The corners' patches marked to be opened (`SetMarkCornerPatches`, fork 60af574),
        // which solidcore does not have.
        if open == Corners::Open {
            return Ok(Err(Error::Unsupported));
        }
        Ok(match blend_edges(k, &mut w, &picked, &each, section, None, &back)? {
            Ok(()) => finish(w, solid_in)?,
            // A corner SMLib does not set back: the first of the caller's points that picked it.
            Err(Ok(j)) => Err(first_corner.get(j).map_or(Error::Unsupported, |&corner| Error::CornerRefused { corner })),
            Err(Err(e)) => Err(e),
        })
    })
}

/// `smlib_chamfer_sides`: its checks and the pick, then not supported: the chamfer's two
/// offsets per edge (`SetFilletParameters`' `pOptSecondRadii`, fork f653ac9) are not in
/// solidcore's executive.
pub(crate) fn chamfer_sides(body: &Body, points: &[[f64; 3]], tolerance: f64, distance: f64, second: ChamferSecond, first: [f64; 3]) -> Result<Body> {
    let second_ok = match second {
        ChamferSecond::Angle(a) => a.is_finite() && a > 1e-6 && a < std::f64::consts::PI - 1e-6,
        ChamferSecond::Distance(d) => size_ok(d),
    };
    if !points_ok(points, false) || !tolerance_ok(tolerance) || !size_ok(distance) || !second_ok || !first.iter().all(|x| x.is_finite()) {
        return Err(Error::Argument);
    }
    if !(p3(first).length() > 0.0) {
        return Err(Error::Argument);
    }
    on_copy("smlib_chamfer_sides", body, |k, w| {
        if let Err(m) = pick_edges(k, &w, points, tolerance)? {
            return Ok(Err(edge_miss(m)));
        }
        Ok(Err(Error::Unsupported))
    })
}

/// `law_ok`: a clamped B-spline of `degree` 1 to 3 from 0 to 1, 2 to 100 control radii, each
/// finite, at least 0 and at most 1e6.
fn law_ok(degree: usize, knots: &[f64], radii: &[f64]) -> bool {
    let n = radii.len();
    if !(1..=3).contains(&degree) || !(2..=100).contains(&n) || n <= degree || knots.len() != n + degree + 1 {
        return false;
    }
    if !knots.iter().chain(radii).all(|x| x.is_finite()) {
        return false;
    }
    for (i, &k) in knots.iter().enumerate() {
        let (low_end, high_end) = (i <= degree, i >= n);
        if (low_end && k != 0.0) || (high_end && k != 1.0) || (i > 0 && knots.get(i - 1).is_some_and(|&p| k < p)) {
            return false;
        }
    }
    radii.iter().all(|&r| (0.0..=1e6).contains(&r))
}

/// `law_pinches`: whether the law's radius is zero somewhere inside (0, 1).
fn law_pinches(degree: usize, knots: &[f64], radii: &[f64]) -> bool {
    let n = radii.len() as i64;
    let p = degree as i64;
    let all_zero = |from: i64, to: i64| -> bool {
        let lo = from.max(0);
        let mut j = lo;
        while j <= to && j < n {
            if radii.get(j as usize).is_some_and(|&r| r != 0.0) {
                return false;
            }
            j += 1;
        }
        from <= to
    };
    let kn = |i: i64| knots.get(i as usize).copied().unwrap_or(f64::NAN);
    let n_knots = n + p + 1;
    let mut i = p;
    while i + 1 < n_knots && i < n {
        if kn(i) < kn(i + 1) && all_zero(i - p, i) {
            return true;
        }
        i += 1;
    }
    let mut s = p + 1;
    while s < n {
        let mut m = 1;
        while s + m < n && kn(s + m) == kn(s) {
            m += 1;
        }
        if all_zero(s + m - 1 - p, s - 1) {
            return true;
        }
        s += m;
    }
    false
}

/// Whether the law has a corner inside (0, 1) (`SmBSplineFilletLaw::GetCornerParameters`, fork
/// 6eecf04): an interior knot where the radius' slope from the left and from the right differ by
/// more than `SM_EFF_ZERO * (1 + |left| + |right|)`. The slope is the derivative B-spline's,
/// `Q_i = p (r_{i+1} - r_i) / (u_{i+p+1} - u_{i+1})`; it jumps only at a knot repeated `m >= p`
/// times, from `Q_{s-2}` to `Q_{s+m-1-p}` (`s` the knot's first index).
fn law_has_corner(degree: usize, knots: &[f64], radii: &[f64]) -> bool {
    const EFF_ZERO: f64 = 1e-12;
    let p = degree;
    let q = |i: usize| -> Option<f64> {
        let (r0, r1) = (radii.get(i)?, radii.get(i + 1)?);
        let span = knots.get(i + p + 1)? - knots.get(i + 1)?;
        Some(p as f64 * (r1 - r0) / span)
    };
    let n = radii.len();
    let mut s = p + 1;
    while s < n {
        let Some(&u) = knots.get(s) else { break };
        let mut m = 1;
        while knots.get(s + m) == Some(&u) {
            m += 1;
        }
        if m >= p && u > 0.0 && u < 1.0 {
            let (Some(left), Some(right)) = (s.checked_sub(2).and_then(q), q(s + m - 1 - p)) else {
                return true;
            };
            if (left - right).abs() > EFF_ZERO * (1.0 + left.abs() + right.abs()) {
                return true;
            }
        }
        s += m;
    }
    false
}

/// `law_along`: the radius law along `e`, the B-spline with the radius as its x and its own
/// parameter as its y (control points at the Greville abscissae), reversed when `start` is
/// nearer the end of the edge's interval than its start. `None` where the capi's is null.
fn law_along(k: &Kernel, b: &Brep, e: EdgeId, start: [f64; 3], degree: usize, knots: &[f64], radii: &[f64]) -> solidcore::Result<Option<FilletLaw>> {
    let edge = b.edge(e)?;
    let Some(curve) = edge.curve.as_ref() else { return Ok(None) };
    let ivl = edge.interval;
    let (Some(lo), Some(hi)) = (answer(curve.evaluate_point(k, ivl.min()))?, answer(curve.evaluate_point(k, ivl.max()))?) else {
        return Ok(None);
    };
    let at = p3(start);
    let reverse = (hi - at).length() < (lo - at).length();
    let (nk, nr) = (knots.len(), radii.len());
    let kv: Vec<f64> = (0..nk).map(|i| if reverse { 1.0 - knots[nk - 1 - i] } else { knots[i] }).collect();
    let mut cps = Vec::with_capacity(nr);
    for i in 0..nr {
        let g: f64 = (1..=degree).filter_map(|j| kv.get(i + j)).sum();
        let r = if reverse { radii[nr - 1 - i] } else { radii[i] };
        cps.push(Vec3::new(r, g / degree as f64, 0.0));
    }
    let (mut unique, mut mults): (Vec<f64>, Vec<usize>) = (Vec::new(), Vec::new());
    for &x in &kv {
        match (unique.last(), mults.last_mut()) {
            (Some(&u), Some(m)) if u == x => *m += 1,
            _ => {
                unique.push(x);
                mults.push(1);
            }
        }
    }
    let data = CanonicalData {
        dim: 3,
        degree,
        control_points: &cps,
        form: BSplineCurveForm::Unspecified,
        knot_multiplicities: &mults,
        knots: &unique,
        knot_type: KnotType::Unspecified,
        weights: None,
        trim: None,
    };
    let Some(c) = answer(BSplineCurve::create_canonical(&data))? else { return Ok(None) };
    // The law's map is the whole curve, 0 to 1 (`FilletLaw::new`'s).
    Ok(Some(FilletLaw::new(FilletLawKind::BSpline { law_curve: c, extended_law: None })))
}

/// `smlib_fillet_variable`: the one edge nearest `point` rounded with the radius law of
/// `degree`, `knots` and `radii`, 0 at the end nearest `start`.
pub(crate) fn fillet_variable(
    body: &Body,
    point: [f64; 3],
    tolerance: f64,
    start: [f64; 3],
    degree: usize,
    knots: &[f64],
    radii: &[f64],
) -> Result<Body> {
    if !points_ok(&[point], false) || !start.iter().all(|x| x.is_finite()) || !tolerance_ok(tolerance) || !law_ok(degree, knots, radii) {
        return Err(Error::Argument);
    }
    if law_pinches(degree, knots, radii) {
        return Err(Error::Unsupported);
    }
    on_copy("smlib_fillet_variable", body, |k, mut w| {
        let picked = match pick_edges(k, &w, &[point], tolerance)? {
            Ok((p, _)) => p,
            Err(m) => return Ok(Err(edge_miss(m))),
        };
        let [e] = picked[..] else { return Ok(Err(Error::NoMatch { point: 0 })) };
        let Some(law) = law_along(k, &w, e, start, degree, knots, radii)? else {
            return Ok(Err(Error::Status(INVALID_INPUT)));
        };
        // A law with a corner splits the round's face there (`SplitFilletFacesAtRadiusCorners`,
        // fork 6eecf04), which solidcore does not have.
        if law_has_corner(degree, knots, radii) {
            return Ok(Err(Error::Unsupported));
        }
        let solid_in = brep_p2::is_manifold_solid(&w)?;
        let (r0, r1) = (radii.first().copied().unwrap_or(0.0), radii.last().copied().unwrap_or(0.0));
        Ok(match blend_edges(k, &mut w, &[e], &[r0.max(r1)], Section::Round, Some(law), &[])? {
            Ok(()) => finish(w, solid_in)?,
            Err(Ok(_)) => Err(Error::Unsupported),
            Err(Err(e)) => Err(e),
        })
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn laws_are_checked_as_the_capi_checks_them() {
        assert!(law_ok(1, &[0.0, 0.0, 1.0, 1.0], &[1.0, 2.0]));
        assert!(!law_ok(1, &[0.0, 0.0, 1.0], &[1.0, 2.0]));
        assert!(!law_ok(1, &[0.0, 0.5, 1.0, 1.0], &[1.0, 2.0]));
        assert!(!law_ok(1, &[0.0, 0.0, 1.0, 1.0], &[-1.0, 2.0]));
        assert!(!law_ok(4, &[0.0; 7], &[1.0, 2.0]));
        assert!(law_pinches(1, &[0.0, 0.0, 0.5, 1.0, 1.0], &[1.0, 0.0, 1.0]));
        assert!(!law_pinches(1, &[0.0, 0.0, 1.0, 1.0], &[0.0, 1.0]));
        // A straight law has no corner; a bent one does at its inner knot; a cubic none.
        assert!(!law_has_corner(1, &[0.0, 0.0, 1.0, 1.0], &[1.0, 2.0]));
        assert!(!law_has_corner(1, &[0.0, 0.0, 0.5, 1.0, 1.0], &[1.0, 1.5, 2.0]));
        assert!(law_has_corner(1, &[0.0, 0.0, 0.5, 1.0, 1.0], &[1.0, 2.0, 1.0]));
        assert!(!law_has_corner(3, &[0.0, 0.0, 0.0, 0.0, 0.5, 1.0, 1.0, 1.0, 1.0], &[1.0, 2.0, 1.0, 2.0, 1.0]));
    }

    #[test]
    fn picks_follow_the_capis_rule() {
        let all = [0.0f64, 1.0, 3.0];
        let d = |x: f64, p: Point3| Ok(Some((x - p.x).abs()));
        assert_eq!(pick(&all, &[[0.9, 0.0, 0.0]], 0.5, 1e-6, d).unwrap(), Ok((vec![1.0], vec![0])));
        assert_eq!(pick(&all, &[[2.0, 0.0, 0.0]], 0.5, 1e-6, d).unwrap(), Err(Miss::None(0)));
        assert_eq!(pick(&all, &[[1.0, 0.0, 0.0], [0.5, 0.0, 0.0]], 1.0, 1e-6, d).unwrap(), Err(Miss::Ambiguous(1)));
        // The same entity twice is picked once, by its first point.
        assert_eq!(pick(&all, &[[3.1, 0.0, 0.0], [2.9, 0.0, 0.0]], 0.5, 1e-6, d).unwrap(), Ok((vec![3.0], vec![0])));
    }
}
