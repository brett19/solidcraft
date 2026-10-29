//! Shells, face offsets, thickening, sheet offsets and drafts on solidcore: the C API's logic
//! round SMLib's offset executive, ported from the fork's `capi/src/smlib_ops.cpp`
//! (`smlib_shell`, `smlib_offset_faces`, `smlib_thicken`, `smlib_draft`, `leaves_cavity`,
//! `FaceSet`, `outer_faceuse`, `move_faces`, `draft_faces`), `capi/src/smlib_sheets.cpp`
//! (`smlib_offset_sheet`) and `capi/src/smlib_pick.h` (`pick_faces`).
//!
//! As there, every operation works on a copy of its body (`SmApiBrepCopy`) and picks faces by
//! point; the shell and the offsets are `SmApiShellBrepFull` and `SmApiOffsetBrepFull`, with
//! convex edges extended and the results merged.
//!
//! What the capi asks of SMLib code that solidcore does not have is [`Error::Unsupported`],
//! refused where the capi would first reach it, so every error before it is the C++ path's: a
//! face move (`SmOffsetFacesExecutive`, fork a96ce9f, solidcore#1060).
//!
//! A draft's tilted faces go in through solidcore's `SmBrep::LocalOperation`
//! ([`local_operation`]); the capi's check that it moved some face (`moved_faces`) needs face
//! history solidcore does not report (solidcore#1462) and is not made.

use solidcore::EntityRef;
use solidcore::SolidError;
use solidcore::curve::curve::Curve;
use solidcore::geom::attribute::{AttributeStore, ObjectKey};
use solidcore::geom::axis2_placement::Axis2Placement;
use solidcore::geom::object::{ObjectRef, Shape};
use solidcore::geom::vector3d::{Point3, Vec3};
use solidcore::intersect::point_class::PointClassType;
use solidcore::kernel::Kernel;
use solidcore::math::Point2;
use solidcore::nurbs::b_spline_surface_n_lib::p2::OrientType;
use solidcore::nurbs::knots::Side;
use solidcore::offset::offset_executive::p2::{
    OffsetNotifications, OffsetOptions, ShellCopyIdentities, ShellOptions, api_offset_brep_full, api_shell_brep_full,
};
use solidcore::query::api_queries::{BrepRef, face_bounding_box, face_classify_uv, face_closest_point_in_shape, face_internal_point};
use solidcore::query::brep::{BrepIdentity, BrepQuery, LiveBrepObjects};
use solidcore::surface::surface::SurfaceKind;
use solidcore::sweep::topology_sweep::store::SweepIdentities;
use solidcore::topology::brep::local_operation::local_operation;
use solidcore::topology::brep::{p1 as brep_p1, p2 as brep_p2, p3 as brep_p3};
use solidcore::topology::face::p2 as face_p2;
use solidcore::topology::topology::{NotifyRecords, Topology};
use solidcore::topology::{Brep, BrepId, EdgeId, FaceId, FaceuseId, RegionId};

use super::blend::{Miss, on_copy, p3, pick, points_ok, status as blend_status, tolerance_ok};
use super::{Body, answer};
use crate::{Error, Result};

/// The capi's statuses as SolidCraft errors: an operation's own result.
type Out<T> = std::result::Result<T, Error>;

/// `SM_ERR_INVALID_INPUT`.
const INVALID_INPUT: i32 = 1007;

/// `SM_ERR_OFFSET_WALLS_TOUCH` (fork a759622), which `status` (smlib_shim.h) reports as
/// `SMLIB_ERR_WALLS_TOUCH`.
const OFFSET_WALLS_TOUCH: i32 = 1042;

/// `status` (smlib_shim.h): an offset whose walls touch is [`Error::WallsTouch`]; any other
/// status as the blends pass it on.
fn status(e: SolidError) -> Error {
    match e {
        SolidError::Upstream { code: OFFSET_WALLS_TOUCH, .. } => Error::WallsTouch,
        e => blend_status(e),
    }
}

/// `std::fabs(x) > 1e6` and the rest of the capi's size checks: finite, not zero, at most 1e6.
fn distance_ok(x: f64) -> bool {
    x.is_finite() && x != 0.0 && x.abs() <= 1e6
}

/// The B-rep id every shape and face key in this module names the one body by.
const ONE: BrepId = BrepId::from_index(0);

/// Faces with their loose bounding boxes, for nearest-face queries (`FaceSet`).
struct FaceSet {
    faces: Vec<FaceId>,
    boxes: Vec<(Point3, Point3)>,
    /// Each face's shape (`SmApiFaceClosestPoint` makes one per query; it depends only on the
    /// face, so it is made once).
    shapes: Vec<Shape>,
}

impl FaceSet {
    fn new() -> FaceSet {
        FaceSet { faces: Vec::new(), boxes: Vec::new(), shapes: Vec::new() }
    }

    /// `add`: false when the face's box cannot be measured.
    fn add(&mut self, k: &mut Kernel, b: &Brep, f: FaceId) -> solidcore::Result<bool> {
        let Some(bx) = answer(face_bounding_box(k, b, f, false))? else { return Ok(false) };
        let Some(shape) = answer(Shape::new(k, &(ONE, b), &[], &[(ONE, f)], &[], &[]))? else { return Ok(false) };
        self.faces.push(f);
        self.boxes.push((bx.min(), bx.max()));
        self.shapes.push(shape);
        Ok(true)
    }

    /// `nearest`: the distance from `p` to the nearest face, with the nearest point; infinite
    /// when none answers. A face whose box is farther than the nearest face so far is skipped.
    fn nearest(&self, k: &Kernel, live: &LiveBrepObjects<'_, '_>, p: Point3) -> solidcore::Result<(f64, Option<Point3>)> {
        let (mut best, mut at) = (f64::INFINITY, None);
        for ((&f, (lo, hi)), shape) in self.faces.iter().zip(&self.boxes).zip(&self.shapes) {
            let mut bx = 0.0;
            for (l, (h, x)) in [lo.x, lo.y, lo.z].into_iter().zip([hi.x, hi.y, hi.z].into_iter().zip([p.x, p.y, p.z])) {
                let out = (l - x).max((x - h).max(0.0));
                bx += out * out;
            }
            if f64::sqrt(bx) >= best {
                continue;
            }
            if let Some(c) = answer(face_closest_point_in_shape(k, shape, live, (ONE, f), p))?
                && c.distance.is_finite()
                && c.distance < best
            {
                best = c.distance;
                at = Some(c.point);
            }
        }
        Ok((best, at))
    }
}

/// The live view of `b` the closest-point solves read, under `identity`: `query`'s effective
/// identity ([`BrepQuery::effective_identity`]). A working copy (`copy_of`) has no naming
/// scheme of its own, so `BrepIdentity::of_brep` gives it the arena's identities, which name
/// none of its geometry owners (`copy_of`'s keys, recorded on the copy as proven owners): every
/// face and edge then drops out of a solve (solidcore#1440 was this).
fn live<'a, 'i>(b: &'a mut Brep, identity: &'a BrepIdentity<'i>) -> solidcore::Result<LiveBrepObjects<'a, 'i>> {
    LiveBrepObjects::new(b, identity, None)
}

/// `pick_faces` (`face_distance`: `SmApiFaceClosestPoint`; a face whose query returns a status
/// other than success is skipped, [`answer`]).
pub(super) fn pick_faces(k: &mut Kernel, b: &mut Brep, points: &[[f64; 3]], tol: f64) -> solidcore::Result<std::result::Result<Vec<FaceId>, Miss>> {
    let all = brep_p1::faces(b, None)?;
    let model_tol = b.tolerance();
    let mut shapes = Vec::with_capacity(all.len());
    for &f in &all {
        shapes.push(answer(Shape::new(k, &(ONE, &*b), &[], &[(ONE, f)], &[], &[]))?);
    }
    let query = BrepQuery::of_brep(b);
    let identity = query.effective_identity();
    let objects = live(b, &identity)?;
    let k: &Kernel = k;
    let indices: Vec<usize> = (0..all.len()).collect();
    let picked = pick(&indices, points, tol, model_tol, |i, p| {
        let (Some(&f), Some(Some(shape))) = (all.get(i), shapes.get(i)) else { return Ok(None) };
        Ok(answer(face_closest_point_in_shape(k, shape, &objects, (ONE, f), p))?.map(|c| c.distance))
    })?;
    Ok(picked.map(|(p, _)| p.iter().filter_map(|&i| all.get(i).copied()).collect()))
}

/// A face pick's failure as `pick_error` (ops.rs) reads it.
pub(super) fn face_miss(m: Miss) -> Error {
    match m {
        Miss::None(point) => Error::NoMatch { point },
        Miss::Ambiguous(point) => Error::Ambiguous { point },
    }
}

/// `outer_faceuse`: the faceuse of `f` on the outside of its body, the one bounding the infinite
/// region (for a sheet, both do; then the upward one).
fn outer_faceuse(b: &Brep, f: FaceId) -> solidcore::Result<Option<FaceuseId>> {
    let Some(fu) = face_p2::upward_faceuse(b, f)? else { return Ok(None) };
    let use_ = b.faceuse(fu)?;
    let outside = match use_.shell {
        Some(s) => b.shell(s)?.region.is_some() && b.shell(s)?.region == b.infinite_region(),
        None => false,
    };
    Ok(if outside { Some(fu) } else { use_.mate })
}

/// `leaves_cavity`: whether an inward offset of `wall` (> 0) from every face of the solid `b`
/// but `open` leaves a cavity: a point inside `b` farther than `wall`, and a margin, from every
/// face that is offset. A point counts only as the centre of a ball inside `b`, reached from a
/// face sample by stepping inward along the normal, then from ball to ball. The capi refuses a
/// wall this finds no room for before SMLib sees it (C++ SMLib corrupts memory there,
/// solidcore#380); the same refusal here keeps the C++ path's answer.
fn leaves_cavity(k: &mut Kernel, b: &mut Brep, open: &[FaceId], wall: f64) -> solidcore::Result<bool> {
    let list = brep_p1::faces(b, None)?;
    let (mut all, mut walls) = (FaceSet::new(), FaceSet::new());
    for &f in &list {
        if !all.add(k, b, f)? {
            return Ok(false);
        }
        if !open.contains(&f) && !walls.add(k, b, f)? {
            return Ok(false);
        }
    }
    if walls.faces.is_empty() {
        return Ok(true);
    }
    let tol = b.tolerance();
    let margin = (10.0 * tol).max(1e-3 * wall);
    let need = wall + margin;
    // Face samples: a grid over each offset face's domain, the points inside its trims, each
    // with the inward unit normal there.
    const GRID: usize = 5;
    const MAX_SAMPLES: usize = 2048;
    let mut samples: Vec<(Point3, Vec3)> = Vec::new();
    for &f in &walls.faces {
        let Some(fu) = outer_faceuse(b, f)? else { continue };
        let inward = if b.faceuse(fu)?.orientation == OrientType::Same { -1.0 } else { 1.0 };
        let dom = b.face(f)?.uv_domain;
        for i in 0..GRID {
            for j in 0..GRID {
                if samples.len() >= MAX_SAMPLES {
                    break;
                }
                let uv: Point2 = dom.evaluate((i as f64 + 0.5) / GRID as f64, (j as f64 + 0.5) / GRID as f64);
                if answer(face_classify_uv(k, b, f, uv))? != Some(PointClassType::Face) {
                    continue;
                }
                let Some(surf) = b.face(f)?.surface.as_ref() else { break };
                let (Some(at), Some(n)) = (answer(surf.evaluate_point(k, uv))?, answer(surf.evaluate_normal(k, uv, Side::Left, Side::Left))?) else {
                    continue;
                };
                let len = n.length();
                if !(len > 0.0) || !len.is_finite() {
                    continue;
                }
                samples.push((at, n * (inward / len)));
            }
        }
    }
    let query = BrepQuery::of_brep(b);
    let identity = query.effective_identity();
    let objects = live(b, &identity)?;
    let k: &Kernel = k;
    // A step of `h` inward from a sample ends at a ball's centre when no face is nearer to it
    // than `h` (the step crossed none, and the ball touches only the sample's face).
    let ball = |s: Point3, n: Vec3, h: f64| -> solidcore::Result<(bool, Point3, f64)> {
        let c = s + n * h;
        let (r, _) = all.nearest(k, &objects, c)?;
        Ok((r.is_finite() && r >= h * (1.0 - 1e-9), c, r))
    };
    // First the direct step of `need` from each sample.
    for &(s, n) in &samples {
        let (inside, c, _) = ball(s, n, need)?;
        if inside && walls.nearest(k, &objects, c)?.0 >= need {
            return Ok(true);
        }
    }
    // Then climbing, from a small ball at each sample: step away from the nearest offset face by
    // most of the ball's radius (staying inside the ball, so inside the body) while the distance
    // from the offset faces grows.
    const STEPS: usize = 48;
    for &(s, n) in &samples {
        let (inside, mut c, mut r) = ball(s, n, margin.max(1e3 * tol))?;
        if !inside {
            continue;
        }
        for _ in 0..STEPS {
            let (d, q) = walls.nearest(k, &objects, c)?;
            if d >= need {
                return Ok(true);
            }
            let Some(q) = q.filter(|_| d > 0.0 && d.is_finite()) else { break };
            let next = c + (c - q) * (0.9 * r / d);
            let (rn, _) = all.nearest(k, &objects, next)?;
            if !(rn > 0.0) || !rn.is_finite() || walls.nearest(k, &objects, next)?.0 <= d * (1.0 + 1e-9) {
                break;
            }
            c = next;
            r = rn;
        }
    }
    Ok(false)
}

/// An object name in key domain `domain`: each B-rep the offset executive touches (the source,
/// its working copy, the result, the merge's bodies) names its objects in its own domain, as
/// solidcore's offset fixture does, so their notification records never collide.
fn named(entity: EntityRef, domain: u64) -> ObjectRef {
    let r = brep_p3::topology_ref(0, entity);
    ObjectRef { type_code: r.type_code, key: ObjectKey(domain + r.type_code as u64 * 1_000_000 + (r.key.0 & 0xffff_ffff)) }
}

const SOURCE: u64 = 100_000_000_000;
const RESULT: u64 = 200_000_000_000;
const WORKING: u64 = 600_000_000_000;

fn source_key(e: EntityRef) -> ObjectRef {
    named(e, SOURCE)
}

fn result_key(e: EntityRef) -> ObjectRef {
    named(e, RESULT)
}

fn working_key(e: EntityRef) -> ObjectRef {
    named(e, WORKING)
}

fn curve_in(domain: u64, edge: EdgeId, type_code: i64) -> ObjectRef {
    ObjectRef { key: ObjectKey(domain + u64::from(edge.index())), type_code }
}

fn result_curve(edge: EdgeId, type_code: i64) -> ObjectRef {
    curve_in(300_000_000_000, edge, type_code)
}

fn source_curve(edge: EdgeId, type_code: i64) -> ObjectRef {
    curve_in(400_000_000_000, edge, type_code)
}

fn working_curve(edge: EdgeId, type_code: i64) -> ObjectRef {
    curve_in(650_000_000_000, edge, type_code)
}

fn lists_in(domain: u64, region: RegionId) -> [ObjectRef; 2] {
    [0, 1].map(|list| ObjectRef { key: ObjectKey(domain + u64::from(region.index()) * 2 + list), type_code: 0 })
}

fn source_lists(result: bool, region: RegionId) -> [ObjectRef; 2] {
    lists_in(if result { 510_000_000_000 } else { 500_000_000_000 }, region)
}

fn working_lists(region: RegionId) -> [ObjectRef; 2] {
    lists_in(900_000_000_000, region)
}

fn merge_curve(brep: BrepId, edge: EdgeId, type_code: i64) -> ObjectRef {
    ObjectRef { key: ObjectKey(700_000_000_000 + u64::from(brep.index()) * 1_000_000_000 + u64::from(edge.index())), type_code }
}

/// Identities of the B-reps the merge makes, disjoint from the keys above.
struct MergeBodies;

impl SweepIdentities for MergeBodies {
    fn topology(&self, brep: BrepId, entity: EntityRef) -> ObjectRef {
        named(entity, 1_000_000_000_000 + u64::from(brep.index()) * 100_000_000_000)
    }
    fn owned_auxiliary_objects(&self, brep: BrepId, model: &Brep) -> Vec<ObjectKey> {
        model.edges().iter().filter(|(_, e)| e.curve.is_some()).map(|(edge, _)| merge_curve(brep, edge, 0).key).collect()
    }
    fn curve(&self, brep: BrepId, edge: EdgeId, curve: &Curve) -> ObjectRef {
        merge_curve(brep, edge, curve.type_code())
    }
    fn new_curve(&self, brep: BrepId, edge: EdgeId, type_code: i64) -> solidcore::Result<ObjectRef> {
        Ok(merge_curve(brep, edge, type_code))
    }
    fn source_lists(&self, brep: BrepId, region: RegionId) -> solidcore::Result<[ObjectRef; 2]> {
        Ok(lists_in(800_000_000_000 + u64::from(brep.index()) * 1_000_000_000, region))
    }
}

/// The notification stores of one offset call on `source` (SM_API's context keeps them for the
/// process; each call here has its own).
struct Notifications {
    records: NotifyRecords,
    attributes: AttributeStore,
}

impl Notifications {
    /// Registers `source` (B-rep 0 of the source domain), its objects and the reserved result.
    fn new(source: &mut Brep) -> Notifications {
        source.set_key(source_key(EntityRef::Brep(ONE)).key);
        let mut records = NotifyRecords::new();
        records.insert(source.key(), Topology::new());
        records.insert(result_key(EntityRef::Brep(ONE)).key, Topology::new());
        let entities = source
            .shells()
            .iter()
            .map(|(id, _)| EntityRef::Shell(id))
            .chain(source.faces().iter().map(|(id, _)| EntityRef::Face(id)))
            .chain(source.faceuses().iter().map(|(id, _)| EntityRef::Faceuse(id)))
            .chain(source.edges().iter().map(|(id, _)| EntityRef::Edge(id)))
            .chain(source.edgeuses().iter().map(|(id, _)| EntityRef::Edgeuse(id)))
            .chain(source.vertices().iter().map(|(id, _)| EntityRef::Vertex(id)))
            .chain(source.vertexuses().iter().map(|(id, _)| EntityRef::Vertexuse(id)))
            .chain(source.loopuses().iter().map(|(id, _)| EntityRef::Loopuse(id)))
            .chain(source.regions().iter().map(|(id, _)| EntityRef::Region(id)));
        for e in entities {
            records.insert(source_key(e).key, Topology::new());
        }
        Notifications { records, attributes: AttributeStore::default() }
    }

    fn context(&mut self) -> OffsetNotifications<'_> {
        OffsetNotifications {
            records: &mut self.records,
            attributes: &mut self.attributes,
            original_key_of: &source_key,
            result_key_of: &result_key,
            original_curve_ref: &source_curve,
            result_curve_ref: &result_curve,
            boolean_source_lists: &source_lists,
            result_brep: result_key(EntityRef::Brep(ONE)),
            merge_identities: Some(&MergeBodies),
        }
    }
}

/// `SmApiShellBrepFull(b, distance, TRUE, TRUE, TRUE, open, r)`: convex edges extended, results
/// merged, side walls made at a sheet's edges, and (as the fork does, 5448b9a) a thickened
/// sheet's coplanar side walls one face.
fn shell_brep_full(k: &mut Kernel, mut b: Brep, distance: f64, open: &[FaceId]) -> solidcore::Result<Brep> {
    let mut n = Notifications::new(&mut b);
    let options = ShellOptions {
        offset: OffsetOptions { distance, extend_convex_edges: true, merge_results: true },
        create_offset_solid: true,
        merge_sheet_side_walls: true,
    };
    let faces: Vec<_> = open.iter().map(|&f| Some((ONE, f))).collect();
    let copy = ShellCopyIdentities { key_of: &working_key, curve_ref: &working_curve, boolean_source_lists: &working_lists };
    api_shell_brep_full(k, Some(BrepRef { brep: &b, handle: ONE }), options, &faces, &copy, &mut n.context())
}

/// `SmApiOffsetBrepFull(b, distance, TRUE, TRUE, r)`.
fn offset_brep_full(k: &mut Kernel, b: &mut Brep, distance: f64) -> solidcore::Result<Brep> {
    let mut n = Notifications::new(b);
    let options = OffsetOptions { distance, extend_convex_edges: true, merge_results: true };
    api_offset_brep_full(k, Some(b), options, &mut n.context())
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

/// An SMLib call's own status passed on (`status(...)` in the capi), else the value.
fn called<T>(r: solidcore::Result<T>) -> solidcore::Result<Out<T>> {
    match r {
        Ok(v) => Ok(Ok(v)),
        Err(e) => Ok(Err(status(e))),
    }
}

/// `smlib_shell`: the faces nearest `points` (each within `tolerance`) opened, the rest offset
/// by `thickness` (negative: inward) into a wall; none gives a closed hollow body. An inward
/// wall of a solid that leaves no cavity is [`Error::NoRoom`], refused before the shell.
pub(crate) fn shell(body: &Body, points: &[[f64; 3]], tolerance: f64, thickness: f64) -> Result<Body> {
    if !points_ok(points, true) || !tolerance_ok(tolerance) || !distance_ok(thickness) {
        return Err(Error::Argument);
    }
    on_copy("smlib_shell", body, |k, mut w| {
        let picked = match pick_faces(k, &mut w, points, tolerance)? {
            Ok(p) => p,
            Err(m) => return Ok(Err(face_miss(m))),
        };
        let solid_in = brep_p2::is_manifold_solid(&w)?;
        if thickness < 0.0 && solid_in && !leaves_cavity(k, &mut w, &picked, -thickness)? {
            return Ok(Err(Error::NoRoom));
        }
        match called(shell_brep_full(k, w, thickness, &picked))? {
            Ok(r) => finish(r, solid_in),
            Err(e) => Ok(Err(e)),
        }
    })
}

/// `smlib_offset_faces`: its checks, the pick, the solid check and (moving faces in) the cavity
/// check, then not supported: the face move is the fork's `SmOffsetFacesExecutive` (a96ce9f),
/// which solidcore does not have.
pub(crate) fn offset_faces(body: &Body, points: &[[f64; 3]], tolerance: f64, distance: f64) -> Result<Body> {
    if !points_ok(points, false) || !tolerance_ok(tolerance) || !distance_ok(distance) {
        return Err(Error::Argument);
    }
    on_copy("smlib_offset_faces", body, |k, mut w| {
        let picked = match pick_faces(k, &mut w, points, tolerance)? {
            Ok(p) => p,
            Err(m) => return Ok(Err(face_miss(m))),
        };
        if !brep_p2::is_manifold_solid(&w)? {
            return Ok(Err(Error::Argument));
        }
        // Faces moved in so far that no material is left behind them: the body must reach
        // deeper than the move behind the moved faces.
        if distance < 0.0 {
            let others: Vec<FaceId> = brep_p1::faces(&w, None)?.into_iter().filter(|f| !picked.contains(f)).collect();
            if !leaves_cavity(k, &mut w, &others, -distance)? {
                return Ok(Err(Error::NoRoom));
            }
        }
        // `move_faces`: SmOffsetFacesExecutive::OffsetFaces (fork a96ce9f).
        Ok(Err(Error::Unsupported))
    })
}

/// `smlib_thicken`: a sheet thickened by `thickness` along its normals, or (symmetric) moved back
/// by half first; a closed solid is [`Error::Argument`].
pub(crate) fn thicken(body: &Body, thickness: f64, symmetric: bool) -> Result<Body> {
    if !distance_ok(thickness) {
        return Err(Error::Argument);
    }
    on_copy("smlib_thicken", body, |k, mut w| {
        if brep_p2::is_manifold_solid(&w)? {
            return Ok(Err(Error::Argument));
        }
        if symmetric {
            w = match called(offset_brep_full(k, &mut w, -0.5 * thickness))? {
                Ok(moved) => moved,
                Err(e) => return Ok(Err(e)),
            };
        }
        match called(shell_brep_full(k, w, thickness, &[]))? {
            Ok(r) => finish(r, true),
            Err(e) => Ok(Err(e)),
        }
    })
}

/// `smlib_offset_sheet`: every face of a sheet moved onto its surface's offset by `distance`
/// along its normal; a closed solid is [`Error::Argument`].
pub(crate) fn offset_sheet(body: &Body, distance: f64) -> Result<Body> {
    if !distance_ok(distance) {
        return Err(Error::Argument);
    }
    on_copy("smlib_offset_sheet", body, |k, mut w| {
        if brep_p2::is_manifold_solid(&w)? {
            return Ok(Err(Error::Argument));
        }
        called(offset_brep_full(k, &mut w, distance))
    })
}

/// A unit vector from `v`, as `smlib_draft`'s `unit`; `None` for a vector too short or not
/// finite.
fn unit(v: [f64; 3]) -> Option<Vec3> {
    if !v.iter().all(|x| x.is_finite()) {
        return None;
    }
    let u = p3(v);
    let len = u.length();
    (len > 1e-12 && len.is_finite()).then(|| u * (1.0 / len))
}

/// `smlib_draft`: its checks, the solid check, the pick, then `draft_faces`: each picked face (a
/// plane parallel to the pull) gets a copy of its plane turned by `angle` about its line on the
/// neutral plane, and `SmBrep::LocalOperation` extends or trims the faces around them to meet
/// the tilted planes.
pub(crate) fn draft(
    body: &Body,
    points: &[[f64; 3]],
    tolerance: f64,
    neutral_origin: [f64; 3],
    neutral_normal: [f64; 3],
    pull: [f64; 3],
    angle: f64,
) -> Result<Body> {
    let (Some(normal), Some(along)) = (unit(neutral_normal), unit(pull)) else { return Err(Error::Argument) };
    if !points_ok(points, false)
        || !tolerance_ok(tolerance)
        || !neutral_origin.iter().all(|x| x.is_finite() && x.abs() <= 1e9)
        || !angle.is_finite()
        || angle == 0.0
        || angle.abs() >= 1.5697963267948966
    {
        return Err(Error::Argument);
    }
    on_copy("smlib_draft", body, |k, mut w| {
        if !brep_p2::is_manifold_solid(&w)? {
            return Ok(Err(Error::Argument));
        }
        let picked = match pick_faces(k, &mut w, points, tolerance)? {
            Ok(p) => p,
            Err(m) => return Ok(Err(face_miss(m))),
        };
        // `draft_faces`.
        let mut surfaces = Vec::with_capacity(picked.len());
        for &f in &picked {
            let fu = outer_faceuse(&w, f)?;
            let face = w.face(f)?;
            let (Some(surf), Some(fu)) = (face.surface.as_ref(), fu) else {
                return Ok(Err(Error::Status(INVALID_INPUT)));
            };
            let SurfaceKind::Plane(plane) = surf.kind_with_nurb(k)? else {
                return Ok(Err(Error::Unsupported));
            };
            // The face's outward normal: the plane's, turned with the outer faceuse.
            let mut n = plane.position().z_axis();
            if w.faceuse(fu)?.orientation != OrientType::Same {
                n = -n;
            }
            if n.dot(along).abs() > 1e-6 {
                return Ok(Err(Error::Unsupported));
            }
            let Some(inside) = answer(face_internal_point(k, &mut w, f))? else {
                return Ok(Err(Error::Status(INVALID_INPUT)));
            };
            let toward = along.dot(normal);
            if toward.abs() < 1e-12 {
                return Ok(Err(Error::Argument));
            }
            let hinge = inside + along * ((p3(neutral_origin) - inside).dot(normal) / toward);
            // Turning n about n x pull by `angle` gives n cos(angle) + pull sin(angle).
            let axis = n.cross(along);
            let len = axis.length();
            if !(len > 0.5) {
                return Ok(Err(Error::Unsupported));
            }
            let axis = axis * (1.0 / len);
            let Some(surf) = w.face(f)?.surface.as_ref() else {
                return Ok(Err(Error::Status(INVALID_INPUT)));
            };
            let Some(mut tilted) = answer(surf.copy(k))? else {
                return Ok(Err(Error::Status(INVALID_INPUT)));
            };
            let mut turn = Axis2Placement::IDENTITY;
            turn.rotate_about_axis_at_point(angle, hinge, axis);
            if answer(tilted.transform(k, &turn, None))?.is_none() {
                return Ok(Err(Error::Status(INVALID_INPUT)));
            }
            surfaces.push(tilted);
        }
        // `moved_faces(b, status(b->LocalOperation(faces, surfaces)))`. The capi marks the
        // tilted surfaces first and, when SMLib reports success, answers SMLIB_ERR_NO_EFFECT if
        // no face of the result lies on a marked one. solidcore surfaces carry no attributes and
        // the local operation reports no face history (solidcore#1462), so that check is not
        // made here: a success is the capi's answer whenever SMLib's marks survive.
        let mut notes = Notifications::new(&mut w);
        if let Err(e) = called(local_operation(k, &mut w, &picked, surfaces, &mut notes.context()))? {
            return Ok(Err(e));
        }
        finish(w, true)
    })
}
