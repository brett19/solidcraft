//! Face and edge history on solidcore: a port of the fork's `capi/src/smlib_history.cpp`
//! (`smlib_history_tag`, `smlib_history_read`) onto solidcore's B-rep-owned attribute store
//! ([`Brep::add_attribute`]), which solidcore's operations carry through their edits with SMLib's
//! handlers: Split when a face or edge is cut in two (both children get it), Merge when two become
//! one (the survivor gets it), Copy when an object is copied (a boolean copies the second operand
//! into the first), Coincident where a boolean finds one operand's entity lying on the other's.
//! New topology (a boolean's intersection edges) is made without any.
//!
//! The capi's Tag is one attribute holding a set of (operand, index) origins, with its own Split,
//! Merge and Coincident handlers joining the sets where SMLib's defaults keep only one. solidcore
//! runs SMLib's default handlers alone (no application attribute class can override them), so each
//! origin here is an attribute of its own, its id naming the origin ([`id`]): the default
//! handlers give an object every attribute it has no attribute of that id for, which on a set of
//! distinct ids is the union the capi's handlers make. The face's boundary edges then (`edges`)
//! and the edge's points then (`at`) ride on the origin's attribute.
//!
//! Deviations from the capi, each where its own handlers differ from the default ones:
//! - An object that already stands for something and is copied onto (`SmAttribute::Copy`, not
//!   overridden there) keeps only its own origins in the capi, here takes the copy's too. No
//!   operation copies onto an object it did not just make.
//! - An untagged object a coincidence or merge lands on gets the capi's Tag without `at` (so it
//!   never counts as moved); here the origin's attribute carries its `at` along. A split child
//!   gets the split edge's `at` in both (the capi's Split handler sets it on an untagged child),
//!   so either piece of a split edge can tell where the edge lay.
//!
//! The rest is as there: the same origins, boundary records, samples and classification.

use std::collections::{BTreeMap, BTreeSet, HashMap};

use solidcore::geom::attribute::{Attribute, AttributeValue, Behavior};
use solidcore::kernel::Kernel;
use solidcore::query::api_queries::edge_closest_point;
use solidcore::topology::face::p4 as face4;
use solidcore::topology::{Brep, EdgeId, VertexId, edge};
use solidcore::{EntityRef, Point3, SolidError};
use solidcraft_geom::brep::history::Change;

use super::boolean::copy_of;
use super::{Body, answer, with_body};
use crate::history::{Raw, RawEdge, RawFace};
use crate::{Error, Result};

/// Operand numbers and indices a tag accepts (`TAG_MAX`: keeps every value inside 30 bits).
const TAG_MAX: usize = 1 << 30;

/// The first id of a tag's attributes: above SMLib's own ids and the applications' (all below
/// `SM_AI_USER_3 + 10000`), so a tag is told apart by its id alone.
const TAG_BASE: u64 = 1 << 62;

/// An edge's tag ids set this bit; a face's don't.
const EDGE_BIT: u64 = 1 << 60;

/// One (operand, index) pair a face or edge stands for.
type Origin = (usize, usize);

/// The attribute id of origin `o` on a face or (`edge`) an edge.
fn id(o: Origin, edge: bool) -> u64 {
    TAG_BASE | if edge { EDGE_BIT } else { 0 } | ((o.0 as u64) << 30) | o.1 as u64
}

/// The origin an attribute id names on a face or (`edge`) an edge, if it is a tag's.
fn origin_of(id: u64, edge: bool) -> Option<Origin> {
    if id & !(EDGE_BIT | ((1 << 60) - 1)) != TAG_BASE || (id & EDGE_BIT != 0) != edge {
        return None;
    }
    let mask = (1u64 << 30) - 1;
    Some((usize::try_from((id >> 30) & mask).ok()?, usize::try_from(id & mask).ok()?))
}

/// What one of a face's or edge's tag attributes says: its origin, for a face the boundary edges
/// that origin had, for an edge the points it ran through then.
struct Said {
    origin: Origin,
    edges: Vec<usize>,
    at: Vec<Point3>,
}

/// The tag attributes on `e` (`Tag::on`), each read; none where it has no tag.
fn tags(b: &Brep, e: EntityRef) -> Vec<Said> {
    let edge = matches!(e, EntityRef::Edge(_));
    b.entity_attributes(e)
        .into_iter()
        .filter_map(|(_, a)| {
            let origin = origin_of(a.attribute_id(), edge)?;
            let AttributeValue::Generic { longs, doubles, .. } = a.value() else { return None };
            let edges = longs.iter().filter_map(|x| usize::try_from(*x).ok()).collect();
            let at = doubles.as_chunks::<3>().0.iter().map(|[x, y, z]| Point3::new(*x, *y, *z)).collect();
            Some(Said { origin, edges, at })
        })
        .collect()
}

/// `Tag::set`: replaces whatever tag `e` has with exactly origin `o`, the boundary edges `edges`
/// (a face's) and the points `at` (an edge's).
fn set(b: &mut Brep, e: EntityRef, o: Origin, edges: &[usize], at: &[Point3]) -> std::result::Result<(), SolidError> {
    let edge = matches!(e, EntityRef::Edge(_));
    let old: Vec<_> = b.entity_attributes(e).into_iter().filter(|(_, a)| origin_of(a.attribute_id(), edge).is_some()).map(|(h, _)| h).collect();
    for h in old {
        b.remove_attribute(e, h)?;
    }
    let longs: Vec<i64> = edges.iter().filter_map(|x| i64::try_from(*x).ok()).collect();
    let doubles: Vec<f64> = at.iter().flat_map(|p| [p.x, p.y, p.z]).collect();
    b.add_attribute(e, Attribute::generic(id(o, edge), Behavior::Copy, &longs, &doubles, &[]))?;
    Ok(())
}

/// `samples`: points along `e`, its two ends, then three between, evenly in its parameter. Empty
/// where it can't be evaluated.
fn samples(k: &Kernel, b: &Brep, e: EdgeId) -> std::result::Result<Vec<Point3>, SolidError> {
    let edge = b.edge(e)?;
    let Some(curve) = edge.curve.as_ref() else { return Ok(Vec::new()) };
    let (lo, hi) = (edge.interval.min(), edge.interval.max());
    let mut p = Vec::with_capacity(5);
    for t in [0.0, 1.0, 0.25, 0.5, 0.75] {
        let Some(v) = answer(curve.evaluate(k, lo + t * (hi - lo), 0, true, false))? else { return Ok(Vec::new()) };
        let Some(v) = v.first() else { return Ok(Vec::new()) };
        p.push(Point3::new(v.x, v.y, v.z));
    }
    Ok(p)
}

/// `moved`: have `pieces`, the result edges standing for one edge, moved from where its tag says
/// it lay (`at`, the first piece's record of at least two points)? Together they must run between
/// its ends (the vertices only one piece ends on; none for an edge closed on itself, whose end some
/// piece still ends on), and every point it ran through must lie on one of them (within `tol`).
/// One piece is the edge still whole; more, the edge cut by vertices. Pieces with no record of
/// where the edge lay have not moved.
fn moved(k: &Kernel, b: &Brep, pieces: &[(EdgeId, &[Point3])], tol: f64) -> std::result::Result<bool, SolidError> {
    let Some(at) = pieces.iter().map(|(_, at)| *at).find(|at| at.len() >= 2) else { return Ok(false) };
    let [a, z, ..] = at else { return Ok(false) };
    let near = |p: &Point3, q: &Point3| p.distance(*q) <= tol;
    let mut ends: BTreeMap<VertexId, usize> = BTreeMap::new();
    for (e, _) in pieces {
        let (Some(s), Some(f)) = edge::start_end_vertices(b, *e)? else { return Ok(true) };
        *ends.entry(s).or_default() += 1;
        *ends.entry(f).or_default() += 1;
    }
    let mut open = Vec::new();
    for (v, n) in &ends {
        if n % 2 == 1 {
            open.push(b.vertex(*v)?.point);
        }
    }
    let span = match open.as_slice() {
        [p, q] => (near(p, a) && near(q, z)) || (near(p, z) && near(q, a)),
        [] => {
            let mut on_end = false;
            for v in ends.keys() {
                on_end = on_end || near(&b.vertex(*v)?.point, a);
            }
            near(a, z) && on_end
        }
        _ => false,
    };
    if !span {
        return Ok(true);
    }
    for q in at.iter().skip(2) {
        // `SmApiEdgeClosestPoint` failing counts as off a piece, as a distance above `tol` does.
        let mut on = false;
        for (e, _) in pieces {
            if let Some(c) = answer(edge_closest_point(k, b, *e, *q))?
                && c.closest.distance <= tol
            {
                on = true;
                break;
            }
        }
        if !on {
            return Ok(true);
        }
    }
    Ok(false)
}

/// `smlib_history_tag`: a copy of `body` (`SmApiBrepCopy`) whose face i and edge i stand for face
/// i and edge i of operand `operand` (in its face and edge order, which the copy keeps), a face
/// also recording its boundary edges then, an edge the points it ran through.
pub(crate) fn tag(body: &Body, operand: usize) -> Result<Body> {
    if operand >= TAG_MAX {
        return Err(Error::Argument);
    }
    with_body("smlib_history_tag", body, |k, b| {
        let mut r = copy_of(k, b, 1)?;
        let faces = r.face_list().to_vec();
        let edges = r.edge_list().to_vec();
        if faces.len() >= TAG_MAX || edges.len() >= TAG_MAX {
            return Ok(Err(Error::Argument));
        }
        let edge_at: HashMap<EdgeId, usize> = edges.iter().enumerate().map(|(i, e)| (*e, i)).collect();
        for (i, f) in faces.iter().enumerate() {
            let own: BTreeSet<usize> = face4::edges(&r, *f, None, false, None)?.iter().filter_map(|e| edge_at.get(e).copied()).collect();
            set(&mut r, EntityRef::Face(*f), (operand, i), &own.into_iter().collect::<Vec<_>>(), &[])?;
        }
        for (i, e) in edges.iter().enumerate() {
            let at = samples(k, &r, *e)?;
            set(&mut r, EntityRef::Edge(*e), (operand, i), &[], &at)?;
        }
        Ok(Ok(r))
    })?
    .map(Body::new)
}

/// `smlib_history_read`: each face's and edge's origins in `result`, a face's change from how
/// many origins it has, how many result faces share them and whether its boundary is still
/// exactly its origin's edges; each edge's faces. In the body's own face and edge order.
pub(crate) fn read(result: &Body) -> Result<Raw> {
    with_body("smlib_history_read", result, |k, b| {
        let faces = b.face_list().to_vec();
        let edges = b.edge_list().to_vec();
        let face_at: HashMap<_, usize> = faces.iter().enumerate().map(|(i, f)| (*f, i)).collect();
        let edge_at: HashMap<EdgeId, usize> = edges.iter().enumerate().map(|(i, e)| (*e, i)).collect();
        let tol = b.tolerance();

        // Each face's origins and the boundary edges they had; each edge's origins and points.
        let mut fo: Vec<BTreeSet<Origin>> = Vec::with_capacity(faces.len());
        let mut fe: Vec<BTreeSet<usize>> = Vec::with_capacity(faces.len());
        let mut eo: Vec<BTreeSet<Origin>> = Vec::with_capacity(edges.len());
        let mut at: Vec<Vec<Point3>> = Vec::with_capacity(edges.len());
        let mut face_uses: BTreeMap<Origin, usize> = BTreeMap::new();
        let mut edge_uses: BTreeMap<Origin, usize> = BTreeMap::new();
        for f in &faces {
            let said = tags(b, EntityRef::Face(*f));
            let o: BTreeSet<Origin> = said.iter().map(|s| s.origin).collect();
            for x in &o {
                *face_uses.entry(*x).or_default() += 1;
            }
            fe.push(said.iter().flat_map(|s| s.edges.iter().copied()).collect());
            fo.push(o);
        }
        for e in &edges {
            let said = tags(b, EntityRef::Edge(*e));
            let o: BTreeSet<Origin> = said.iter().map(|s| s.origin).collect();
            for x in &o {
                *edge_uses.entry(*x).or_default() += 1;
            }
            // The capi's Tag has one `at`; an edge with one origin has one attribute.
            at.push(said.into_iter().next().map(|s| s.at).unwrap_or_default());
            eo.push(o);
        }

        let mut out_faces = Vec::with_capacity(faces.len());
        for (i, f) in faces.iter().enumerate() {
            let (Some(o), Some(own)) = (fo.get(i), fe.get(i)) else { continue };
            let change = match o.iter().next() {
                None => Change::Generated,
                Some(_) if o.len() > 1 => Change::Merged,
                Some(first) if face_uses.get(first).copied().unwrap_or(0) > 1 => Change::Split,
                Some(first) => {
                    // Kept when its boundary is still exactly where the edges it had were: each
                    // boundary edge one of its own operand's, standing for one edge only, every
                    // piece of that edge on this face (an edge only cut by a vertex, as splitting
                    // the face beside it cuts it, is still all there), the pieces where it lay (an
                    // edge an operation moved, as a draft moves a tilted face's neighbours' edges,
                    // leaves those faces trimmed differently), and those edges exactly the ones it
                    // had.
                    let mut pieces: BTreeMap<usize, Vec<(EdgeId, &[Point3])>> = BTreeMap::new();
                    let mut whole = true;
                    for e in face4::edges(b, *f, None, false, None)? {
                        let Some(j) = edge_at.get(&e).copied() else {
                            whole = false;
                            break;
                        };
                        let (Some(origins), Some(points)) = (eo.get(j), at.get(j)) else {
                            whole = false;
                            break;
                        };
                        let only = match origins.iter().next() {
                            Some(x) if origins.len() == 1 => *x,
                            _ => {
                                whole = false;
                                break;
                            }
                        };
                        whole = only.0 == first.0;
                        if !whole {
                            break;
                        }
                        let p = pieces.entry(only.1).or_default();
                        if !p.iter().any(|(x, _)| *x == e) {
                            p.push((e, points.as_slice()));
                        }
                    }
                    let mut now = BTreeSet::new();
                    for (edge, p) in &pieces {
                        whole = whole && p.len() == edge_uses.get(&(first.0, *edge)).copied().unwrap_or(0) && !moved(k, b, p, tol)?;
                        now.insert(*edge);
                    }
                    if whole && now == *own { Change::Kept } else { Change::Modified }
                }
            };
            out_faces.push(RawFace { change, origins: o.iter().copied().collect() });
        }

        let mut out_edges = Vec::with_capacity(edges.len());
        for (i, e) in edges.iter().enumerate() {
            let bounds: BTreeSet<usize> = edge::faces(b, *e, None)?.iter().filter_map(|f| face_at.get(f).copied()).collect();
            let origins = eo.get(i).map(|o| o.iter().copied().collect()).unwrap_or_default();
            out_edges.push(RawEdge { faces: bounds.into_iter().collect(), origins });
        }
        Ok(Raw { faces: out_faces, edges: out_edges })
    })
}
