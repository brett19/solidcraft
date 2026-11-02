# 1080-fillet-notched-block-top-outline-r8 input (solidcore#2364 check)

Case: SolidCraft `tests/kernel-cases/fillets/1080-fillet-notched-block-top-outline-r8` (recipe.json and expected.json copied here, from the rig's worktree).

## Input
`before-step3.smb` (sha256 cd69b91c2f8799f8e9ba5006bf9129c03b52e0285dcd4a0c5fd9447f07dda7ce) is the body passed to recipe step 3 (index 2), dump `000041-blend_call-brep.smb`:
`solid.fillet {"edges": [[20, 0, 10], [40, 15, 10], [20, 30, 10], [0, 15, 10]], "radius": 8, "tangent_chain": true}`.
Edges are given as points on them; the blend selects the edge through each point.

Made by the binaries `solidcraft-cli-h2364d` (head, solidcore #2364 10331dc53) and `solidcraft-cli-b2364d` (base 380cc8218): the PR-check rig's h2364/b2364 builds (SolidCraft ws/pin-16 5c09cff62, #2365's flags, the converter arms, fillet_blend_corner) plus the local, uncommitted PRRIG_DUMP_ALL switch (see ../../2307-peg-chamfer/README.md), since the rig's dump hook writes a call's inputs only when it fails. Run with `SOLIDCRAFT_KERNEL=solidcore PRRIG_DUMP_ALL=1 solidcraft-cli kernel-cases --json --dumps=<dir> <case>`. **Base and head dumped byte-identical bodies**, so the input doesn't depend on the head.

Note: the rig used the ws/pin-16 recipe (copied here, `tangent_chain: true`); solidcraft-solidcore's copy of this recipe has `tangent_chain: false`.

## Results (rig, solidcore kernel; the seam is the continuity_order check's worst edge)
| build | status | volume mm³ | faces | seam / message |
|---|---|---|---|---|
| base 380cc8218 | pass | 10198.718249145513 | 13 |  |
| #2364 10331dc53 | fail | 10320.299091699917 | 15 | g0 seam: edge 5 (`F3:blend:F1:+z\|F1:-x\|F3:blend:F1:+z\|F1:-x#[F1:+z,F1:-y,F2:+z,F2:-x,F3:blend:F1:+x\|F1:+z]`) at [0.4999999999999929, 7.999999999999996, 10.0], normal angle 90.0° |

## Follow-up: the rig's flags, the edges handed to the blend, and #2364's new head
**UpstreamFixes.** These are printed by an uncommitted debug line at the end of `crates/smlib/src/core.rs` `fixes()`, in a rebuilt h2364 (same flags and patches, dump-all). The full Debug output is in `../fixes-h2364.txt` (old head 10331dc53) and `../fixes-h2364-3dcc89a9f.txt` (new head). There are 313 flags and 308 are true. These 5 are false: `cone_unbounded_parameterized_as_nurb`, `ellipse_open_arc_keeps_end_angle`, `fillet_round_on_round_as_definition`, `fillet_overflow_crease_along_edge`, `fillet_refuse_rounds_breaking_through`. Rig-specific settings that are true: `fillet_keep_features`, `fillet_round_as_definition`, `fillet_blend_corner`. `fillet_extended_rail_end_at_side_edge_crossing` is removed. The rig also has the wrapper-setback and converter-arms patches. On 3dcc89a9f the only difference is the new field `shell_one_construction: false`.

**Arguments to blend_call.** These are printed by an uncommitted eprintln at the top of `crates/smlib/src/core/blend.rs` `blend_call`:
`kind=Round points=[[20.0, 0.0, 10.0], [40.0, 15.0, 10.0], [20.0, 30.0, 10.0], [0.0, 15.0, 10.0]] sizes=[8.0, 8.0, 8.0, 8.0] tol=2.5505097567963926 corners=[] setbacks=[]`
SolidCraft does no chain expansion before this call: the points are exactly the recipe's picks (`tangent_chain: true` adds no points here). Any tangent propagation happens inside solidcore. Note the pick tolerance that SolidCraft passes (`tol` above, the body-size-based value), which is much larger than 1e-6. The dumped input is byte-identical to `before-step*.smb` here.

**Rerun on #2364's new head 3dcc89a9f.** It was built like h2364d (same flags and patches, fillet_blend_corner on, dump-all), with `SOLIDCRAFT_KERNEL=solidcore`. The case still reproduces:
| head | status | volume mm³ | faces | seam |
|---|---|---|---|---|
| 10331dc53 (rebuilt) | fail | 10320.299091699917 | 15 | edge 5 at [0.5, 8.0, 10.0], 90° (with_blends) |
| 3dcc89a9f | fail | 10320.299091699917 | 15 | edge 5 at [0.5, 8.0, 10.0], 90° (with_blends) |

The new head still reproduces, so the edge-list-only rerun on the old head wasn't needed. The edge list is the same as the swarm's 4 (or 2) picks in any case.

Filed by Claude Code for brett19.
