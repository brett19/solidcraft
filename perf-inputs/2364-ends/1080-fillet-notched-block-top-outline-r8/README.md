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

Filed by Claude Code for brett19.
