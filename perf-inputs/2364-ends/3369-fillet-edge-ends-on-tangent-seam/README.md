# 3369-fillet-edge-ends-on-tangent-seam input (solidcore#2364 check)

Case: SolidCraft `tests/kernel-cases/fillets/3369-fillet-edge-ends-on-tangent-seam` (recipe.json and expected.json copied here, from the rig's worktree).

## Input
`before-step9.smb` (sha256 9659b87d334c66f2e30b8a50fc64574cfad91e2559a3bdf4f049470ed3852c06) is the body passed to recipe step 9 (index 8), dump `000029-blend_call-brep.smb`:
`solid.fillet {"edges": [[6.5, 0, 9.5]], "radius": 2}`.
Edges are given as points on them; the blend selects the edge through each point.

Made by the binaries `solidcraft-cli-h2364d` (head, solidcore #2364 10331dc53) and `solidcraft-cli-b2364d` (base 380cc8218): the PR-check rig's h2364/b2364 builds (SolidCraft ws/pin-16 5c09cff62, #2365's flags, the converter arms, fillet_blend_corner) plus the local, uncommitted PRRIG_DUMP_ALL switch (see ../../2307-peg-chamfer/README.md), since the rig's dump hook writes a call's inputs only when it fails. Run with `SOLIDCRAFT_KERNEL=solidcore PRRIG_DUMP_ALL=1 solidcraft-cli kernel-cases --json --dumps=<dir> <case>`. **Base and head dumped byte-identical bodies**, so the input doesn't depend on the head.

## Results (rig, solidcore kernel; the seam is the continuity_order check's worst edge)
| build | status | volume mm³ | faces | seam / message |
|---|---|---|---|---|
| base 380cc8218 | pass | 9763.665662831354 | 11 |  |
| #2364 10331dc53 | fail | 9763.665662831354 | 11 | g0 seam: edge 5 (`F2:side:l2\|F4:blend:F2:side:l2\|F3:+y#[F3:+x]`) at [9.500000000000004, 0.0, 9.5], normal angle 90.0° |

Filed by Claude Code for brett19.
