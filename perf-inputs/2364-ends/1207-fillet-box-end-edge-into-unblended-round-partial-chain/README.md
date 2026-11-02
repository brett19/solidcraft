# 1207-fillet-box-end-edge-into-unblended-round-partial-chain input (solidcore#2364 check)

Case: SolidCraft `tests/kernel-cases/fillets/1207-fillet-box-end-edge-into-unblended-round-partial-chain` (recipe.json and expected.json copied here, from the rig's worktree).

## Input
`before-step3.smb` (sha256 d2e555c6738fc3442a92ec2d145c91b2dc06562b5a21606e4c1c8903ad1bcb36) is the body passed to recipe step 3 (index 2), dump `000019-blend_call-brep.smb`:
`solid.fillet {"edges": [[0, 8, 20]], "radius": 1, "tangent_chain": false}`.
Edges are given as points on them; the blend selects the edge through each point.

Made by the binaries `solidcraft-cli-h2364d` (head, solidcore #2364 10331dc53) and `solidcraft-cli-b2364d` (base 380cc8218): the PR-check rig's h2364/b2364 builds (SolidCraft ws/pin-16 5c09cff62, #2365's flags, the converter arms, fillet_blend_corner) plus the local, uncommitted PRRIG_DUMP_ALL switch (see ../../2307-peg-chamfer/README.md), since the rig's dump hook writes a call's inputs only when it fails. Run with `SOLIDCRAFT_KERNEL=solidcore PRRIG_DUMP_ALL=1 solidcraft-cli kernel-cases --json --dumps=<dir> <case>`. **Base and head dumped byte-identical bodies**, so the input doesn't depend on the head.

## Results (rig, solidcore kernel; the seam is the continuity_order check's worst edge)
| build | status | volume mm³ | faces | seam / message |
|---|---|---|---|---|
| base 380cc8218 | pass | 7957.505245494063 | 8 |  |
| #2364 10331dc53 | fail | 7957.723438212544 | 9 | g0 seam: edge 17 (`F1:-x\|F3:blend:F1:+z\|F1:-x#[F2:blend:F1:+y\|F1:+z]`) at [0.0, 17.0, 19.999999999999986], normal angle 90.0° |

Filed by Claude Code for brett19.
