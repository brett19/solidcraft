# 5158-fillet-adjacent-plate-edges-as-thick-as-plate input (solidcore#2364 check)

Case: SolidCraft `tests/kernel-cases/fillets/5158-fillet-adjacent-plate-edges-as-thick-as-plate` (recipe.json and expected.json copied here, from the rig's worktree).

## Input
`before-step10.smb` (sha256 41dddfd15f04305d784883ded95cef795169225c74e8b62dd99c65304a5d191b) is the body passed to recipe step 10 (index 9), dump `000031-blend_call-brep.smb`:
`solid.fillet {"edges": [[0, 15.5, 2], [-15, 0, 2]], "radius": 2}`.
Edges are given as points on them; the blend selects the edge through each point.

Made by the binaries `solidcraft-cli-h2364d` (head, solidcore #2364 10331dc53) and `solidcraft-cli-b2364d` (base 380cc8218): the PR-check rig's h2364/b2364 builds (SolidCraft ws/pin-16 5c09cff62, #2365's flags, the converter arms, fillet_blend_corner) plus the local, uncommitted PRRIG_DUMP_ALL switch (see ../../2307-peg-chamfer/README.md), since the rig's dump hook writes a call's inputs only when it fails. Run with `SOLIDCRAFT_KERNEL=solidcore PRRIG_DUMP_ALL=1 solidcraft-cli kernel-cases --json --dumps=<dir> <case>`. **Base and head dumped byte-identical bodies**, so the input doesn't depend on the head.

## Results (rig, solidcore kernel; the seam is the continuity_order check's worst edge)
| build | status | volume mm³ | faces | seam / message |
|---|---|---|---|---|
| base 380cc8218 | pass | 1681.903764966137 | 8 |  |
| #2364 10331dc53 | fail | 1682.8536106106817 | 10 | g0 seam: edge 1 (`F2:side:l3\|F5:blend:F2:side:l3\|F2:top#[F2:side:l3]`) at [-13.0, 15.5, 0.0], normal angle 90.0° |

Filed by Claude Code for brett19.
