# every-block-cylinder-tangent-concave-arc-r2-partial-chain input (solidcore#2364 check)

Case: SolidCraft `tests/kernel-cases/fillets/every-block-cylinder-tangent-concave-arc-r2-partial-chain` (recipe.json and expected.json copied here, from the rig's worktree).

## Input
`before-step3.smb` (sha256 1c77bafbd407cbfc6ff3d77f0a20b0231c1c20252a0efcefbe5c2782b2ba3475) is the body passed to recipe step 3 (index 2), dump `000025-blend_call-brep.smb`:
`solid.fillet {"edges": [[10, 0, 15]], "radius": 2, "tangent_chain": false}`.
Edges are given as points on them; the blend selects the edge through each point.

Made by the binaries `solidcraft-cli-h2364d` (head, solidcore #2364 10331dc53) and `solidcraft-cli-b2364d` (base 380cc8218): the PR-check rig's h2364/b2364 builds (SolidCraft ws/pin-16 5c09cff62, #2365's flags, the converter arms, fillet_blend_corner) plus the local, uncommitted PRRIG_DUMP_ALL switch (see ../../2307-peg-chamfer/README.md), since the rig's dump hook writes a call's inputs only when it fails. Run with `SOLIDCRAFT_KERNEL=solidcore PRRIG_DUMP_ALL=1 solidcraft-cli kernel-cases --json --dumps=<dir> <case>`. **Base and head dumped byte-identical bodies**, so the input doesn't depend on the head.

The base's blend_call on this input refuses (status 1001, the tangent-propagation hint), which the case expects; the head blends and leaves a g0 seam.

## Results (rig, solidcore kernel; the seam is the continuity_order check's worst edge)
| build | status | volume mm³ | faces | seam / message |
|---|---|---|---|---|
| base 380cc8218 | pass | None | None | step 3 (solid.fillet): SMLib fillet: operation: SMLib failed (status 1001) (a picked edge continues tangentially into the edge at (4.734, 8.808, 15.000), which is not blended with it, and the blend cannot end there: with |
| #2364 10331dc53 | fail | 19085.7707448594 | 9 | g0 seam: edge 14 (`F2:side\|F3:blend:F1:+z\|F2:side~1`) at [8.17868193157977, 5.754056122601736, 15.0625], normal angle 90.0° |

Filed by Claude Code for brett19.
