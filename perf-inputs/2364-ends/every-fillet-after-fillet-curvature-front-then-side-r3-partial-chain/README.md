# every-fillet-after-fillet-curvature-front-then-side-r3-partial-chain input (solidcore#2364 check)

Case: SolidCraft `tests/kernel-cases/fillets/every-fillet-after-fillet-curvature-front-then-side-r3-partial-chain` (recipe.json and expected.json copied here, from the rig's worktree).

## Input
`before-step3.smb` (sha256 6163690b70c22e491486363cda39b1cf5bb81ce45c63494f0bf0647d2516dbbd) is the body passed to recipe step 3 (index 2), dump `000017-blend_call-brep.smb`:
`solid.fillet {"edges": [[40, 15, 20]], "radius": 3, "type": "curvature", "tangent_chain": false}`.
Edges are given as points on them; the blend selects the edge through each point.

Made by the binaries `solidcraft-cli-h2364d` (head, solidcore #2364 10331dc53) and `solidcraft-cli-b2364d` (base 380cc8218): the PR-check rig's h2364/b2364 builds (SolidCraft ws/pin-16 5c09cff62, #2365's flags, the converter arms, fillet_blend_corner) plus the local, uncommitted PRRIG_DUMP_ALL switch (see ../../2307-peg-chamfer/README.md), since the rig's dump hook writes a call's inputs only when it fails. Run with `SOLIDCRAFT_KERNEL=solidcore PRRIG_DUMP_ALL=1 solidcraft-cli kernel-cases --json --dumps=<dir> <case>`. **Base and head dumped byte-identical bodies**, so the input doesn't depend on the head.

The base's blend_call on this input refuses (status 1001, the tangent-propagation hint), which the case expects; the head blends and leaves a g0 seam.

## Results (rig, solidcore kernel; the seam is the continuity_order check's worst edge)
| build | status | volume mm³ | faces | seam / message |
|---|---|---|---|---|
| base 380cc8218 | pass | None | None | step 3 (solid.fillet): SMLib G2 fillet: operation: SMLib failed (status 1001) (a picked edge continues tangentially into the edge at (40.000, 0.906, 19.094), which is not blended with it, and the blend cannot end there:  |
| #2364 10331dc53 | fail | 23870.59509178037 | 9 | g0 seam: edge 5 (`F1:+x\|F3:blend:F1:+x\|F1:+z#[F2:blend:F1:+z\|F1:-y]`) at [40.0, 2.9999999999999996, 20.0], normal angle 90.0° |

Filed by Claude Code for brett19.
