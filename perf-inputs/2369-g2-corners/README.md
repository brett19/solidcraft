# solidcore #2369: G2 corners refused under the setback path

The inputs for four SolidCraft kernel cases. With `fillet_blend_corner` (WR-616) on, #2369's head 6c87a7323 refuses each of them as "a G2 corner is set back". On solidcore main, and on the same build with WR-616 off, they build.

Each folder has:
- `before-fillet.smb`: the body passed to `blend_call`, dumped by SolidCraft's rig on #2369's head, so it is exactly the input that got refused. The pre-fillet body doesn't depend on #2369.
- `recipe.json`: SolidCraft's command list. The last `solid.fillet` gives the edge picks, as points on each edge, plus the radius and type. `type: curvature` is the Gⁿ section round (G2).
- `expected.json`: SolidCraft's expected result.

| Case | Fillet | main / WR-616 off | #2369 |
|---|---|---|---|
| report-fillet-curvature-three-edges-at-bar-corner-solver-residues-outside-setback | 3 edges at a bar's top corner, r 2, curvature, tangent_chain | **passes**: 9526.705049, 13 faces (expected 9526.705048390) | refused |
| report-fillet-curvature-three-edges-at-bar-corner | same corner, exact sketch | 9362.2067, 11 faces; fails only a G1 seam check (expected 9361.3087 ±0.29%) | refused |
| report-fillet-curvature-three-edges-at-bar-corner-solver-residues | same corner, with the sketch solver's residues | 9546.8774, 11 faces (expected 9545.9794 ±0.28%) | refused |
| cov-fillet-g2-pocket-corner | a pocket's corner: 2 floor edges and the upright, all concave, r 2, curvature | 21046.3144, 15 faces; G1 at one seam | refused |

Edge picks:
- bar corner: (0, 0, 4), (20, 0, 10), (0, 8, 10)
- solver-residue variants: (0.48929, 0.0579, 4), (21.53, 0.0579, 10), (0.48929, 8.058, 10)
- pocket: (30, 10, 5), (15, 20, 5), (15, 10, 7.5)

Rig flags: every SolidCraft pin-16 flag on, plus `fillet_round_as_definition`, and #2369 stacked on #2274 941770b26.
