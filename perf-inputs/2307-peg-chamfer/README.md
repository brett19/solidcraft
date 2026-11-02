# report-peg-front-chamfer-d1-then-concave-chamfer-d2 inputs (solidcore#2307 check)

Case: SolidCraft `tests/kernel-cases/fillets/report-peg-front-chamfer-d1-then-concave-chamfer-d2`
(recipe.json and expected.json copied here). SolidCraft ws/pin-16 5c09cff62, every pin-16 fix flag on.

## Inputs (binary .smb, the rig's blend_call dump hook)
- `before-chamfer1-front-top-edge-d1.smb` (sha256 f3bff138...cc6d): the body passed to recipe step 13,
  `solid.chamfer {"edges": [[20, 0, 10]], "distance": 1}` (front top edge, d1).
- `before-chamfer2-concave-arc-d2.smb` (sha256 38eedc13...63ab): the body passed to step 14,
  `solid.chamfer {"edges": [[32, 8, 10]], "distance": 2}` (concave arc, d2).

Made by the BASE binary (solidcore main c002debb5): the rig's dump hook writes a call's inputs only when
it fails, so a local build added an env switch (PRRIG_DUMP_ALL, not committed) that dumps every call;
the inputs are the `blend_call` dumps 000025 and 000034 (`dumps-all/` has every call's input).
The pre-chamfer bodies don't depend on the head (nothing before step 13 is a blend #2307 touches).

## Final results (whole recipe)
| build | status | volume mm³ | faces | edges | vertices |
|---|---|---|---|---|---|
| base c002debb5 | pass | 9432.066068032667 | 9 | 21 | 14 |
| #2307 08d16ae37 | pass | 9432.066057580232 | 9 | 21 | 14 |

Δ = −1.045e-5 mm³ (base → #2307). The case is graded by invariants (bbox, abs 4e-5); both pass.
