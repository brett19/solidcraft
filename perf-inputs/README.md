# Fillet perf inputs

Two bodies from SolidCraft's kernel-case rig, saved just before their fillet, for timing SMLib's
fillet on its own. Each `.smb` is SMLib's own binary save (`smlib_brep_write`, ascii = false) of
the body SolidCraft hands its fillet, made with the SMLib kernel at rev
`5e8c9c19bb2cff6b5ae321ecda577496955fe037` (`5e8c9c1`, `crates/smlib/SMLIB_REV`; SolidCraft
`fork/smlib` b7e83cd7c).

| File | Bytes | SHA-256 |
|---|---|---|
| `4143-notch-rim-plate.smb` | 48070 | `e206751af6c8e371cc0872562dbbdb183beee7540482705c2d1a67aef7c2a8f3` |
| `1213-bent-tube.smb` | 5898 | `12f39f05e1f7a43bffe28ac7406ce8a163dd7bf5e1cf4ee56479eed527655852` |

## How SolidCraft calls the fillet

`crates/kernel/src/blend_smlib.rs` (`blend_dropping` and `run_with`): the body goes into SMLib
(saved here as is), is tagged as operand 0 for history (`smlib_history_tag`; not applied to these
files), and gets one call, `Brep::blend_dropping(Blend::Round, points, radii, tolerance)`. `points`
is the midpoint of each picked edge, `tolerance` is `0.05 * body size + 0.001` (the pick
tolerance). The calls below are the ones SolidCraft makes.

## Case A: 4143-fillet-notch-rim-on-bspline-ramp

- Recipe: `tests/kernel-cases/fillets/4143-fillet-notch-rim-on-bspline-ramp/recipe.json`
  (an extruded plate with a cubic B-spline ramp, and an R5 half-round notch from the `SlotCut`).
- Body: "Plate" after `SlotCut`. 13 faces, 33 edges, 22 vertices. Volume 27603.900411785664.
- Fillet: R4, recipe pick (0, 45, 5.72).
- Edge: an open B-spline edge between the ramp (face 1, B-spline surface) and the notch
  (face 11, cylinder). It runs from (5, 50, 7.737457924122623) to (-5, 50, 7.737457924122619).
- SMLib call: point (0.00001182624777918979, 45.000052048335505, 5.715995280617577),
  radius 4, tolerance 5.709984147814741.
- Expected: the rig expects pass, meaning one closed valid solid with G1 continuity. There is
  no closed form for the volume. The call above on the read-back file gave a valid solid with
  14 faces, 36 edges and 24 vertices, volume 27517.05701396358, nothing dropped, in 49.7 s.

## Case B: 1213-round-end-circle-of-bent-tube

- Recipe: `tests/kernel-cases/fillets/1213-round-end-circle-of-bent-tube/recipe.json`
  (five R20 circles lofted along a 60-degree arc of radius 100).
- Body: "Tube" after the loft `Bent`. 3 faces, 3 edges, 2 vertices. Volume 131601.59750075627.
- Fillet: R3, recipe pick (0, -14.1421, 14.1421).
- Edge: the closed start circle in the plane x = 0, with its vertex at (0, 0, 20). It is a
  rational B-spline curve between the loft side (face 0, rational B-spline surface) and the
  start cap (face 1, plane).
- SMLib call: point (0, 1.877783516794525e-14, -19.999999999999996), the arc-length midpoint
  of the closed edge. Radius 3, tolerance 6.856654600401045.
- Expected: volume 131360.14839866068 (analytic, rel tol 5e-4). The call above on the
  read-back file gave a valid solid with 4 faces, 5 edges and 3 vertices, volume
  131367.93494174248 (rel diff 5.9e-5), nothing dropped, in 31.2 s.

Face and edge numbers are in SolidCraft's record order, which can differ from SMLib's order in
the file. Use the points.

## How the files were made

`apps/solidcraft-cli/examples/fillet_input_smb.rs` (this branch; `solidcraft-smlib` is
a dev-dependency of `solidcraft-cli` so it can read the file back). It runs the recipe
without its last command (the `solid.fillet`), takes the fillet's body, resolves the pick with
`kernel::blend_edges` as the blend does, and writes `kernel::smlib_body(body)` with
`Brep::write(path, false)`. It then reads the file with `Brep::read` and runs the same
`blend_dropping` call on it. The times above come from that run: a release build with debuginfo,
x86_64 Linux, one run each.

```
cargo build --release -p solidcraft-cli --example fillet_input_smb
target/release/examples/fillet_input_smb tests/kernel-cases/fillets/4143-fillet-notch-rim-on-bspline-ramp perf-inputs/4143-notch-rim-plate.smb
target/release/examples/fillet_input_smb tests/kernel-cases/fillets/1213-round-end-circle-of-bent-tube perf-inputs/1213-bent-tube.smb
```
