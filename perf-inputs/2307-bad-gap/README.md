# solidcore#2307 Bad Gap repro (SolidCraft fillet_tabbed_plate_outer_loop_and_partial_selection_after_hole)

Test: SolidCraft `crates/kernel/src/tests.rs`, `fillet_tabbed_plate_outer_loop_and_partial_selection_after_hole`
(fn at :1773), the fillet call at :1823 (first loop, mask 0b1111100: the tab's run).
SolidCraft tree: ws/pin-16 5c09cff62, every pin-16 fix flag on (fillet_blend_corner, WR-616, among them).

## Input
`input-plate-hole-rim-r0.5.smb` (binary .smb, `solidcraft_smlib::Brep::write(path, false)`; sha256
dd23638ac8a25c6da2c7f3a084f1a904b7805eec9cbe3786c51caa0c9f9b7320): the tabbed plate (R25 disc with a
tab, extruded 0..2 on XY), its hole r1.5 at (0, 262/9) with the hole's top rim already rounded r0.5.
Input volume 4040.367787504, 10 faces. Written identically (byte for byte) by the base and the head build.

## The call
Round, constant radius **r = 0.5**, edges picked by these points on them (z = 2, the top face's rim):
- (3.999999997492012, 27.894518234675537, 2)
- (3.4142135598750105, 32.525324672801695, 2)
- (-2.480541994032137e-9, 33.11111111046627, 2)
- (-3.4142135648720653, 32.52532467287703, 2)
- (-4.0000000025087745, 27.89451823429679, 2)

(The tab's run: its two sides, its two R2 corners and its top, tangent through the corners, ending at
the sharp concave corners at x = ±4.)

## Results
- base, solidcore main c002debb5: builds, volume 4039.146522146, 15 faces, check_brep clean.
- head, solidcore #2307 08d16ae37: SolidCraft's check refuses the result:
  `SmBrep::Test #4 Bad Gap: Edge/Surface gap across Edgeuse larger than tol (offset=4.1077428e-05, tolerance= 4e-05) [owner SmEdgeuse]`.
  No volume (the result is refused before it is measured).

Logs: run-base-c002debb5.log, run-head-08d16ae37.log (local #[ignore] test `zz_prrig_bad_gap_repro`, not committed).
