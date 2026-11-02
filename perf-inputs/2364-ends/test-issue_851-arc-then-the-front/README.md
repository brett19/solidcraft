# test-issue_851-arc-then-the-front input (solidcore#2364 check)

A SolidCraft unit test, not a rig case: crates/kernel/src/issue_851_tests.rs `issue_851_tests::blend_the_arc_top_edge_and_the_top_front_edge`, the fillet pass's "arc, then the front".

## Input
`before-blend.smb` (sha256 a347c4fef9225d0774fde6e7730031a3a74f3cfe6c0cc93908ce84c3137658d2) is the body passed to the failing blend_call, dump `000006-blend_call-brep.smb`.
Body: `part()` (a 41⅙ × 21 × 10 plate plus the corner a circle of radius 21 leaves; TIP_X = 62⅙, R_WALL = 21, HEIGHT = 10) with the arc's top edge already rounded: `fillet(&part, &[arc_top()], 2.0)`, arc_top() = (TIP_X − 21·cos45°, −21 + 21·sin45°, 10) ≈ (47.31747, −6.15076, 10).
Call: `fillet(&a, &[tf()], 2.0)` with tf() = (20, −21, 10), the front's top edge, radius 2, no tangent chain.

Captured by the same blend_call dump hook: a local, uncommitted test change set `solidcraft_smlib::set_dump_dir` just around that call, with PRRIG_DUMP_ALL on, in a worktree of ws/pin-16 5c09cff62 with the rig's h2364/b2364 flags and patches; run by `cargo nextest run --cargo-profile ci -p solidcraft-kernel` with solidcore patched to #2364's head and to its base. **Base and head dumped byte-identical bodies.**

## Results
| build | result |
|---|---|
| base 380cc8218 | pass (removed within 0.06 of 60.12201493442991) |
| #2364 10331dc53 | fail: removed 59.147564123979464, expected 60.12201493442991 |

Filed by Claude Code for brett19.
