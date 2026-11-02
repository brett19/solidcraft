# test-a_round_next_to_a_round_of_another_radius input (solidcore#2364 check)

A SolidCraft unit test, not a rig case: crates/kernel/src/blend_corner.rs `blend_corner::tests::a_round_next_to_a_round_of_another_radius`.

## Input
`before-blend.smb` (sha256 e79e4d5604ff8d93432777a61d030b71a5b996932caa0000b8e2e3017d8c7acf) is the body passed to the failing blend_call, dump `000006-blend_call-brep.smb`.
Body: the 20 mm box `box_solid((0,0,0),(20,20,20))` with edge a, the edge through (20, 10, 20) (along y), already rounded: `fillet(&box, &[(20,10,20)], 2.0)`.
Call: `fillet(&ra, &[(10, 0, 20)], 3.0)` — kernel `fillet(body, edge points, radius)`, one edge b (along x, through (10, 0, 20)), radius 3, no tangent chain; the round of b ends on the round of a.

Captured by the same blend_call dump hook: a local, uncommitted test change set `solidcraft_smlib::set_dump_dir` just around that call, with PRRIG_DUMP_ALL on, in a worktree of ws/pin-16 5c09cff62 with the rig's h2364/b2364 flags and patches; run by `cargo nextest run --cargo-profile ci -p solidcraft-kernel` with solidcore patched to #2364's head and to its base. **Base and head dumped byte-identical bodies.**

## Results
| build | result |
|---|---|
| base 380cc8218 | pass (8 faces, volume within 0.05 of strips(2,3,0) = 7945.579) |
| #2364 10331dc53 | fail: `assert_eq!(r.face_count(), 8)`: 9 faces |

Filed by Claude Code for brett19.
