# solidcore#2364 (head 10331dc53) ends: blend inputs

Cases where #2364 changes the result against base 380cc8218: six rig fillet cases now leave a g0 seam (continuity_order fails) and two SolidCraft kernel unit tests fail. Each folder holds the body passed to the blend call that differs, with the exact call in its README. Captured with the PRRIG_DUMP_ALL switch (every call's inputs), by both the head and the base build.

| case | input | sha256 (first 16) | base and head byte-identical |
|---|---|---|---|
| 5158-fillet-adjacent-plate-edges-as-thick-as-plate | `5158-fillet-adjacent-plate-edges-as-thick-as-plate/before-step10.smb` | 41dddfd15f04305d | yes, base = head |
| 1080-fillet-notched-block-top-outline-r8 | `1080-fillet-notched-block-top-outline-r8/before-step3.smb` | cd69b91c2f8799f8 | yes, base = head |
| 3369-fillet-edge-ends-on-tangent-seam | `3369-fillet-edge-ends-on-tangent-seam/before-step9.smb` | 9659b87d334c66f2 | yes, base = head |
| 1207-fillet-box-end-edge-into-unblended-round-partial-chain | `1207-fillet-box-end-edge-into-unblended-round-partial-chain/before-step3.smb` | d2e555c6738fc344 | yes, base = head |
| every-block-cylinder-tangent-concave-arc-r2-partial-chain | `every-block-cylinder-tangent-concave-arc-r2-partial-chain/before-step3.smb` | 1c77bafbd407cbfc | yes, base = head |
| every-fillet-after-fillet-curvature-front-then-side-r3-partial-chain | `every-fillet-after-fillet-curvature-front-then-side-r3-partial-chain/before-step3.smb` | 6163690b70c22e49 | yes, base = head |
| test-a_round_next_to_a_round_of_another_radius | `test-a_round_next_to_a_round_of_another_radius/before-blend.smb` | e79e4d5604ff8d93 | yes, base = head |
| test-issue_851-arc-then-the-front | `test-issue_851-arc-then-the-front/before-blend.smb` | a347c4fef9225d07 | yes, base = head |

Filed by Claude Code for brett19.
