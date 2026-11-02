# 851-fillet-top-front-then-upright inputs (solidcore#2431 slowdown)

Case: SolidCraft `tests/kernel-cases/fillets/851-fillet-top-front-then-upright`. Its recipe.json and expected.json are copied here; expected.json grades by invariants, with no volume.

## Input
`before-fillet2-front-upright-r2.smb` (sha256 3bb0c88f3680ad519e6c15100bb6f42ae87d3a93e7c67489b8a4a34d55ad2e9c) is the body passed to recipe step 7 (index 6). It was captured by the rig's blend_call dump hook. Base and head dumped byte-identical bodies.
- Step 7: `solid.fillet {"edges": [[41.166666666666664, -21, 5]], "radius": 2, "tangent_chain": false}` (the front upright at the arc, R2).
- The body already has step 6's round: `solid.fillet {"edges": [[20, -21, 10]], "radius": 2, "tangent_chain": false}` (the top front edge, R2).
- The edge is given as a point on it. The blend selects the edge through that point, with tangent propagation off.

## Results (SolidCraft rig, solidcore kernel, the #2365 rig's flags and patches, C++ trap 0)
| build | step 7 result | time |
|---|---|---|
| base 8d300fbc1 | 1001 (tangent-propagation hint) | 0.6-0.9 s |
| #2431 ed45b3a80 | the rig's 180 s timeout; run alone, "unsupported" after 310 s | 180 s+ / 310 s |

The head's blend_call dump text reads "the blend is larger than its faces allow at the edge at (41.175, -20.413, 9.415); the largest radius that fits is 1.96". 
Note: this base comes before #2409 (20466d554). On main with #2409 the case builds (9524.3437, 10 faces), so measure main against main+#2431.

Filed by Claude Code for brett19.
