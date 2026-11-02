# cov-fillet-section-curvature-pipe-on-plate-r2: inputs and results (PR-rig, local only)

Case: tests/kernel-cases/fillets/cov-fillet-section-curvature-pipe-on-plate-r2 (recipe.json, expected.json here).
A plate 60x40x10 joined to a pipe R10. The concave edge is rounded curvature-continuously at size 2. Analytic volume 40307.8856312.

| build | result |
|---|---|
| base c002debb5 (b2308) | fail: 40302.590788, 9 faces; the volume check fails (off by 5.3) and the continuity_order check fails at the picked seam |
| 366c604da, fillet_round_as_definition OFF | pass: 40307.885632797, 9 faces (1.6e-6 from analytic) |
| 366c604da, fillet_round_as_definition ON | fail at export: "an approximated face surface that is not a B-spline" (the ProceduralBlend gap at convert.rs:271) |
| 941770b26 (#2274), ON, with the converter's ProceduralBlend arm (net lift) | pass: 40307.885632797, 9 faces |

The C++ trap logged no calls on any run. dumps-all/: every SMLib-API call (PRRIG_DUMP_ALL), from the h2369nd build
(366c604da, flags as in ../2368-together-r2/README.md). dump-run.json and dump-run-stderr.txt: that run's result and stderr.
result-*.json: the CLI's case results per row above.
SolidCraft: ws/pin-16 5c09cff62 plus ../2368-together-r2/solidcraft-local.diff.
