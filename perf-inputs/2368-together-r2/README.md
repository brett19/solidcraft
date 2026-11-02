# report-drafted-peg-front-and-concave-rounds-together-r2: inputs (PR-rig, local only)

Case: tests/kernel-cases/fillets/report-drafted-peg-front-and-concave-rounds-together-r2 (recipe.json, expected.json here).
The call: step 13 solid.fillet, radius 2, tangent_chain on, picks at (20, 0.87489, 10) (the drafted
peg's front top edge) and (32.87489, 8, 10) (the top's concave arc where it meets that edge tangentially).

Result: fail, "SMLib failed (status 1001)", 0.37 s. The C++ trap logged no calls.
- The inner SolidError: Upstream { operation: "blend::fillet_executive::session::finish", code: 1001 }
- error_info: code 3 (VertexProblem), status 2, corner_type NByTwo, vertex VertexId(5), entity
  SmFilletNx2Corner, comment "Fillet Corner Error: Unable to Make Topology". After create_corners,
  step_ok = true, with no error noted. SolidCraft's corner flag was false, so the error was mapped by status() to Error::Status(1001).
  (History: unsupported on #2368's base, 1001 on #2368.)

solidcore: 366c604da (sc-h2369n), see solidcore-used.txt.
SolidCraft: ws/pin-16 5c09cff62 plus solidcraft-local.diff. The diff holds the C++ trap; the flags fillet_round_as_definition and
fillet_keep_features on and fillet_extended_rail_end_at_side_edge_crossing removed; the PRRIG_DUMP_ALL dump of every call; and blend.rs eprintlns (the PRRIG lines in run-stderr.txt).

Files:
- rig-failure-dump/: the failing blend_call's input brep (.smb). The dump records no numeric arguments; use the picks and radius above.
- dumps-all/: every SMLib-API call of the run (48 files).
- run.json, run-stderr.txt: the CLI's case result and stderr.
