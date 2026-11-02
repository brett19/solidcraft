# WIP: cancel long solidcore operations from SolidCraft's job cancel

Parked work in progress. Branched from `fork/solidcore` at 584075662; the solidcore pin there is
2dbd88b6e, which has solidcore#2263, the `CancelToken`.

## Design
- **The token is per job, never per kernel.** `solidcraft_smlib::cancel::Cancel` wraps solidcore's
  `CancelToken`. Each `doc::progress::Progress` owns a fresh one, and `Progress::cancel()` throws it.
  Tokens are not reused, so nothing needs a reset.
- **How a call gets the token.** `progress::watch` → `kernel::watch::watching` sets a thread-local
  (`smlib::cancel::cancelling`), and `Watch::kernel_cancel` supplies its value. `kernel::par`
  carries it to rayon and scoped workers. STEP and IGES worker threads already re-enter `watching`.
- **Install and clear.** `smlib::core::run`, the single entry to the one process kernel and under
  its lock, puts the job's token on the `Kernel` for that call only and clears it afterwards.
  Calls run one at a time, so one job's cancel cannot reach another job's call.
- **What a cancelled call returns.** A call made after a cancel fails at once, before or after
  waiting for the lock. A call cancelled part way fails with `smlib::Error::Cancelled`, whatever it
  answered, and the kernel is replaced with a fresh one, as after a panic, because solidcore says
  its caches, RNG and context may be partly modified.
- **Shielded caches.** Lazy caches shared across jobs are filled with no token, so a cancel can't
  store a "cancelled" answer in them. These are `smlib::Brep::{cpp,core}`, `ssi::Surface`,
  `kernel::Body::smlib` and doc `ModelBody::{measure,joint_faces}`.
- **Typed outcomes.** New `KernelError::Cancelled` (STEP import's cancel uses it now) and
  `EngineError::Cancelled(cmd)`. `Session::execute` turns any non-panic error that happens under a
  cancelled watch into `Cancelled`, and the design is restored as for any failure. `add_feature`
  refuses a feature that the cancel skipped. A cancelled `doc.open` keeps its old `Ok` with skipped
  features.
- **Where cancel is wired in the UI.** `StopOnDrop` is held by the jobs that were not cancelled
  before: the agent `ControlJob` (Stop), the agent-cursor `Ahead`, history `Deferred` (the Cancel on
  the Computing toast) and `DeleteCheck`. These jobs now run under `progress::watch`. Dialog
  `Applying`, file load and design properties already cancelled their `Progress`.
- **C++ kernel.** No change: there is no token, and its calls run to the end.

## Measured latency (ci profile, x86-64, cancel 4 s in; logs in scratch-solidcraft/cancel/latency-{1,2}.txt)
| Case | Call | Returned after cancel |
|---|---|---|
| booleans/3455-join-of-64-separate-pegs (combine) | smlib_boolean_union_all | 0.07 s |
| booleans/5126-join-menger-sponge-level-3 (combine) | smlib_boolean_union_all | 0.24 s |
| booleans/3455 (extrude, in its check) | check | 0.01 s |
| shell/566-shell-swept-bar-with-bend | smlib_shell | 3.8 s (ran to the end) |
| fillets/9-fillet-corner-r5-curvature-default-setback | blend_call | 1.2 s |
| fillets/cov-fillet-section-curvature-boss-root-r2 | blend_call | 76.9 s (ran to the end) |
| fillets/every-hole-top-rim-r1-curvature | blend_call | 81.7 s (ran to the end) |

The Gⁿ section (curvature) round (`blend/fillet_section*`) and offset/shell have no check points
in solidcore. An issue draft for solidcore is at
`/home/brett/tangency/scratch-solidcraft/cancel/drafts/solidcore-cancel-gaps.md`. It is **not
filed**; dedup was done (only #2136/#2263 are related).

## Tests written
- smlib `core::tests::a_cancel_stops_its_jobs_calls_and_no_other` and
  `cancel::tests::a_cancel_reaches_only_the_calls_made_inside_its_scope`.
- engine `tests::a_join_cancelled_while_solidcore_computes_it_leaves_the_design_and_the_next_job_alone`
  passes in 4.9 s. It cancels mid `smlib_boolean_union_all`, then checks for `Cancelled`, that the
  doc `Arc` is the same, and that the next job builds the exact volume.
- engine `tests::cancelling_one_designs_job_leaves_another_designs_job_alone` passes in 2.4 s.
- ui-egui: the Stop test in `history_off_thread_tests.rs` and the Cancel test on the Computing
  toast now assert that the worker's `Progress` is cancelled. Not yet run.

## Untested / remaining
- **Test run t2** (`--profile ci` across smlib, kernel, doc, engine and ui-egui) was stopped at
  8822 passed of 9139. Every failure up to that point is in `tests/KNOWN_FAILING_TESTS`, except
  `kernel step_in::tests::a_solid_smlib_refuses_with_its_faces_is_refused_not_meshed`. That test
  runs no watch, so this change cannot affect it, and it also failed in an earlier smlib-merge run.
  The ui-egui tests are not confirmed. `hostile_params_never_panic` (merge-queue only) was not
  finished.
- **Gates not run:** fmt has run; still to run are clippy -D warnings with and without C++, xtask
  layers/lists/ui-containers/skills, wasm with and without C++ (scripts/fetch-smlib.sh
  wasm32-unknown-unknown first), and known-failing.
- **Screenshots** of the visible effect (Stop or Cancel on a slow boolean frees the kernel at once)
  have not been taken.
- **Open question:** a cancelled call's `&mut Brep` operands. Blends run on `brep_copy`, but
  measures lock a shared body and take `&mut` (solidcore may fill memo caches in it).
- **Delete this file** before the work leaves WIP.
