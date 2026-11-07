# B2 infrastructure investigation

Operational authority: `analysis/CODEX_HANDOFF_B2_CONSOLIDATION.md`, Sections 1, 12 and 14.
Starting branch/head: `b2-consolidation`, `466037b`. Existing uncommitted `formal/results/mutex.json` is preserved.

## Scope and execution order

1. Instrument the existing orchestrator in a separate diagnostics script; reproduce the historical worker death,
   testing one launch variable at a time. Do not infer a root cause from a successful probe.
2. Add a project-wide engine lease, opt-in engine tests and cleanup requiring parent, UUID and creation time.
3. Validate one isolated engine, one complete scenario, then two isolated roots/scenarios, before permitting two slots.
4. Close the infrastructure gate with all 11 development diagnostics without an infrastructure death.
5. Only then proceed to Issues 3, 4, 1, 2 and the remaining obligations, formal verification, freeze, campaign/report.

No property, threshold, noise profile, deadline or acceptance criterion is changed by this work.
Each behavioral change receives an independent review; accepted/rejected findings are recorded here.
No simulation runs concurrently with a full test/formal suite. Reviewers must never launch an engine.

## Source investigation

Installed Unreal source: `C:/Program Files/Epic Games/UE_5.3/Engine/Source`, version 5.3.2.

- `Runtime/TraceLog/Private/Trace/Control.cpp:59,113-125`: default 1985, then up to ten ports in 32768..40959;
  no runtime port option. `TRACE_PRIVATE_CONTROL_ENABLED=0` is compile-time and would require rebuilding TraceLog.
- `Runtime/TraceLog/Private/Trace/Detail/Windows/WindowsTrace.cpp:160-198`: IPv4 wildcard bind.
- `-notraceserver` suppresses UnrealTraceServer, not the engine's own control listener.
- A temporary wildcard socket with `SO_EXCLUSIVEADDRUSE`, without listening, prevents the default engine bind.
  If 1985 is already occupied, leave that owner untouched. Inspect our exact engine PID after launch, and reject
  an engine which nevertheless acquired 1985. The stock fallback selects a free high port, not a predetermined one.
  This briefly prevents other new binds to 1985; no existing listener is modified.
  Socket semantics: [Microsoft](https://learn.microsoft.com/en-us/windows/win32/winsock/using-so-reuseaddr-and-so-exclusiveaddruse).
- `-UserDir` gives a private project Saved/log directory; `-enginesaveddirsuffix` separates Engine Saved.
- Octrees and `materials.csv` need independent roots: `engine/Source/Holodeck/General/Private/Octree.cpp:132-148,501-522`.
  Neither cache nor materials may be junctions or hardlinks. Engine/Content can reuse the existing read-only junction pattern.

## Probe 1: instrumented orchestrator, inherited console

Started 2026-11-04 09:43 Europe/Rome. Diagnostic files: `results/dev/launch/baseline/1793781786142873200`.
Raw run: `results/robustness/raw_dev/infra_baseline/B2_CLEAN/CLEAN/gate_single/s0`.

- Orchestrator PID 59980; worker 65008; engine 13840.
- Engine UUID `296914a5-f75c-42ed-a547-b4ad7d55b64b`; creation time `1793781788.1292648`.
- Actual listener 37891, no engine listener on 1985; unique UserDir.
- Worker is in a Windows job object. This fact alone does not identify who terminates a process.
- Priority/affinity, stdout redirection and the orchestrator's worker command are retained.
  The child runs through an instrumentation wrapper; logs/Saved and port reservation differ from the historical launch.
- COMPLETE / PASS, worker exit 0, wall time 190.5 s, minimum true separation 3.281 m, max CR occupancy 1.
  E2, E3, E4 and E5 were zero. No idle or committed stall. Engine port remained 37891.
  Native engine exit 1 in the final diagnostic record follows the client's deliberate `Popen.kill()` at cleanup;
  it is not a worker failure.

This is exploratory infrastructure evidence, not a frozen result. Root cause and closure remain open.

## Independent review: diagnostics wrapper

All four findings were independently checked and accepted:

- Startup retry cleanup still used parent/name alone: override it with recorded parent/UUID/creation-time checks.
- A process-inspection race could kill the heartbeat thread: catch per-engine psutil errors and retain monitoring.
- A retry after releasing the first reservation could take 1985: reacquire before every engine launch.
- Relative output paths could differ between parent and worker: resolve the diagnostic output directory before launch.

No rejected finding. Reviewers ran only the diagnostic unit file (4 passed), never an engine.
Probe 1 had already started, with no startup retry and repository cwd, so the two latter fixes do not change it.

## Guarded launch and root preparation

`scripts/prepare_engine_roots.ps1` created `F:/Andrea/holoocean_patched_root_slot0` and `_slot1` without overwriting
the base root. Copied engine SHA256: `A5E9AA1FB7756C06B83AF3A0896E42E6188F0390DD7A3D3D6E6E0532549FA028`.
Saved, Octrees and materials.csv are ordinary private paths; Engine/Content reuse the existing junction pattern.

All project native launches now enter `sim/engine_session.make`. OS locks enforce one slot by default, with a hard
maximum of two (`HOLO_FLEET_ENGINE_SLOTS`, enabled only for the controlled validation after the single-engine steps).
The lease persists worker PID/creation time, UUID, engine PID/creation time/parent, root, command and observed ports.
The `.lock` holds the OS lock; the `.json` ownership record is atomically replaced after fsync. A pending handoff
is persisted before process creation and fails closed after an uncertain interruption. A rejected creation frees
its pending state only when inspection proves that no identifiable child has its UUID. An unreadable/unknown
handoff needs inspection; it is never silently reclaimed or used to kill a process by PID alone.

The original parent/name-only retry cleanup is removed. Native cleanup, orphan reaping and priority changes require
generation/parent/UUID evidence. Engine-launching pytest is opt-in with `HOLO_FLEET_ENGINE_TESTS=1`; ordinary runs
skip the octree engine regression. `test_run_demo.py` uses a fake runner and does not launch an engine.
The legacy single-beam calibration clears only its selected private cache under the engine lease; its final cache
is retained until the next k1/k2 launch rather than deleting a cache after releasing ownership.

## Probe 2: direct orchestrator, guarded private root

Started 2026-11-04 09:59:54 Europe/Rome, ended 10:03:09. No diagnostic wrapper.
Raw: `results/robustness/raw_dev/infra_guarded/B2_CLEAN/CLEAN/gate_single/s0`.

- Orchestrator 62184; worker 96840 (creation `1793782794.8601987`); engine 105252 (creation `1793782796.608513`).
- UUID `64dbc944-a2a8-4aa4-b0e1-2be4fa70afe6`, root `_slot0`, actual listener 37463.
- COMPLETE / PASS, worker exit 0, wall 193.6 s, minimum separation 3.248 m, max occupancy 1.
- E2/E3/E4/E5 = 0; no idle/committed stall. Root-local engine log and private cache/materials confirmed.
- The startup log includes a nonfatal D3D12 viewport Ensure (`SwapChain4.GetReference()`, SecondsSinceStart=0).
  It produced a diagnostic minidump; the scenario continued and completed. Preserve it with the log.

These two successful probes do not establish a root cause for the historical abrupt worker exits.
The guarded probe preceded the atomic-persistence/lock-scope review fixes and must be repeated on the reviewed code.

## Independent review: shared guard

Accepted after checking source and reproducing the defects:

- A truncate/rewrite or interrupted Popen handoff could hide a live orphan: atomic sidecar metadata and a durable
  pending marker; mutation-style tests fail on the previous implementation.
- Two calibration imports were damaged by a broad replacement: restored; all calibration files compile.
- Root/environment/cache selection was outside the same-process launch lock: the entire setup/launch is now under
  that lock, including environment changes in `use_patched_holoocean`.
- A known process-creation rejection left a permanently pending slot: clear only after passive no-child evidence;
  preserve uncertain handoffs. Fake native-boundary tests cover success, default/absent-port refusal and creation failure.

No rejected finding. Final re-review found no remaining concrete lifecycle/safety issue. Real two-engine operation
and the historical root cause are still unvalidated. The file-by-file full test run is in
`results/dev/tests_infra_20261104`: 38 files, 448 passed, one engine regression deselected, no failures/retries.

Provenance fix: untracked code now makes `code_version.dirty_files` nonempty, so a final campaign cannot silently
run untracked implementation. Both exploratory probes above were dirty-tree development runs.

## Current-operation validation and user steering

User instruction during this session: verify current two-worker operation and proceed with the substantive issues;
the historical infrastructure failure need not be causally explained first. It remains unexplained, not claimed fixed.

- Probe 3, background hidden supervisor: COMPLETE/PASS, worker exit 0, 184.6 s, P1 minimum 3.253 m. Its parent shell
  exited before the run, yet it completed; the historical tool-lifetime hypothesis was not reproduced either.
- `scripts/verify_engine_isolation.py` passed: two simultaneous native clients, different UUIDs, roots and ports,
  uniquely named agents/sensors. Each client received its own sensor and never the other one's. A repeat with
  explicit low priorities also passed (`results/dev/launch/pair_probe_low/isolation.json`).
- Two actual orchestrator workers, different complete scenarios, at low priority and I/O:
  `results/robustness/raw_dev/infra_two_workers`, shards 0/2 and 1/2, B2_CLEAN/CLEAN/s0.
  - gate_single: COMPLETE/PASS, 203.2 s, P1 minimum 3.281 m, max occupancy 1, E2/E3/E4/E5=0;
  - formation_triangle: COMPLETE/PASS, 133.7 s, P1 minimum 4.588 m.
  - workers 4004 and 87928; engine UUIDs `cc65a954-ec22-414b-ac34-c5776090748d` and
    `ab1bf228-8cba-4e98-828a-f930e1d6f6d8`; engine ports 37019 and 38578; roots `_slot0` and `_slot1`.
  - snapshot: both engines BELOW_NORMAL (16384), low I/O; free RAM 66.1 GiB, GPU free 40553 MiB, GPU utilization
    65%, host CPU sample 67.1%. These are observations, not newly invented acceptance thresholds.
  - all ownership records were clear after normal exits. No external process was modified.

The gate result is consistent with the single-instance probes (minima 3.248..3.281 m, all PASS), but these few
observations do not estimate run-to-run distributions or establish zero impact on unrelated workloads.
Per the user's steering, proceed to Issue 3 with the validated two-slot configuration. The default remains one;
development campaign processes explicitly select two, with the hard project cap unchanged.

Follow-up: retries now move prior INVALID/interrupted data and all pending worker logs into a unique `_attempts`
directory under the raw root, instead of deleting them. Both move endpoints are resolved/checked inside that root.
The independent review found a fallback log-name collision; an unused numbered name and a three-log preservation
regression address it. Targeted campaign unit file: 8 passed. `psutil`, already installed/used, is now declared as
a runtime dependency because the guarded launcher requires it.

## Cross-worker startup reservation race (14:12--14:40 Europe/Rome)

The Issue 3 BLACKOUT probe refused its launch explicitly: listener `[1985]` failed the port audit. This is not the
historical silent worker exit. The temporary reservation could be held by worker A when worker B checked, then
released before B's native process bound its listener. B correctly killed only its verified own engine and released
its lease; the refused attempt remains in `raw_dev/issue3_full_region_probe`.

`startup_lock()` now holds a project OS byte lock through setup, native launch, listener audit and reservation close.
It is released before simulation runtime; the two-slot lifetime cap and all native timeouts remain unchanged.
The OS releases this lock on worker death. Lock-contention polling changes no mission threshold or deadline.

Independent review found no concrete production defect, but identified a vacuous contender test: both test children
waited for stdin, so a missing lock still timed out. Verified the finding, changed only the first child to wait, and
executed a no-op-lock mutation: the revised test rejects it. `test_engine_lease.py` plus `test_engine_session.py`:
15 passed, including contention, abrupt owner exit, failure cleanup and listener refusal. No finding was rejected.

Two distinct orchestrator processes were then started with two slots and no campaign retry:
`results/robustness/raw_dev/startup_serialized_probe/B2_CLEAN/CLEAN`.

| Case | Start | Wall | Result | P1 minimum | Engine UUID | Port |
|---|---|---:|---|---:|---|---:|
| formation_triangle s0 | 14:38:31 | 130.2 s | PASS, exit 0 | 4.586 m | 90707d48-c3e7-41d4-991b-af72bfeece19 | 34466 |
| formation_square s0 | 14:38:41 | 122.9 s | PASS, exit 0 | 3.450 m | 769a3125-9d2f-4c99-8a18-4e6351f86791 | 33729 |

The intervals overlap, roots/logs are distinct, and the worker/engine generation and UUID records are in each
run_metrics.json. Both slot sidecars were empty after normal exit. Start samples reported 67.9/66.2 GiB free RAM;
these are resource observations, not acceptance thresholds. A later attempt to capture a live resource snapshot
found zero own engines because both had already finished; it supplies no additional priority/GPU evidence.
The low-priority orchestrator/engine policy is unchanged. No foreign process was modified.
