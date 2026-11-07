# B2 consolidation: implementation status

**Operational update, 2026-11-04 evening (H6 payload phase, continued by Claude at the user's request):**

- **Why H6.** Codex executed a counterexample: range-bearing fixes alone cannot separate an occupied CR from a cleared one
  (results/dev/issue3/indistinguishable_occupancy.json); the full-region veto is safe but stalled CLEAN gate_single.
- **H6 (commits e6eb5db, 6b2d596, c69643b, 443adf6):** every ping carries a 56-bit packet (perception/payload.py):
  onboard mission-frame position, an error-ball bound from NavBound (perception/nav.py, declared MEDIUM sensor
  envelope, monotone, integrity flag), 2-bit TDMA frame, CRC-16; 129 ms of the 167 ms slot at 1 kbit/s net (an
  unverified bench assumption, stated).  Strictly one-way.  Receivers attach a payload only if consistent with the
  range-bearing region; gate rules use the intersection (B2-P randomized check + mutation B2-Pm); an unknown occupant
  never becomes FREE by timeout and holds the stage-2 entry; veto hysteresis; acoustic-only episodes need A2 of the
  drones heard inside; relocalization beside the opening accepted only when payload balls exclude every other drone
  (evaluated at the uncorrected pose); formation lag/ahead/A-D from the neighbours' reported positions.
- **Issue 1:** automaton edge pass_abort (MUTEX_PASS -> MUTEX_APPROACH, committed cleared), observation entry_abort,
  invariant I5, G2/G2c restated, mutation MA; give-up only behind s = -(cr_half_len + R_OUT + 1.0 m), never once
  admitted.  Determinism 104/104, observations 32/32.  Livelock from a persistent stage-2 cause documented (M5).
- **Referee v4:** UNSAFE_ACTION = E2.to_free | E3 | E4; inclusive 1 m; SAFE_DEGRADED with the hold; crossing
  obligation from the first commit; bypass participation; nav-bound coverage per run (NAV_BOUND_PREMISE_VIOLATED).
- **Scenario gate_stalled_occupant** (ground-truth mid-CR thruster loss) and the pre-registered campaign
  (docs/ACCEPTANCE_CRITERIA.md v4, `--phase b2final`, 920 runs on seeds 100-109).
- **Reviews:** two independent reviewers (formal; underwater acoustics/methodology), two rounds each; every blocking
  finding fixed or recorded (bound reset removed as unsound; CRC-16; ball over h and z; sticky give-up flag; stage-2
  unknown occupant).  Open, stated: modem air time unbench-tested; payload loss is a stress knob; HIGH outside the
  envelope; give-up livelock; a drone silent after a possibly-inside fix blocks the gate.
- **Evidence so far:** smoke B2_CLEAN gate_single s0 PASS at T_nom with the payload (Codex's candidate STALLED); payload
  off (B2NP_) on the same case STALLED; NavBound never exceeded in 303 231 baseline samples (offline) nor in the
  runs logged so far.  Batch 1 was mostly INVALID (engine slots busy: workers now wait for a slot).

The entries below are history.


**Operational update, 2026-11-04:** `CODEX_HANDOFF_B2_CONSOLIDATION.md`, plus the user's latest instructions,
supersede the historical entries below. Starting head: `466037b`, branch `b2-consolidation`.

- Infrastructure: shared OS-held slots (default one, hard cap two), atomic ownership records, exact-identity cleanup,
  separate writable roots/logs/cache/materials, non-default TraceLog listeners, opt-in engine tests.
- Independent infrastructure reviews resolved. File-by-file suite: 448 passed across 38 files, no failures.
- Single-engine, background-supervisor and two-client sensor-isolation probes passed. Two actual low-priority
  orchestrator workers completed `gate_single` and `formation_triangle` with PASS and P1/P2 held.
- Per the user's steering, the unexplained historical worker death no longer blocks progress; it is not claimed fixed.
- Baseline complete: 55 diagnostic runs (five repetitions of 11 cases), 46 PASS, five STALLED, four INCOMPLETE,
  all worker exits 0 and P1/P2 held. Aggregates: `results/b2/consolidation/before_issue3`.
- Current work: Issue 3 candidate, not accepted. Full-envelope occupancy blocks unsafe releases, but a CLEAN probe
  stalled before any CR passage. Independent review and an executed identical-packet example expose a conflict
  between unconditional expiration after true exit and complete uncertainty coverage. No criterion was relaxed.
- Current required user decision: the conditional Issue 3 closure proposal in
  `2026-11-04_ISSUE3_DESIGN_AND_FINDINGS.md` (theoretical expiration scope and separate late-occupancy E7 coverage).
  The question is pending. Do not infer approval from time, quota reset, automation activation or autonomous-work
  authorization. Do not mark Issue 3 closed or advance to Issue 4 until resolved; the CLEAN stall remains a blocker.
- Candidate checks: 39 files / 464 passed, one engine regression deselected, no retries; 85 latest focused tests
  passed after the final producer/capture fixes. Independent reviews and verified mutation checks are recorded in
  the Issue 3 findings. Final formal rerun and after diagnostics are still outstanding.
- A concrete cross-worker TraceLog reservation race is corrected by serializing startup only. Independent review,
  a no-op-lock mutation and 15 unit tests passed; two distinct concurrent orchestrators then completed PASS, exit 0,
  with distinct roots/UUIDs/ports. See the infrastructure log. Two runtime instances remain permitted, default one.
  Startup fix `bce9194` is pushed; both validation runs ended and no own engine remains active at this checkpoint.
- Next: Issues 4, 1, 2; remaining referee/default/documentation items; full formal rerun; freeze; final campaign/report.
  No final campaign, freeze or main merge has occurred. The withdrawn fit and flow-only retreat remain withdrawn.
- Quota checkpoint history: work stopped at the prior window limit and resumed after the user pointed out the missed
  13:53 restart. At 14:00 the renewed window was verified at 1%. The next reset is 19:00:50 Europe/Rome; the same
  continuation automation is scheduled explicitly in UTC for 18:50. Check actual usage and stop at 90%.
  Resume context: `2026-11-04_ISSUE3_DESIGN_AND_FINDINGS.md`; inspect ownership before any simulation, and resolve
  `2026-11-04_CRITERIA_REVIEW.md` before freeze. No automation timing guarantee is claimed.

Evidence and review decisions: `2026-11-04_INFRASTRUCTURE_LOG.md`. The older implementation notes below are retained
as history; in particular their original G.4 claims were withdrawn, as the later review section records.

Branch `b2-consolidation` (from `b2-acoustic-aided` at `a6efd0f`). Plan: `analysis/2026-11-03_B2_REVIEW_AND_PLAN.md`,
Section G. The user approved it on 2026-11-03 and asked for autonomous execution of G.1-G.7, with independent reviewers
in every phase, literature-grounded criteria and a final independent validation campaign.

## Historical implementation record (superseded by the operational update)

### G.1 Criteria v3 and classifier (referee only)

Files: `holo_fleet/referee/criteria.py`, `terminal.py`, `gate_events.py`, `robustness_metrics.py`, `referee.py`, plus
`config.RefereeConfig`.

Every constant carries a provenance label: [REQ] requirement, [PROT] protocol bound, [LIT] literature, [PROF] derived from
the profile definition.

**Tolerances and hold time:**
- E_TOL = 1.0 m is a single band for P3 [REQ].
- t_ok_hold = 4 / k_slot = 8.9 s, the 2 % settling time of the slot-tracking loop [LIT].
- The progress tolerance is t_rejoin · (v_nominal - v_wait) = 3 m [PROT], checked at the SUCCESS instant (not latched).

**Targets and deadline:**
- T_nom is the instant the plan clock reaches the end of the path.
- The targets are the planned end positions.
- The deadline is T_nom plus: the gate excess over the plan's dwell; t_rejoin + T_hold; a give-way term for single
  drones; the fault terms; and FAILSAFE outages at the 99 % Poisson quantile of the profile's outage processes. Nothing is
  derived from runs of the system under test.

**Requirements:**
- A gate passage must go through the critical region (CR) cross-section, otherwise it is tagged GATE_BYPASS.
- P3-deg vacancy is required only from drones that expect the lost drone as a neighbour.
- A false vacancy is judged on ground truth.

**Gate events E1-E8:**
- Each decision is judged against the truth at the instant it was made.
- E2_free counts any release to FREE while the CR is truly occupied.
- E4 is the stage-2 admission, logged as `gate.entry`.
- Bounded-response gate liveness:
  - idle: every healthy, not-passed drone is queued, the CR is empty and nobody is committed, for longer than
    t_occ_max + t_clear;
  - committed: longer than L / v_pass + t_occ_max + t_clear.
- A-cross is reported as a validity flag.

**Classes:** INVALID, SUT_ERROR, SAFETY_FAILURE, UNSAFE_ACTION, SPEC_VIOLATION, STALLED, PASS, CENSORED, SAFE_DEGRADED,
INCOMPLETE.
- SAFE_DEGRADED is causal: the drones in sensor-fault FAILSAFE are noise drones and explain every missed requirement.
- Every metric is computed on the replay horizon.
- The runner records exceptions in the system under test as SUT_ERROR.

**Campaign script:** v3 deadlines; no B0 CLEAN dependency.

**Offline re-evaluation:** `scripts/reevaluate_v3.py`.

**Independent review:** review G1 found 1 blocking and 12 major issues. All were addressed; a re-review is pending.

### G.5 Formation-clock re-synchronisation (B2)

Files: `perception.py` (rules A-P/A-D, `acoustic_lag`), `formation_time.py` (`behind`, `sync`), `controller.py`.

- A neighbour heard certainly AHEAD on its lane is present, not missing. This removes the positive feedback that let the
  private clocks (tau) diverge.
- A neighbour certainly BEHIND slows the drone's own clock by k_progress · lag, staying within [v_wait, v_plan]. This is
  a first-order progress consensus (Ghabcheloo et al. 2009).

### G.3 Gate soundness and liveness

- `acoustic_gate.heard_inside` uses only the clear-path region (no one-sided multipath excess), requires two fixes on one
  track, and a latest fix no older than one ping period plus the latency. Before, a drone waiting beyond the exit looked
  possibly inside and blocked the queue.
- Rule A4: with every other drone excluded at t_lc and positively outside now, the phantom corridor echo is released.
- Stage 2 at the hold point, `GateObs.entry_ok`: no persistent CR-compatible FRONT echo over 3 captures, and not heard
  inside.

### G.4 Gate liveness

- `structure_aiding.update(drones=...)`: with every other drone located, echoes that a drone could have produced are
  dropped and a fit beside the opening may be applied. Synthetic test: at s = -1.8 m, lateral errors of 0.6-1.2 m are
  corrected to 0.06-0.35 m.
- `flows.gate`: the stage-2 hold is bounded to t_occ_max + t_clear. After that the drone returns to its queue point and
  retries, at most 2 times, then gives up. ENTRY_RETRY and ENTRY_GIVEN_UP are logged events.

## Independent review G345 (behaviour changes) and the response, commit 92a72e5

**Blocking findings, both verified and fixed by withdrawing the mechanism:**
- **B1.** A retreat after a bounded hold left the drone committed, so it re-approached without the stage-1 check and two
  drones could be committed at once. The retreat and retry are removed; the hold is logged once as ENTRY_HOLD_TIMEOUT. A
  real give-up needs an automaton edge MUTEX_PASS -> MUTEX_YIELD plus formal re-verification. This is pending.
- **B2.** The identity-assisted map fit dropped every bar echo near the gate, because the acoustic regions are far too
  wide under the deployed envelope. It is withdrawn: `structure_aiding.py` is back at 0d1b7cd.

**Major findings:**
- **M1.** The `heard_inside` veto had a gap after one lost ping. Fixed: freshness is now `fresh_s`.
- **M2, M3, M4.** The "ahead counts as present" rule:
  - it fed unbounded residuals into the progress consensus;
  - it could keep a drifting failed drone from ever being declared vacant;
  - its envelope-certainty dead bands of 3-5 m left the positive feedback in place.

  Redesigned:
  - point estimates (the median of A_CONFIRM fixes of one track);
  - a lag beyond k_sigma · sigma_w(r), with a continuous sync;
  - the A-P rule back to v2;
  - "ahead" only vetoes a NEW missing episode, for at most t_rejoin.
- **M5** (retreat path) and **M6** (B2-S isolation): both resolved by the withdrawals.
- **A4:** not yet tested under the deployed envelope (minor).
- **M4h:** only 16 % of the CR is observable from the hold point, so the stage-2 sonar check has little value.

A re-review of both reviews (G1 and G345) has been requested.

## Re-review of G1 (commit 92a72e5) and its fixes (commit 05e1c5e)

**Fixed in 05e1c5e:**
- **N1 (blocking).** The online monitor and the offline replay used different T_nom. Both now use the planned end.
- **N2.** Slow committed passages now violate the crossing budget.
- **N3.** A v3 replay mismatch is classified REPLAY_MISMATCH.
- **N4 (partly).** Errors while building the controllers are recorded as SUT_ERROR or INFRA.
- **N6.** Stage 2 is detected from the logged key.
- **N9.** Overstated claims have been reworded.

**Open:**
- **M2.** No generic no-progress detector outside the gate.
- **M3.** A limit cycle with an in-band phase longer than 8.9 s still passes; this is stated as a limit.
- **M10.** The contacts and the intruder flag are still taken from the full log.
- **N5.** The historical re-evaluation is almost entirely CENSORED. T_nom is now the planned end, so the old logs are too
  short.
- **N7.** The idle-stall check stays off after a bypass.
- **N10.** The avoidance term is loose (an upper bound).
- **N11.** The FAILSAFE allowance covers only blackouts and DVL outages, per drone.
- **Minor:** P3-deg for n < 3; the p1_two_lines formation term; the t_occ_max term for the first passage.

**Re-review of G345:** requested after 92a72e5; result pending.

## BLOCKER (2026-11-03, 23:35 onwards): the scenario runs die in HoloOcean

**Symptom.** Every scenario run dies 2-3 s of simulated time (3-5 s of wall time) into the loop:
- exit code 1;
- no Python traceback;
- `finally` blocks do not run (`events.jsonl` stays empty);
- the HoloOcean engine log stops without a shutdown.

**Where it fails:**
- B0 and B2 alike;
- in-process (`runner.run`), as an orchestrator worker, and outside the sandbox;
- also at the pre-change commit a6efd0f (checked in a temporary worktree, since removed).

The cause is therefore not the consolidation code.

**What still works:**
- a minimal HoloOcean environment (one BlueROV2 with DVL; one BlueROV2 with the six sonars, static, 600 ticks);
- `tests/test_octree_patch.py::test_octree_rebuild_regression` when run alone.

**Other observations:**
- Long pytest runs, and a ProcessPool in `reevaluate_v3.py`, also died silently at random points, with "A process in
  the pool was terminated abruptly".
- Per-file test runs pass.
- The GPU has 42 of 49 GB free; disk and RAM are fine; Defender and the system logs show nothing relevant.
- Other processes on the machine, which must not be touched: CARLA (`scripts/start_carla.py`, since 02/11 16:53), the
  user's Instagram worker (started 23:11), and the CodexSandboxService.

**Update at 2026-11-04 00:05:**
- Runs that completed:
  - the in-process probe (`runner.run`, B0 and B2);
  - a worker started in the foreground from bash, with `--terminal` (gate_single, up to t = 160 s, status COMPLETE).
- The deaths are not deterministic and not caused by the consolidation code. Every worker launched through the
  orchestrator loop (`scripts/run_dev_diagnostics.sh`, started in the background) died about 15.7 s after launch.
- Confound, my own mistake: several of my probes ran while the background loop was alive, i.e. two HoloOcean instances
  at once. The reviewers' test runs may also have started HoloOcean, because
  `test_octree_patch::test_octree_rebuild_regression` launches the engine.
- Rule from now on: before any HoloOcean probe, no other HoloOcean or orchestrator may be running. Reviewers must not run
  that test.
- One orphan worker of mine (gate_single, started 00:03:35) was left to finish on its own; stopping it by PID was refused
  by the permission classifier.
- Probe outputs are in `results/robustness/raw_dev/probe3` and `probe4` and are not valid evidence; they ran
  concurrently.

**Next steps to try:**
- move the static sonar probe;
- add the gate structures and the octree rebuild;
- run 3 drones;
- check the noise wrapper and the beacon transport;
- run `scripts/run_demo` for one scenario;
- if needed, ask the user to check the machine.

## Measured facts used for the design

**Stage-2 false holds** at the hold point with the CR truly empty, from the logs of 624 runs:

| Condition | CLEAN and LOW | MEDIUM |
|---|---|---|
| Latch not FREE | 0 | 11-55 % |
| FRONT non-STRUCTURE echo ≤ 2.6 m | 0 | 3-19 % |

The latch is therefore not used at stage 2.

**Geometry from the hold point:** the bars appear in the FRONT cone from 1.27 m. The far half of the CR is masked, so
M4h cannot hold in full. This is a stated limit.

**One-sided braking (S1):**
- SAT at the configured a_brake = 0.5;
- UNSAT at the calibrated 0.74;
- the smallest verified d_warning with one-sided braking is 2.56 m.

## Historical next steps (superseded by the operational update)

1. Fix any failures from the full test suite and commit. Authorship is Andrea Bedei only, with no co-author trailer.
2. Targeted HoloOcean diagnostics, one simulator at a time at low priority, in `results/robustness/raw_dev/`:
   - formation_six MEDIUM-ALL s1;
   - formation_square MEDIUM-ONE s2;
   - gate_single MEDIUM-ONE s0;
   - lost_drone_mutex MEDIUM-SUBSET s0 and B2_CLEAN s0;
   - integrated_short MEDIUM-SUBSET s2;
   - mutex_deadlock_resolution LOW s2;
   - gate_single MEDIUM-SUBSET s1;
   - lost_drone_timeout MEDIUM-SUBSET s2.
3. Independent reviewers: a re-review of G.1, and a review of the behaviour changes (G.3, G.4, G.5).
4. G.6 identity-based vacancy and re-inclusion; G.2 B2 as default; formal updates (I); documents.
5. Reference runs, the held-out set, the frozen campaign and the final report.
