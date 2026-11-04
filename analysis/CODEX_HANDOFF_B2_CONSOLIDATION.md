# Codex handoff: B2 consolidation of HoloHybridAutomaton

Written 2026-11-04 on branch `b2-consolidation`, head `a19c8fe` (pushed to `origin/b2-consolidation`). This document is
the authoritative operational context for continuing the work. It is self-contained; the original files are cited so
that every statement can be checked.

Labels used throughout:
- **[IMPL]** implemented and committed, with unit tests;
- **[IMPL-UNVAL]** implemented and committed, not yet validated in HoloOcean;
- **[WITHDRAWN]** implemented, found wrong by review, removed (must not come back);
- **[PROPOSED]** designed or planned, not implemented;
- **[HYP]** an unverified hypothesis;
- **[FACT]** measured or checked, with the source given.

---

## 1. Instructions for Codex

1. Continue autonomously from the current Git state of `b2-consolidation`. Treat this file as the authoritative
   operational context; where it disagrees with an older document, this file wins, and fix the older document.
2. Do not work on `main`. At the very end, merge `b2-consolidation` into `main` with an explicit merge commit
   (`git merge --no-ff`). Never squash, never rebase destructively, never force-push.
3. Commits are authored by **Andrea Bedei** only. No co-author trailer, no "Generated with" line.
4. Reply to the user in Italian; write code comments, documentation and reports in English.
5. Read first, in this order:
   1. this file;
   2. `analysis/CONSOLIDATION_STATUS.md`;
   3. `analysis/2026-11-03_B2_REVIEW_AND_PLAN.md`, Sections 0, F, G, H and I;
   4. `docs/GUARANTEES.md`;
   5. `docs/B2_ACOUSTIC_AIDED_COORDINATION.md`;
   6. `formal/results/SUMMARY.md`;
   7. `analysis/review_A_formal_hybrid.md` and `analysis/review_B_marine_experimental.md`.
6. Work order (details in Section 12):
   1. the infrastructure: the orchestrator failure, then HoloOcean isolation and parallelism;
   2. Issue 3, the occupancy veto;
   3. Issue 4, formation synchronisation;
   4. Issue 1, the abort/retry edge;
   5. Issue 2, gate localisation;
   6. the remaining open items;
   7. formal re-verification;
   8. the freeze;
   9. the final campaign;
   10. the report.
7. Use independent reviewers (sub-agents) for every behavioural change, not only at the end. A reviewer gets the code,
   the diff, the property definitions and the constraints, never your conclusions or earlier review verdicts. Verify
   every reviewer claim yourself before acting on it, and record accepted and rejected findings with reasons.
8. The user's standing rules (Section 14) are binding: no invented thresholds or criteria, no weakening of the
   properties, no longer timeouts to hide deadlocks, and no absolute guarantees where physical, sensing or
   environmental limits exist.
9. Stop and report to the user if a decision would change P1, P2, P3 or the frozen acceptance criteria, or if a
   process that is not ours would have to be touched.

---

## 2. Project objectives and scientific contributions

The project is a leaderless BlueROV2 fleet in HoloOcean 2.3.0 (Unreal 5.3.2) with a patched sonar octree. Each drone
runs the same local **hybrid automaton** (10 modes, `holo_fleet/ha/spec.py`). There is no coordinator and no
negotiation.

Perception per drone:
- six wide-beam single-beam sonars;
- DVL, IMU and depth;
- in B2, a one-way acoustic identity beacon.

Contributions to defend:
1. **A decentralised, verifiable automaton** with an explicit priority hierarchy: safety > gate > formation, and
   fault → FAILSAFE from every mode. Determinism, completeness and the observation invariants are checked by z3.
2. **P1 separation** under a stated sensing envelope: an association-free distance bound plus a braking and escape
   chain.
3. **P2 mutual exclusion** at a gate: a decentralised precedence rule (queue order) plus an occupancy latch. B2 adds
   acoustic evidence.
4. **P3 formation recovery**, and P3-deg after permanent faults.
5. **B2**, a one-way acoustic identity beacon: TDMA pings, no payload, no handshake. Range comes from the one-way
   travel time, bearing from an inverted USBL. The beacon is used only as additional evidence for occupancy, exit and
   neighbour identity, never for separation or precedence.
6. **An honest evaluation methodology**: a-priori criteria v3, gate events framed as STPA unsafe control actions,
   bounded-response liveness, and Kaplan-Meier completion analysis.

---

## 3. Architecture and the B2-only decision

- **B2 is the only operational architecture** (the user's decision of 2026-11-03).
  - B0 (sonar-only) stays as **historical results only**: `results/robustness/raw_final/B0`, about 290 runs.
  - The sonar-only logic remains as B2's **acoustic-outage degraded mode**. The BLACKOUT channel is the outage
    condition.
- B2 code paths:
  - `holo_fleet/sim/beacon.py` (BeaconTransport, TDMA) and `holo_fleet/sim/acoustic_channel.py`;
  - `holo_fleet/perception/acoustic.py` (tracker, fixes);
  - `holo_fleet/perception/acoustic_gate.py` (A1/A2 exclusion, exit by range, `heard_inside`, A4);
  - `holo_fleet/perception/perception.py`, `acoustic_neighbours` (A-P / A-D rules, the "ahead" veto, `acoustic_lag`).
- The isolation of B2 is checked statically: `tests/test_b2_isolation.py`, formal B2-S. Acoustics never reach
  separation, collision avoidance, precedence or the sonar witness.
- **[PROPOSED] G.2 is not done.** Still pending:
  - `holo_fleet/config.py` still has `BeaconConfig.mode: str = "NONE"  # NONE (B0) | ACOUSTIC_RB (B2)`;
  - `scripts/run_sensor_robustness_campaign.py` still plans protocols `B0` and `B2_<level>`, with `SEEDS = (0, 1, 2)`;
  - the final campaign plan (Section 13) is **not yet encoded** in the orchestrator.
- Main configuration (`holo_fleet/config.py`; every value below was checked in the file):

  | Group | Fields |
  |---|---|
  | `SeparationThresholds` | `d_warning = 2.4`, `d_warning_exit = 2.7`, `d_safe = 1.0` |
  | `Envelope` | `dt = 0.1`, `tau_max = 0.25`, `a_brake = 0.50`, `a_brake_measured = 0.743` (formal S1d only) |
  | `GateRule` | `cr_half_len = 0.8`, `cr_half_width = 1.0`, `cr_half_height = 1.0`, `queue_spacing = 3.5`, `t_clear = 1.0`, `t_occ_max = 25.0`, `exit_s = 3.5`, `v_pass = 0.50`, `queue_lateral_max = 3.5` |
  | `FormationRule` | `e_lost = 1.2`, `e_ok = 0.5`, `t_ok_hold = 2.0` (the controller's own values, unchanged); `k_slot = 0.45`, `k_progress = 0.08`, `v_nominal = 0.30`, `v_wait = 0.20`, `neighbour_range_m = 7.5`; `catch_up` and `t_rejoin` are properties |
  | `RefereeConfig` | `e_lost = 1.0`, `e_ok = 1.0`, `t_ok_hold = 8.9` |
  | `BeaconConfig` | `period_s = 1.0` |
  | `AcousticConfig` | `k_sigma = 3.0`, `sigma_range_m = 0.15`, `sigma_az_deg = 2.5`, `latency_max_s = 0.40`, `fresh_s = 2.5` |

---

## 4. Property definitions (do not change them)

- **P1 separation.** For every pair of drones the true distance is at least `d_safe = 1.0 m` at every instant, on
  ground truth. A contact with gate structures is outside P1's drone-drone scope; it is reported separately.
- **P2 mutual exclusion.**
  - At every instant, at most one drone is inside the critical region (CR), on ground truth:
    `G(Σ_i inside_CR_i ≤ 1)`.
  - The CR is the box `|x| ≤ cr_half_len`, `|y| ≤ cr_half_width`, `|z| ≤ cr_half_height` in the gate frame.
  - Precedence: exactly the first drone of the queue order leads. Decentralised precedence is part of P2's design;
    it is checked formally by M1/M2.
- **P3 formation recovery (criteria v3).**
  - Formation scenarios: the formation error of the healthy drones is at most `E_TOL_M = 1.0 m`, held for
    `T_HOLD_S = 4 / k_slot = 8.9 s`.
  - Every loss episode (error > E_TOL) has recovered.
  - One band opens and closes an episode.
- **P3-deg.** With a permanent fault, the formation is that of the healthy drones on their original slots. Every
  healthy drone that expects the lost drone as a neighbour (template distance ≤ `neighbour_range_m`) has declared its
  slot vacant. No healthy drone holds vacant the slot of a drone that is not permanently faulted (ground truth).
- **Mission requirements (v3)**, judged at the SUCCESS instant (`holo_fleet/referee/criteria.py`, module docstring):
  - **progress:** every healthy drone is within `progress_tol_m = t_rejoin·(v_nominal − v_wait) = 3 m` of its
    planned end position at T_nom;
  - **gate:** every healthy drone has passed through the CR cross-section; a drone that went around the frame is
    tagged GATE_BYPASS;
  - **not waiting:** no drone is still in FORMATION_WAIT_REJOIN;
  - **gate liveness** (bounded response, Section 6.3).

The controller thresholds (`FormationRule.e_lost/e_ok/t_ok_hold`) are **not** the referee tolerance. They stay as
they are; the 1.0 m tolerance is a referee and acceptance criterion only.

---

## 5. Implementation status of G.1-G.7

| Step | Status | Content |
|---|---|---|
| G.1 instrumentation, criteria v3, classifier | **[IMPL]**, offline-tested | Sections 6.1-6.4 |
| G.2 B2 as the only architecture | **[PROPOSED]** | defaults, campaign protocols, archive B0, docs; README wording already fixed |
| G.3 gate admission and veto | partly **[IMPL-UNVAL]** | stage 2 `entry_ok`, the `heard_inside` redesign, A4. **Not done:** (b) witness hardening (3-capture confirmation and a B2 veto on the sonar witness); (e) positive acoustic occupancy |
| G.4 gate liveness | **[WITHDRAWN]** parts (b) and (d); only a logged hold timeout remains | Issues 1 and 2 |
| G.5 formation clock re-synchronisation | **[IMPL-UNVAL]** | Issue 4 |
| G.6 identity-based vacancy and re-inclusion | **[PROPOSED]** | Issue 4 |
| G.7 criteria v3, deadline, freeze | criteria and deadline **[IMPL]**; freeze **not done** | Section 6 |

### Commits on `b2-consolidation` (branched from `b2-acoustic-aided` at `a6efd0f`)

| SHA | Content |
|---|---|
| `0d1b7cd` | analysis only: review, root causes, reviews A/B, literature review, plan |
| `92a72e5` | step 1: criteria v3, gate events, G.5 resync, `heard_inside`, A4, stage 2, formal S1o/S1d, `check_stage2.py` |
| `05e1c5e` | re-review fixes of G.1 (N1-N4, N6, N9) |
| `3f69fa6` | status update |
| `02f0dfc` | third-review fixes (ahead veto on the latest fresh fix, wider `heard_inside` window, sync only while keeping the slot) |
| `a19c8fe` | status update: the simulation blocker |

**Working tree:** `formal/results/mutex.json` is modified. The formal runs regenerate it; it is not a deliberate
change. Restore it with `git checkout formal/results/mutex.json`, or re-generate and commit it together with the
formal re-run of Section 11. `formal/results/formation.json` is regenerated the same way.

---

## 6. Criteria v3 and the referee (implemented)

### 6.1 Files

- `holo_fleet/referee/criteria.py` (new):
  - constants `CRITERIA_VERSION="v3"`, `E_TOL_M=1.0` [REQ], `GIVEWAY_OFFSET_M=2.5` [PROT], `DETECT_S=0.35` [PROT]
    and `FS_QUANTILE=0.99` [LIT];
  - functions `t_hold_s`, `progress_tol_m`, `profile_family`, `permanent_faults`, `nominal_end_s`, `plan_targets`,
    `formation_judged`, `expects`, `failsafe_allowance_s`, `deadline_terms` and `deadline_v3`.
- `holo_fleet/referee/terminal.py`: `TerminalMonitor(scenario, cfg, names, deadline, t_nom=None)`, the online
  monitor. It uses the same requirement set as the offline classifier.
- `holo_fleet/referee/gate_events.py` (new): `gate_events(t, gf, states, healthy, gate_id, plans, cfg)`.
- `holo_fleet/referee/robustness_metrics.py`:
  - `analyze_run` replays the terminal monitor first (`replay_terminal`) and cuts every metric at the replay's t_end;
  - it recomputes P1 and P2 from positions;
  - it computes the gate events and the verdict.
- `holo_fleet/referee/referee.py`: `Referee(..., faults=None)`. Scripted permanent faults define the absent set
  exactly.
- `holo_fleet/runner.py`:
  - deadline = `deadline_v3`, `t_nom = nominal_end_s(sc)`;
  - `_error_status` labels an exception as SUT_ERROR (raised in control, perception or ha) or INFRA;
  - `run_status.json` is written, then the exception is re-raised.
- `scripts/reevaluate_v3.py`: offline re-evaluation of the historical runs. Its ProcessPool died silently: run it
  single-process, or in small batches.

### 6.2 Deadline v3 (no term is derived from runs of the system under test)

`deadline = T_nom + gate + formation + avoidance + faults + failsafe`:
- **T_nom** [PROT]: the instant the plan clock of every drone reaches the end of its path, holds and jumps included.
- **gate** [PROT]: `Σ_k (|queue_lateral_k| + exit_s − queue_s)/v_pass + t_occ_max + t_clear`, minus the planned
  dwell, floored at 0.
- **formation** [PROT/LIT]: `t_rejoin + T_hold`, for plans that keep a slot.
- **avoidance** [PROT]: `2·2.5 m / (v_slot_max − v_plan)`, for single-drone plans (the give-way detour).
- **faults** [PROT]:
  - permanent: `t_rejoin·(1 − v_wait/v_nominal)`;
  - temporary, of duration D: `(v_nominal + w_drift_max)·D / catch_up`.
- **failsafe** [PROF]: for each outage process of the profile (sonar blackouts, DVL outages), the 99 % Poisson
  quantile of its count over the horizon × (outage length + 0.35 s).

DEADLINE means "not done within this allowance". It is not proof of a deadlock. Liveness is judged separately
(Section 6.3). Completion times are compared with Kaplan-Meier curves, censored at the deadline.

### 6.3 Gate events (STPA framing; each decision is judged against the truth at row i−1)

- **E1** — false-FREE and false-BUSY belief time of queued drones.
- **E2** — a premature BUSY→EXITING/FREE while the CR is occupied, by evidence kind. `E2.to_free` counts any release
  to FREE while the CR is occupied.
- **E3** — a commit while another drone is in the CR.
- **E4** — an entry admission while another drone is in the CR. It is the stage-2 ADMIT when logged (key
  `gate.entry`), otherwise the commit.
- **E5** — simultaneous occupancy samples, i.e. a P2 violation.
- **E7** — a stage-2 hold started while the CR is occupied: the unsafe approach was detected and cancelled.
- **E8** — passages through the CR cross-section, bypass, waits, entry gaps and dwell.
- **Liveness**, bounded response (Dwyer et al. 1999):
  - **idle:** all unpassed healthy drones are queued, the CR is empty and nobody is committed, for longer than
    `T_release = t_occ_max + t_clear = 26 s`;
  - **committed:** a passage takes longer than `L_k/v_pass + T_release`;
  - **A-cross:** every dwell ≤ `t_occ_max` (a validity flag).

### 6.4 Classes, in priority order

INVALID / SUT_ERROR (infrastructure or a software exception) > REPLAY_MISMATCH > SAFETY_FAILURE (P1 or P2) >
UNSAFE_ACTION (E3/E4 > 0, or E2.to_free > 0) > SPEC_VIOLATION > STALLED > PASS > CENSORED > SAFE_DEGRADED > INCOMPLETE.

- SAFE_DEGRADED is causal: the drones in sensor-fault FAILSAFE are the noise drones and explain every missed
  requirement. It is never given for CLEAN.
- Cause tags include GATE_BYPASS and A_CROSS_VIOLATED.

### 6.5 Open referee items (from the G.1 re-review; verify each one)

- **M2** — there is no generic no-progress detector outside the gate. **[PROPOSED]** A progress function per healthy
  drone (along-path position, gates passed, formation error) that does not improve for T_stall while the drone is
  not in a timer-bounded wait. T_stall must come from a protocol bound, not be tuned.
- **M3** — a limit cycle whose in-band phase is longer than 8.9 s passes. This is a stated limit of the finite run.
- **M10** — contacts and the intruder flag are still taken from the full log instead of the replay horizon. Fix it.
- **N5** — the historical re-evaluation is almost entirely CENSORED, because the old logs end before the new T_nom.
  Report that; do not "fix" it.
- **N7** — the idle-stall check stays off after a bypass. Fix it.
- **N10** — the avoidance term is a loose upper bound. **N11** — the FAILSAFE allowance covers only blackouts and DVL
  outages, per drone. State both in the criteria document.
- **Minor:** P3-deg for n < 3; the p1_two_lines formation term; the t_occ_max term for the first passage.
- **Methodological review still required:** an independent reviewer must check E_TOL, T_HOLD, progress_tol, the
  deadline terms and the class order for consistency **before the freeze** (user requirement 4).

---

## 7. Formal guarantees and their assumptions (current state)

Run all checks with `python formal/check_properties.py`, using the `holo_fleet_ha` interpreter. Results go to
`formal/results/SUMMARY.md` and the `*.json` files. The README count is 205 checks.

Individual checkers:
- `check_separation.py` (S0-S4, S1o, S1d);
- `check_mutex.py` (M1-M4, M3, M3f, M4g);
- `check_formation.py` (F1-F6, F6r);
- `check_acoustic.py` (B2-A1, B2-A2/A3, B2-M4g, B2-S);
- `check_observations.py`;
- `check_determinism.py`;
- `check_stage2.py` (M4h).

**P1:**
- **S0** (distance bound): randomized, about 35 000 scenes. It is evidence, not proof.
- **S1a** (BMC, 4 s) and **S1b** (k-induction, k = 13) assume that **both** drones detect each other and brake.
- **S1o** (one-sided braking, at the configured `a_brake = 0.5`) is **SAT**: the smallest verified d_warning would be
  2.56 m > 2.4 m.
- **S1d** (one-sided braking, at the calibrated 0.743 m/s², `results/calibration/plant_calibration.json`) is
  **UNSAT**. The one-sided claim therefore rests on the simulator calibration.
- **S2** (exhaustive over the sector patterns), **S3** (proof) and **S4** (blackout ≤ 0.8 s from 2.34 m, proof).
- Assumption (D): a hard-edged cone with full detection. The echo envelope is **exceeded under LOW and MEDIUM**:
  - LOW: a per-run mean of 0.86 % of captures beyond +eps_far, data age up to 0.3 s > tau_max;
  - MEDIUM: a mean of 3.6 %.

  P1 under LOW and MEDIUM is therefore **tested, not proved**.

**P2:**
- **M1/M2** (precedence uniqueness): proof.
- **M4a-f** (geometry): grid sweeps.
- **M4g** (exit observability): geometric only.
- **M3** (latch): BMC to 42 s; still UNSAT at 65 s. Its premises are falsified by the campaign:
  - a healthy crossing at ≥ V_LO without stopping (dwell up to 12.5 s observed);
  - perfect corridor detection (9 MEDIUM crossings missed);
  - sound witnesses (69 of 260 MEDIUM witnesses match no drone; 7 premature releases).
- In the campaign P2 held through **kinematic margins and precedence**, not through the latch argument.
- **M3f**: a stalled occupant gives a counterexample (scope).
- **M4h** (`check_stage2.py`): **only about 16 % of the CR is observable from the hold point**. The far part is masked
  by the bars, so the stage-2 sonar check has limited value.

**P3:**
- **F1-F3** (ranking functions) concern each drone's **own estimated slot error against its own private reference**.
  They do **not** imply recovery of the true formation; navigation error and inter-drone clock consistency are not
  modelled.
- **F4** (mode selection): proof.
- **F6/F6r**: each drone individually declares a never-returning neighbour vacant within t_rejoin. Agreement across
  drones is **not** proved.

**B2:**
- **B2-A1** (envelope soundness): randomized.
- **B2-A2/A3**: grid sweep.
- **B2-M4g**: holds for n = 2 and 3.
- **B2-S** (static isolation): checked.

Formal obligations still to discharge (plan, Section I) are in Section 11.

---

## 8. Experimental evidence: what is valid

**[FACT]** The historical campaign `results/robustness/raw_final` was run at commit `ad4fb5e` and classified under
criteria v2: 624 runs, B0 and B2.

**Valid as historical evidence for `ad4fb5e` only:**
- 0 P1 and 0 P2 violations;
- minimum separation 2.26 m;
- 0 of 536 commits while the CR was occupied;
- minimum entry gap 11.5 s.

**Diagnosed defects** (plan, Sections D and E):
- 7 premature releases;
- 69 of 260 MEDIUM sonar witnesses match no drone;
- 9 missed crossings;
- 8 committed stalls before the CR (100-260 s);
- 1 phantom-echo queue stall (204 s);
- 11 clock-divergence P3 failures;
- 5 inconsistent-vacancy P3-deg failures;
- B2_CLEAN idle stalls ≥ 20 s in 12 of 15 gate runs (B0 CLEAN: 1 of 15; BLACKOUT-CLEAN: 0 of 15).

**Statistics:** B2_MEDIUM vs BLACKOUT, McNemar 7 vs 3, p = 0.344. The claimed acoustic benefit is **not**
established.

**Stage-2 false holds** (from the logs of 624 runs):
- CLEAN and LOW: none;
- MEDIUM: 3-19 % of samples;
- the latch is not FREE in 11-55 % of MEDIUM samples, so it is not used at stage 2.

**Not valid for the new code:** every historical result of the gate, formation and vacancy behaviour. G.3 and G.5
changed that logic, so the gate and formation results must be re-run.

**Invalid (do not use):**
- `results/robustness/raw_dev/probe3`, `probe4` and `r02f0dfc_b`. They ran with concurrent HoloOcean instances, or
  died in infrastructure.
- `results/robustness/v3_reeval/*` is a partial offline re-evaluation (the pool died). Regenerate it.

**Supporting files:**
- `analysis/p3_failures.txt`;
- `analysis/witness_crossings.txt`;
- `analysis/forensics_v.txt`;
- `results/robustness/final/{SUMMARY.md,cells.csv,paired.csv,runs.csv}`;
- `analysis/litreview/`.

---

## 9. Reviews: findings, fixes, withdrawals

### Reviews A and B (analysis phase)

The files are `analysis/review_A_formal_hybrid.md` and `analysis/review_B_marine_experimental.md`. They were verified
in plan Section B.1, and their claims are integrated into Sections 7 and 8 above.

### G1 review (criteria v3)

Findings:
- B1: gate bypass counted as a passage;
- M1-M12:
  - SAFE_DEGRADED was not causal;
  - stalls did not block PASS;
  - the hold time had no derivation;
  - the FAILSAFE allowance was circular;
  - T_nom;
  - the deadline terms;
  - the stall detector;
  - E2/E4 decision timing;
  - SPEC_VIOLATION;
  - the horizons;
  - SUT_ERROR;
  - the progress latch.

All were fixed in `92a72e5`.

The re-review found N1 (a T_nom mismatch online vs offline, blocking), N2 (slow committed passages), N3 (no
REPLAY_MISMATCH class), N4 (setup errors), N6 and N9. They were fixed in `05e1c5e`. The open items are in Section 6.5.

### G345 review (behaviour changes), with a re-review

- **B1, blocking** — the stage-2 retreat and retry left the drone `committed`. It re-approached without the stage-1
  check, so two drones could be committed at once. **[WITHDRAWN]**: only a logged hold timeout remains. This is
  **Issue 1**.
- **B2, blocking** — the identity-assisted map fit dropped every bar echo near the gate, because the acoustic regions
  are too wide under the deployed envelope. **[WITHDRAWN]**: `structure_aiding.py` was reverted to `0d1b7cd`. This is
  **Issue 2**.
- **M1** — the `heard_inside` veto had a gap after one lost ping. It was fixed with `fresh_s`, then the window was
  widened (re-review N2). This is **Issue 3**.
- **M2-M4** — the "ahead counts as present" rule:
  - it fed unbounded residuals into the consensus;
  - it could keep a drifting failed drone from ever being declared vacant;
  - its 3-5 m certainty dead bands kept the positive feedback alive.

  It was redesigned, and the re-review found three more problems:
  - N1: the veto was ineffective at the 1 s ping period;
  - N3: A-D used the unbounded set;
  - N4: sync must act only while keeping the slot.

  All were fixed in `02f0dfc`. This is **Issue 4**.
- **M5 and M6** were resolved by the withdrawals.
- **A4** is not tested under the deployed envelope (minor).
- **N6**: the docs were fixed.

---

## 10. The four issues Codex must complete

For every issue, closure requires all of the following:
1. a written design with its justification (literature, physics, calibration or a requirement);
2. unit tests, including mutation-style tests that fail on the defective variant;
3. formal obligations updated and re-run;
4. targeted HoloOcean diagnostics, before and after, at least 5 repetitions per case (engine noise is unseeded);
5. no regression in P1, P2 or gate liveness;
6. an independent review with its findings resolved;
7. a short entry in `analysis/CONSOLIDATION_STATUS.md` (problem, solution, evidence).

### Issue 1 — Unsafe return to the gate queue (abort/retry an entry)

**State.** `holo_fleet/control/flows.py`, gate stage 2:
- the hold at `s_hold` is entered when `loc_ok and entry_ok` is false;
- after `t_occ_max + t_clear` it logs `ENTRY_HOLD_TIMEOUT` once (attributes `hold_timeout`, `entry_state`, `events`,
  drained by the controller);
- there is **no retreat**, so the drone holds indefinitely if the condition never clears.

Automaton modes: `Mode.MUTEX_APPROACH`, `MUTEX_YIELD` and `MUTEX_PASS` in `holo_fleet/ha/spec.py`. Guard G2 forbids
PASS → YIELD.

**Do not reintroduce** a retreat implemented inside the flow while `committed` stays true.

**Required design [PROPOSED]:**
- A real automaton edge MUTEX_PASS → MUTEX_YIELD (or → MUTEX_APPROACH) that **clears `committed`** and releases
  priority.
- It is allowed only while the drone is provably before the CR entry plane:
  `s < −(cr_half_len + R_OUT + margin)`. The margin must be derived from the navigation-error bound, not tuned.
- The drone then re-queues through the normal stage-1 path: precedence, latch, `t_clear`.
- It needs a bounded retry counter, then an explicit degraded outcome (FAILSAFE outside the corridor, logged).
- Precedence after an abort must stay unique. Decide and verify whether the aborting drone keeps or loses its queue
  position, so that M1/M2 still yield exactly one leader.

**Formal work:**
- restate G2;
- run D1, D2, L1, H1-H3 and Q1 on the new edge;
- re-run M1/M2;
- an M3 variant with the hold and the abort;
- a ranking function for the retry counter plus the timer;
- M4b/M4s merge clearance for the return path.

**Evidence to close:**
- no state with two drones committed, as a formal check plus a randomized trace test;
- E3 = E4 = E5 = 0 in the targeted runs;
- every abort is explicit, outside the CR, and followed by a passage or an explicit degraded outcome;
- no committed stall longer than the crossing budget.

### Issue 2 — Acoustic-assisted gate localisation

**State.** `holo_fleet/perception/structure_aiding.py` (and `tests/test_structure_aiding.py`) are back at `0d1b7cd`:
the safe C5 restriction, no fit beside the opening.

The motivating defect remains: 8 committed stalls in which navigation error near the gate (0.3-1.0 m) leaves the bars
unexplained.

**Do not reintroduce** the identity-assisted fit that drops echoes "a drone could have produced" using the full
acoustic uncertainty regions. Under the deployed envelope those regions cover the bars, so every bar echo was
dropped.

**Options to evaluate [PROPOSED], in order:**
1. **(a) Observability-aware approach.** Merge onto the gate axis at about s ≈ −4.2 m (the acquisition distance), so
   that the side sonars see both pillars. Check it first by replaying logged geometry.
2. **(b) Exclusion with point estimates.** Exclude only echoes that lie within `k_sigma·σ_w(r)` of a fresh identified
   fix's **point estimate**, and never echoes that match a mapped bar within the sonar range error. Accept it only if
   a replay shows that bar echoes survive in at least the CLEAN, LOW and MEDIUM logs.
3. **(c) A bounded lateral sweep** before the CR, inside `struct_keepout` and the separation layer.

If none of these is verifiably better, keep the current method and document the residual stall risk. Issue 1's abort
then bounds the consequence.

**Evidence to close:**
- an offline replay of the 8 stall traces;
- the fraction of bar echoes kept;
- the correction error versus ground truth;
- targeted runs gate_single MEDIUM-ONE s0 and lost_drone_mutex MEDIUM-SUBSET s0 with no STALLED and no new structure
  contacts.

### Issue 3 — Acoustic occupancy veto (`heard_inside`)

**State [IMPL-UNVAL].** `holo_fleet/perception/acoustic_gate.py`:
- `heard_inside` uses only the clear-path region (`_inside_clear_path`: region points with range ≥ r − br, i.e. no
  one-sided multipath excess);
- it needs two accepted fixes on one track, the latest no older than `acoustic.fresh_s = 2.5 s`;
- the search window is `t − 2·period − fresh_s`.

It vetoes:
- the `t_occ_max` timeout fallback;
- the stage-2 entry (`GateObs.entry_ok = quiet and not heard_inside`, `perception/gate_perception.py`,
  `ENTRY_QUIET_CAPTURES = 3`).

**Rule A4:** with every other drone excluded at t_lc and positively outside now, the phantom corridor echo is
released. Tests are in `tests/test_acoustic_b2.py` (heard_inside, a lost ping, beyond-the-gate, A4).

**To verify:**
- **Veto completeness (B2-V1, [PROPOSED] formal):** a hull in the CR with in-envelope fixes gives `heard_inside`,
  under dropout, latency up to `latency_max_s`, multipath and σ. Quantify the probability of a veto gap from the
  channel model's dropout process (`sim/acoustic_channel.py`, `docs/ACOUSTIC_CHANNEL_MODEL.md`). Fix the §9
  inconsistency noted in that document.
- **Liveness:** the veto cannot block indefinitely. A drone beyond the exit or outside the CR must not keep the veto
  alive. Show a bound: once the occupant leaves, the veto ends within fresh_s + 2·period.
- **A4** under the deployed envelope (an open item).
- **Targeted runs:**
  - mutex_deadlock_resolution LOW s2;
  - integrated_short MEDIUM-SUBSET s2;
  - gate_single MEDIUM-SUBSET s1;
  - the new **gate_stalled_occupant** scenario **[PROPOSED; it does not exist yet]**: a thruster fault inside the CR,
    built from existing primitives, under CLEAN, MEDIUM and BLACKOUT.
- **Evidence to close:**
  - E7 > 0 and P2 held in gate_stalled_occupant;
  - E4 = 0;
  - B2_CLEAN idle waits no longer than BLACKOUT-CLEAN;
  - no new idle stall.

Remaining G.3 work: (b) the witness 3-capture confirmation with a range evolution consistent with a leaving hull,
plus a B2 veto of the sonar witness while `heard_inside`.

### Issue 4 — Formation synchronisation, neighbour identification, vacancy, rejoining

**State [IMPL-UNVAL]:**
- `holo_fleet/perception/perception.py`, `acoustic_neighbours`:
  - **A-P** (present): back to the v2 rule.
  - **"Ahead" veto:** the latest fresh fix is on its lane (lateral ≤ `e_lost`) and ahead by more than
    `e_lost + band`, with `band = k_sigma·hypot(σ_r, r·σ_az)`. It only blocks starting a **new** missing episode
    (`track_missing`), for at most t_rejoin (`ahead_since`); it never clears an existing one. The set is
    `acoustic_ahead`.
  - **Lag:** the median along-track offset of the `A_CONFIRM` fixes of one track, beyond the band, stored in
    `acoustic_lag`.
  - **A-D** (displaced) excludes `acoustic_ahead`.
  - `FormationObs.behind` = the maximum lag over the non-vacant neighbours (`perception/formation_perception.py`).
- `holo_fleet/control/formation_time.py`, `update(..., behind=None)`:

  ```
  sync = 0 if (mutex or behind is None or not moving or not keeping)
         else min(k_progress·behind, v_plan − v_wait)
  ```

  The target rate is `min(reduced, (v_plan − sync)/v_plan)`; `sync` is recorded in `record`. This is a first-order
  progress consensus (Ghabcheloo et al. 2009). The docstring has stale wording; fix it.
- `holo_fleet/control/controller.py` passes `behind=obs.form.behind`.
- Tests are in `tests/test_acoustic_b2.py` (ahead, lag and veto at the 1 s period; the resync bounds) and
  `tests/test_formation.py`.

**To do:**
- **G.6 [PROPOSED]:**
  - re-occupation of a vacant slot only by an identified fix of that slot's drone inside the slot region;
  - "positively absent" = the last fix grown by `v_bound·age` is still farther than `e_lost` from the slot;
  - under outage, the sonar rules, documented as degraded.
- **Formal [PROPOSED]:**
  - F6′ for the "ahead" veto (vacancy still declared within a bound);
  - a pairwise contraction lemma (|τ_i − τ_j| decreases while both perceive each other within the envelope);
  - F1-F3 and F5 with v_ref in [v_wait, v_plan];
  - B2-A1 extended to the slot regions.
- **Tests still missing:** gradual drift; the false-sync rate; one test reported as vacuous by the reviewer (find it
  by mutating the sync term and checking which tests still pass).
- **Targeted runs:**
  - formation_six MEDIUM-ALL s1;
  - formation_square MEDIUM-ONE s2;
  - lost_drone_timeout MEDIUM-SUBSET s2;
  - formation_six BLACKOUT, MEDIUM-ALL s2.
- **Evidence to close:**
  - max |τ_i − τ_j|·v_plan < 0.5 m at the end (plan G.5);
  - no clock-divergence failure;
  - false-missing and false-vacancy rates reported;
  - P3 or P3-deg reached under the 1.0 m tolerance, with P1 and P2 unchanged.

---

## 11. Other open items and formal obligations

- **G.2** (B2 default):
  - change `BeaconConfig.mode` to `"ACOUSTIC_RB"`;
  - keep explicit outage tests;
  - add a silent-channel equivalence test (B2 with BLACKOUT gives the same decisions as the old sonar-only path);
  - remove the `B0` protocol from the campaign script but keep the archived results, with a manifest;
  - re-label M3, M4 and S* as outage-mode evidence.
- **B2-S** data-flow re-check after every change.
- **A full formal re-run** after the changes: `python formal/check_properties.py`. Update `formal/results/*`,
  `docs/GUARANTEES.md` and the README count.
- **Plan Section I obligations** that are not yet done:
  - G1 with `entry_ok`;
  - O1-O5 / I1-I4 for `entry_ok`;
  - M3 re-encoded with spurious witnesses, missed detections and in-CR holds (bounded nondeterminism), with the
    horizon justified (reviewer A: 51.7 s for n = 3) or replaced by induction;
  - M3s (a stalled occupant blocked at stage 2);
  - B2-V1;
  - B2-X (A1/A2 exclusion of all drones implies no drone in the CR);
  - an N = 2-3 timed model of the queue, latch, stage 2 and abort, with conformance against the logged traces;
  - relabel the "exhaustive numeric" sweeps as grid sweeps, or bound their discretisation error.

---

## 12. Infrastructure: the orchestrator failure and isolated parallel HoloOcean

### 12.1 Verified facts about HoloOcean IPC [FACT, read from the installed client]

The installed client is `F:\Andrea\holoocean-octree-patch\client\src\holoocean`, version 2.3.0. It is imported by the
env `C:\Users\andrea.bedei3\.conda\envs\holo_fleet_ha\python.exe`. The engine root is `F:\Andrea\holoocean_patched_root`,
set through `HOLODECKPATH` by `holo_fleet/sim/holoocean_setup.py::use_patched_holoocean`, or overridden with the
`HOLO_FLEET_HOLOOCEAN_ROOT` environment variable.

- **The client-engine channel has no network port.**
  - `holoocean.make()` (`holoocean.py`) sets `param_dict["uuid"] = str(uuid.uuid4())` for each environment. **There
    is no port parameter.**
  - The engine is launched (`environments.py`, the Windows start) with `--HolodeckUUID=<uuid>`, plus `-LOG=HolodeckLog.txt`,
    `-TicksPerSec` and `-EnvMin*/Max*`.
  - The IPC objects carry the UUID in their names:
    - named semaphores `Global\HOLODECK_LOADING_SEM<uuid>`, `Global\HOLODECK_SEMAPHORE_SERVER<uuid>` and
      `Global\HOLODECK_SEMAPHORE_CLIENT<uuid>`;
    - shared memory `/HOLODECK_MEM<uuid>_<name>` (`shmem.py`).
  - Each client therefore connects only to the engine it launched. `tests/test_engine_isolation.py` pins this:
    `test_every_environment_gets_a_fresh_uuid`, `test_semaphores_and_shared_memory_are_named_by_the_uuid` and
    `test_the_engine_is_started_with_the_uuid_and_no_port`.
- **The engine does listen on one TCP port, which the client never uses.** It is Unreal's TraceLog control port:
  1985, or a free high port when 1985 is taken (docstring of `tests/test_engine_isolation.py`).
  - The orchestrator records it in `run_metrics.json` → `process.engine_listen_ports`.
  - In `raw_final`, the shards that ran concurrently (`process.shard = [i, 3]`) show 1985 in about 40 % of the runs
    and ports 32 000-41 000 in the others. Several engines therefore coexisted historically.
  - **Consequence for the user's "non-default port" requirement:** HoloOcean 2.3.0 has no configurable client port.
    Isolation is by UUID. The only port is Unreal's TraceLog listener, and 1985 is its default.
  - **[PROPOSED]** To honour the requirement, investigate whether the Unreal command line accepts an option that
    disables TraceLog or sets its port. Candidates to verify, not assumed: `-notrace`, `-trace=none`,
    `-tracehost`. The client passes no extra arguments today, so a change would go in the patched client's launch
    arguments or through a wrapper. Verify with `psutil` `net_connections` that our engine does not bind 1985. Never
    rely on a Python parameter alone.
- **Shared resources not keyed by the UUID [FACT, to verify for isolation]:**
  - the engine log `-LOG=HolodeckLog.txt` goes to `<root>/Windows/Holodeck/Saved/Logs/` of the **same root** for every
    instance, so concurrent engines overwrite or interleave the log;
  - the octree cache (`holoocean_setup.octree_cache_dir`);
  - Unreal's `Saved/` directory in general.

  **[PROPOSED]** Options: one patched root copy per slot (for example `F:\Andrea\holoocean_patched_root_slot{0,1}`,
  selected with `HOLO_FLEET_HOLOOCEAN_ROOT`), or a per-instance `-LOG=` file name and an `-abslog=` path. Verify that
  the engine honours them.
- **Other project facts:**
  - the client start-up race on Windows is patched by `wait_for_engine_semaphores`;
  - `holo_env.HoloEnv._kill_own_engines` kills only `Holodeck.exe` processes **whose parent PID is this process**;
  - `sim/engine_watchdog.py` calls `os._exit(3)` when our engine dies;
  - the client's `__on_exit__` kills its own `_world_process`.

### 12.2 The orchestrator failure (unresolved)

**Symptom.** Workers started by the orchestrator (`scripts/run_sensor_robustness_campaign.py`, which launches each
worker with `subprocess.Popen([... "--worker" ... "--terminal"])`) died:
- about 15.7 s after launch, 2-3 s of simulated time into the loop;
- exit code 1 (3 cases);
- no Python traceback;
- `finally` blocks not run (`events.jsonl` empty);
- the engine log cut off without a shutdown.

One case exited with **3221225794 = 0xC0000142 (STATUS_DLL_INIT_FAILED)**: `results/robustness/raw_dev/r02f0dfc_b.log`,
lost_drone_mutex, at 00:03:13.

**What works:**
- a foreground bash worker with `--terminal` (gate_single, COMPLETE to t = 160 s);
- the in-process `runner.run` (B0 and B2);
- minimal HoloOcean probes.

The failure also appears at `a6efd0f`, the code before the consolidation. **Do not attribute it to the consolidation
code without evidence.**

**Confound.** Some earlier probes ran concurrently with the orchestrator loop, and reviewers may have run
`tests/test_octree_patch.py::test_octree_rebuild_regression`, which launches the engine. The earlier evidence is
therefore unreliable.

Long pytest runs and a ProcessPool (`scripts/reevaluate_v3.py`) also died silently ("A process in the pool was
terminated abruptly"). Per-file test runs pass.

**Ruled out:** RAM, disk, Defender, GPU memory (42 of 49 GB free), the Claude sandbox, z3 (not used at runtime). No
kill command was found in the transcripts.

**Hypotheses [HYP], test one at a time, with nothing else of ours running:**
1. **Desktop heap or window-station exhaustion** for processes started without a console or from a background
   session. 0xC0000142 is the classic symptom.
   - Test: launch the same worker command from the orchestrator with `creationflags=CREATE_NEW_CONSOLE`, or
     `CREATE_NO_WINDOW`, versus the current setting.
   - Count the desktop objects (GDI/USER handles) of the user session.
   - Check whether the other interactive programs (CARLA, the user's jobs) hold many windows.
2. **Priority class or affinity.** The orchestrator's `be_nice` and `lower_children` set BELOW_NORMAL, low I/O and
   `cpu_affinity` on the worker and its Holodeck. Run the orchestrator with that call disabled by a flag. Do not edit
   the defaults until the cause is known.
3. **Stdout/stderr redirection to a file** with `PYTHONUNBUFFERED` versus a console. Compare `stdout=wl` with an
   inherited console.
4. **External termination by a process outside our tree** (a job object, an endpoint agent such as
   `CodexSandboxService`, or a parent shell's job object killing its children when the tool call returns).
   - Record the parent chain, the job object membership (`IsProcessInJob`) and the exit code of both the worker and
     its engine.
   - Run the orchestrator itself detached (`DETACHED_PROCESS | CREATE_NEW_PROCESS_GROUP`, or a Windows Scheduled
     Task) instead of as a child of an agent tool call.
   - The "dies after the tool call returns" pattern fits background tool calls.
5. **The engine dies first, and the watchdog exits with 3.** Exit 1 does not match this, but check the engine exit code.

**Instrumentation [PROPOSED]**, kept in a separate diagnostics script, not in the runtime:
- a `faulthandler` dump to a file;
- `atexit` and `sys.excepthook` logging;
- a heartbeat file every second;
- the exit code of every process in the tree;
- a `psutil` snapshot of the parent chain;
- Windows Event Log entries (Application, System) at the time of death.

**Closure:** a reproducible explanation, plus an orchestrator run of the 11 cases of `scripts/run_dev_diagnostics.sh`
with no infrastructure death.

### 12.3 Progressive validation of isolated parallel instances (user requirement; at most 2 of ours)

Work one step at a time. Advance only when the previous step is clean.

1. **One instance on a non-default TraceLog port** (or with TraceLog disabled). Verify with `net_connections` that it
   binds neither 1985 nor any port of another process. Record the UUID, the PIDs and the root.
2. **One complete representative scenario** on that configuration, for example gate_single CLEAN s0 with `--terminal`.
   Expected: COMPLETE, P1 and P2 held, and logs in its own directory.
3. **Two instances** with distinct UUIDs, distinct ports or TraceLog disabled, and distinct roots (or distinct log
   paths).
4. **Each client talks to its own engine.** Check that the UUID in the engine command line (`psutil.Process.cmdline`)
   equals the client's `env._uuid`, and that the agent names and scenario of each instance appear only in its own
   logs.
5. **Two different scenarios at the same time.**
6. **Isolation checks:**
   - both runs complete;
   - separate logs and engine logs;
   - no shared-file write conflicts;
   - results statistically consistent with single-instance runs of the same case (the engine noise is unseeded, so
     compare distributions, not bits);
   - RAM, GPU and CPU stay within limits and the other workloads are not slowed (measure them).
7. **The orchestrator with two slots** (`--shard 0/1 --nshards 2`, one per slot, or an orchestrator-level semaphore).

If two-instance operation is unstable at any step, use **one** instance with the validated configuration.
Correctness and isolation come before throughput.

**Required safeguards [PROPOSED]:**
- **A project-wide instance limit.** A lock-file semaphore with 2 slots, for example under `results/.holo_slots/`.
  Each slot file records the owner PID, the process creation time and the UUID. Every launcher acquires it in
  `HoloEnv.start()` before `holoocean.make()`. That covers the orchestrator workers, `runner.run`, `scripts/run_demo.py`
  and the tests, so nothing can start an engine without a slot.
- **A stale-slot check.** A slot is stale only when its PID no longer exists **or** that PID's creation time differs
  from the recorded one. **Never terminate a process because its PID matches an old log entry.**
- **Engine-launching tests are opt-in.** `tests/test_octree_patch.py::test_octree_rebuild_regression` (and anything
  that calls `holoocean.make`, e.g. `tests/test_run_demo.py`) run only with an explicit environment variable such as
  `HOLO_FLEET_ENGINE_TESTS=1`. Reviewers are told never to set it.
- **Cleanup only for our own tree.** Kill a `Holodeck.exe` only if its parent is our worker and its command line
  carries our UUID. The current `reap()` and `_kill_own_engines()` check the parent only; add the UUID check.
- **Port reservation.** Before a launch, check that the intended TraceLog port is free; after it, check that our
  engine bound the port we intended and nothing else.

### 12.4 Running tests safely

- Run the tests file by file with `-p no:cacheprovider` and
  `--deselect tests/test_octree_patch.py::test_octree_rebuild_regression`. Skip `tests/test_run_demo.py` unless an
  engine slot is free. A file whose pytest process dies without a summary is retried once. The scratch runner used
  so far (not in the repo) is:

  ```bash
  cd /f/Andrea/HoloHybridAutomaton
  PY=~/.conda/envs/holo_fleet_ha/python.exe
  for f in tests/test_*.py; do
    [ "$f" = "tests/test_run_demo.py" ] && continue
    "$PY" -m pytest -q -p no:cacheprovider "$f" \
        --deselect tests/test_octree_patch.py::test_octree_rebuild_regression < /dev/null
  done
  ```

  Commit an equivalent as `scripts/run_tests_by_file.sh` if it is useful.
- Formal checks: `"$PY" formal/check_properties.py`. Restore or commit the regenerated `formal/results/*.json`
  deliberately.
- Never run the full suite, or the formal suite, at the same time as a simulation.

---

## 13. Validation campaign (after every issue is closed)

1. Unit and integration tests pass file by file. The formal suite passes with the expected verdicts.
2. Independent reviews of each behavioural change are resolved.
3. Targeted diagnostics (plan H.2): the cases in `scripts/run_dev_diagnostics.sh` plus gate_stalled_occupant.
   - Use the 3× horizon for exploration only.
   - Run 5 repetitions per case, before and after each change.
   - Run with `bash scripts/run_dev_diagnostics.sh results/robustness/raw_dev/<name>`; the script uses
     `--phase targeted --only --protocols --profiles --seeds --raw --cooldown-s 2`.
   - Compare with the historical raw_final cases.
4. Verify: no safety regression (P1, P2, E3/E4/E5 = 0), no new gate deadlock (idle or committed stalls), formation
   recovery, and degraded operation.
5. Reference runs (H.3): B2 CLEAN, 16 scenarios + gate_stalled_occupant, 5 seeds, at the candidate commit.
6. **Freeze, before any final run:**
   - the code commit;
   - `docs/ACCEPTANCE_CRITERIA.md` v3 (with the post-freeze change ad4fb5e and the discarded stage-E runs recorded);
   - the **held-out set**: seeds such as 100-109, plus one held-out gate scenario built from existing primitives.
   Record the frozen commit in every run's provenance.
7. **The final B2-only campaign** (plan H.4), not to be adjusted for favourable results. 8 cells (sensor profile /
   channel):
   1. CLEAN / CLEAN;
   2. LOW-ALL / LOW;
   3. MEDIUM-ONE / MEDIUM;
   4. MEDIUM-SUBSET / MEDIUM;
   5. MEDIUM-ALL / MEDIUM;
   6. CLEAN / MEDIUM;
   7. CLEAN / BLACKOUT;
   8. MEDIUM-ALL / BLACKOUT.

   Each cell has 11 non-gate scenarios × 5 seeds plus 6 gate scenarios (with gate_stalled_occupant) × 10 seeds, i.e.
   115 runs per cell and **920 runs** in total. The campaign script must be extended to plan exactly this set. Use at
   most 2 verified, isolated instances (Section 12.3), at low priority.

   The plan's smaller variant (3 and 6 seeds, about 560 runs) may be used only if it is chosen **before** the runs,
   with the reason recorded.
8. **Analysis:**
   - Wilson 95 % CIs per cell;
   - Holm-corrected comparisons between cells;
   - exact McNemar only for the exploratory before/after pairs;
   - Kaplan-Meier completion curves;
   - E1-E8, stall, idle-gate and chattering counts;
   - zero-event bounds (rule of three);
   - the acoustic benefit only as B2 vs B2-outage (cells 5 vs 8, and 1 vs 7), with the pairing called nominal;
   - development seeds and held-out seeds reported separately;
   - separate studies: P1 sensitivity to assumption (D), and the acoustic residual-risk analysis O-12.
9. **The final scientific report** (English, in `analysis/` or `docs/`):
   - the problem, the method and the provenance of every criterion;
   - the formal results with their assumptions;
   - the experimental results, **including failures and limitations**;
   - the 624-run campaign as historical evidence for `ad4fb5e` under v2, with v3 recomputed offline.

   Then update `docs/GUARANTEES.md`, `README.md` and `analysis/CONSOLIDATION_STATUS.md`.

**Final deliverables:**
- the code on `b2-consolidation`, merged into `main` with `--no-ff`;
- the tests;
- the formal results;
- the raw and aggregated results;
- the report;
- the updated docs;
- a concise status file.

---

## 14. Constraints (binding)

- **Processes:**
  - Never terminate, restart, re-prioritise or reconfigure processes that are not ours: CARLA (a separate project's
    `start_carla.py`, not in this repository, running since 02/11), other HoloOcean or `Holodeck.exe` instances, the user's other Python jobs (for example an
    Instagram worker), GPU jobs, `CodexSandboxService`.
  - Never change their ports, environment, configuration, priority or working directory.
  - **No global kills** (`taskkill /IM Holodeck.exe`, `Stop-Process -Name python`, and similar).
  - Kill only processes proven to be ours: parent chain plus UUID plus creation time.
- **At most 2 of our HoloOcean instances at once**, and only after Section 12.3 has been validated. Until then, one.
  Run the orchestrator and the campaign at **low priority** (`be_nice`: BELOW_NORMAL, low I/O).
- **Methodology:**
  - no invented thresholds, metrics, timeouts or success criteria; every constant carries a provenance label;
  - do not change P1 thresholds;
  - no tuning per seed or scenario;
  - do not modify noise profiles or success criteria after seeing results;
  - do not change property definitions to get better results;
  - do not use longer timeouts to hide deadlocks;
  - no absolute guarantees where physical, sensing or environmental limits exist.
- **Evidence:** do not delete unique scientific evidence without a verified copy. Keep `raw_final` (the B0 history
  included).
- **Git:** work on `b2-consolidation`; push without force. Commits are authored by Andrea Bedei only, with no
  co-author trailer. Final merge into `main` with `--no-ff`.
- **Token budget:** stop and report at 90 % of the 5-hour usage window.

---

## 15. Scientific references used for decisions (verified DOIs)

- R. Ghabcheloo et al., "Coordinated path-following in the presence of communication losses and time delays", SIAM
  J. Control Optim., 2009, doi:10.1137/060678993. Progress consensus for the formation clocks.
- N. Leveson, *Engineering a Safer World*, MIT Press, 2012, doi:10.7551/mitpress/8179.001.0001. STPA unsafe control
  actions for E1-E8.
- M. Dwyer, G. Avrunin, J. Corbett, "Patterns in property specifications for finite-state verification", ICSE 1999,
  doi:10.1145/302405.302672. The bounded-response pattern for gate liveness.
- E. Kaplan, P. Meier, "Nonparametric estimation from incomplete observations", JASA, 1958,
  doi:10.1080/01621459.1958.10501452. Censored completion times.
- F. Ceragioli, C. De Persis, P. Frasca, "Discontinuities and hysteresis in quantized average consensus", Automatica,
  2011, doi:10.1016/j.automatica.2011.06.020. Hysteresis and quantisation in consensus.
- K. Ogata, *Modern Control Engineering*. The 2 % settling time of 4 time constants, behind T_HOLD = 4 / k_slot.
- Further sources are in `analysis/litreview/`, `docs/SENSOR_ROBUSTNESS_LITERATURE.md`,
  `docs/SENSOR_DEGRADATION_REFERENCES.md`, `docs/ACOUSTIC_CHANNEL_MODEL.md` and `docs/PARAMETER_PROVENANCE.md`.

Never cite a reference without checking it.
