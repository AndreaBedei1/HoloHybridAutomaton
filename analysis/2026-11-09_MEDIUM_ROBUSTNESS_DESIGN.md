# MEDIUM-ALL robustness: root causes of the v10 failures and design

Branch `feature/b2-medium-robustness`, from `main` (120b400). Baseline: tag `b2-freeze-v10` (9866242), kept immutable.
Status: as implemented at the current branch head, after the first independent reviews. Sections 1 and the root causes are from the v10 logs.

## 1. What the v10 failures are

MEDIUM-ALL: 56/66 PASS outside `gate_stalled_occupant`. Every INCOMPLETE run is a liveness failure; safety held.

**Reproduction (2026-11-09, frozen v10 code, same seeds, 4 engines, `results/robustness/raw_dev/v10_repro`):** of the
first 8 of the 10 failing runs repeated, 7 PASS and 1 INCOMPLETE (`formation_gust` s800). The failures are not determined
by the seed (the engine's sensor noise is unseeded and the run is not bit-deterministic): each MEDIUM run fails with a
probability of about 15 %, and the evaluation must be statistical (large N, fresh seeds), not a replay of ten seeds.

Root causes, from the logs of the ten runs (`results/robustness/raw_b2final_v10`):

* **RC1 (6 runs: integrated_short 800/803/804, lost_drone_mutex 801/802/...). The gate bars are not recognised.** A drone
  holding at its queue point drifts from its own estimate (true error 0.8-1.2 m in the gate frame, the true nav error in
  MEDIUM stays below 1.5 m in all 72 runs, while the declared bound reaches 6-20 m). The map-predicted bar range then
  misses the FRONT echo by more than `structure_tol_m` and the bar echo is classified UNKNOWN/DYNAMIC. It then (a) makes
  a FRONT WAIT relation for ever (`has_prio` false) and (b) sets or refreshes the CR latch BUSY through the corridor
  window. The evidence is direct: lost_drone_mutex s801 drone 2 at t=400: echo 4.92 m; true FRONT-sonar to bar range
  4.94 m; the range predicted from the navigation estimate is 5.50 m. The queue never moves. Also near the bars during
  the approach (integrated_short s804): the unexplained bar echoes keep the committed drone in SEPARATION_WARNING for
  hundreds of seconds and it never reaches the hold point.
* **RC2 (the same runs). The payload rules are unavailable exactly when they are needed.** The 6-bit bound field saturates:
  any bound >= 6.2 m is encoded as 63 = invalid, i.e. the payload disappears ("missing") once `NavBound` reaches 6.2 m,
  which happens to every drone on a long run. NavBound is a sum of worst cases and never decreases. Independently of
  that, a position plus a bound of several metres cannot clear a CR of 1.6 x 2 x 2 m at all.
* **RC3 (3 runs: formation_gust, formation_recovery_head_current, lost_drone_rejoin s800). A recovering drone stops for
  good at the end of the path.** `Flows.formation` rejoin phase DROP_BACK clamps the along-track command to
  `[0, max(v_clock, 0)]`; when the formation reference stops at the end of the plan (`v_clock = 0`) the drone can never
  drop back, and stays on the pass level or off its lane for the rest of the run (slot error constant at 1.8-4.0 m in
  the logs; the slot error of the drone is exactly `pass_dz_m` = 3.5 m). `FALSE_VACANCY` follows from it.
* **RC4 (lost_drone_mutex 801-804). Vacancy is not declared by a drone off its own slot.** The vacancy timer runs only
  while the observer holds its slot in a formation mode; a survivor stuck in FORMATION_RECOVERY or MUTEX_YIELD never
  declares the lost neighbour's slot vacant.
* **RC5 (failure model).** The v10 lost-drone scenarios use a thruster-only failure: the "lost" drone keeps pinging and
  keeps reporting a position. Its neighbours can only infer loss from displacement, and the vacancy logic has to tell a
  dead drone from a delayed one with no information. The owner's failure model is total: propulsion, electronics and
  acoustic transmission stop together, and the (positively buoyant) drone rises.
* **RC6 (found in development batch med3, `formation_gust` s912, `lost_drone_rejoin` s911). The relative navigation error blocks the
  return from behind.** MEDIUM draws a compass bias of +/-2 deg per drone. Two drones with opposite signs differ by 1-2 m laterally
  after a 30 m run. The planned lanes (3.5 m) are then 2.6 m apart in reality; the drone returning along its lane enters the warning
  band of the neighbour on the next lane (trigger at about 3.3 m between centres) and stops 2.3-3.9 m short of its slot until the deadline.
  The sonar consensus could not help: it is disabled beyond `e_lost` because the ranges then measure the drone's own displacement.
* **RC7 (med4 `lost_drone_mutex` s920, also the earlier failures of that scenario). An along-axis error stops the drone before it can relocalize.** On the axis only the FRONT sector
  sees the frame until about 2.2 m. With a true-to-estimated error of 0.5-1.1 m along the axis the bar echo is unexplained, the separation layer holds the drone at about 2.7 m, and
  the side sectors that the map fit needs never see the bars.
* **RC8 (med4 `p1_two_lines` s921). The rejoin logic undid a running give-way.** The lane error ignored the give-way offset; a recovering drone was sent back to the lane it had left,
  head-on again, and stayed in a standoff until the deadline.
* **RC9 (med7 `lost_drone_mutex_carried` s981, found by the underwater review of round 4).** A drone committed before another one died was admitted into a CR that holds its hull, pinned under the top bar: P2 violated, separation 1.18 m. The hull is masked by the map window, the stage-2 check does not read the latch, and the drone had reported not being engaged. Fixed by `AcousticExit.adrift`: a silent drone whose last fix is within the reach of a passive hull holds the gate.
* **RC10 (med7 `lost_drone_mutex` s981).** The heading loop was proportional only. During a wait of minutes the heading error of a queued drone grew to 6-16 degrees, beyond the 6 degree tolerance of the queue point, so the drone with priority no longer counted as queued and did not commit for 350 s. Fixed by an integral term on the heading (`LowLevelController`). The tolerance is unchanged. Heading beyond 6 degrees for more than 5 % of the time at the queue: 5 of 57 drone-runs in med6 and med7.
* **RC11 (med8 `lost_drone_mutex_carried` s986, with the reach rule active).** A committed drone held at the hold point was pushed through the CR by the avoidance layer, a hull closing from behind on the same axis. Past the CR the hold sent it back through it, and it met the hull there: P2 violated at 53.4 s, separation 1.55 m. Fixed: a committed drone past the middle of the CR goes on to the exit.
* **RC12 (med9 `lost_drone_mutex` s995).** Two corrections under the identity rule, at 77.3 s and 79.4 s, moved the estimate of a committed drone 2.2 m the wrong way. It was 1.6 m before the gate and 0.5 m off the axis, with a lateral error of -0.57 m and along-axis corrections already applied, and the identity rule became available when the dead drone left the premise (71.1 s). Offline replay of the logged echoes: a near-tie in the fit cost between truth, fit and the mirror of the truth about the axis (0.843, 0.820, 0.838); the first fit is 0.6 to 0.7 m from the mirror, the second at it (0.14 m). The bars sit at the edge of the side cones, and the RIGHT echo of the bar has no predicted bar at the fitted pose and costs nothing. A wrong correction near a bar also occurs with no noise at all (the pose on the far side of the bar; `tests/test_unconfirmed_corrections.py`). The error of 2.7 m was beyond the reach of any later fit (`MAX_CORR_M`), so for the rest of the mission the drone approached 3.5 m off the axis, held, timed out, went back to the queue and came back: 8 timeouts in 732 s, P1 and P2 respected, INCOMPLETE. Fixed by treating a correction with no validated fit as a hypothesis, undone when a hold ends with no valid localization, after the give-up edge (undoing while committed could feed the position-gated transitions of the hold: formal review N13). The corrections of this kind were not seen to lower the error (med5, med6, med7, med9: 309 corrections, 151 better, 64 worse by more than 0.05 m, mean change +0.02 m by our count; 285, 97 better, 80 worse, mean +0.004 m by the formal reviewer's; 278, 153 better, 79 worse, mean +0.004 m by the underwater reviewer's, who also separates axial (n 150, mean -0.02 m, worst +0.13 m) and identity (n 128, mean +0.03 m, worst +1.09 m), and by initial error: below 0.2 m an identity fit made it worse in 22 of 25, mean +0.43 m; at 0.7 m or more it improved it in 20 of 26, mean -0.46 m). Two limits after round 7: the undo covers only the corrections since the last commit (those before it include real drift corrected at the queue), and the step of a correction under the identity rule is capped at 0.5 m (`IDENTITY_STEP_M`), so one wrong step stays within reach of a later fit (a one-step truncation of the logged fits, not a replay with new fits: 44 worse and 75 better, mean -0.065 m, worst +0.5 m, against 61, 60, +0.03 m, +1.09 m uncapped). The undo acts on the sum sealed at the end of the hold: what is corrected between the timeout and the give-up edge stays. A hold without validation does not refute a correction: the undo is a recovery heuristic. A filter on the fit was tried offline and rejected: refuting a candidate by an unexplained echo blocks 48 % of the worsening corrections and 35 % of the improving ones (our offline estimate, not reproduced by the underwater reviewer). The direct cause lies in the cost function: a sector with an echo and no predicted bar costs nothing and LEFT pays the same cap for every candidate, so the two sectors that break the left-right symmetry contribute nothing (round 7, underwater). `lost_drone_mutex_rise_low` s960 (med6, 9 timeouts) is a different mechanism with the same precursor: no identity fit accepted (a good fit, refused: `identified` false), 1.09 m of corrections in 5 axial steps, lateral error growing from 0.04 m at 2 s to 0.51 m at 30 s. This rule does not address it. A correct fit refused at 0.5-1.1 m of lateral error stays open and is declared; the reason for the refusal is logged from now on (`nav_aid.others`).

## 2. Failure model and vacancy (as implemented)

* **`TOTAL_FAILURE` replaces the thruster-only failure** in every permanent-fault scenario. From `t_on` the drone runs no controller, emits
  no ping and no packet, has zero thrust and rises. The engine's BlueROV2 is exactly neutral (`BlueROV2.cpp`), so the ascent is applied on
  the simulator side as a vertical current on that drone only, added to the drift vector (`config.FailureModel`, `sim/holo_env.py`).
  Nothing onboard reads it.
* **Rise speed.** Nominal `v_rise_mps` = 0.10 m/s (credible range 0.05-0.13 m/s, an estimate for a ballasted vehicle, not a measurement of a
  real one). Bench `calibration/probe_buoyant_ascent.py`, `results/calibration/buoyant_ascent.json`: measured terminal speed 0.095 m/s
  at the nominal target (ratio 0.95), 0.144 at 0.15, 0.188 at 0.20; 0.040 at 0.05 (ratio 0.80, outside the 15 % check, which is left
  failing and reported). The ascent is independent of a horizontal current up to 0.30 m/s (0.094-0.099 m/s).
* **Failure inside the opening.** A drone failing under the top bar of G06 with |s| <= 0.2 m (also at yaw 45 and 90 deg, and at l = 0.7 m)
  stays pinned at dz = 0.87 m, inside the critical region, for as long as the bench ran. From |s| >= 0.3 m it clears the bar and leaves the
  CR through the top in 12.5-13.2 s. So a failed drone may leave the CR by itself or may never leave it, and nothing onboard can tell which:
  this is why silence never clears a gate.
* **Scope of the failure model, disclosed.** Only total loss is modelled. Partial failures (thrusters lost, electronics alive) are outside this
  phase's experiments, by the owner's decision. `lost_drone_rejoin` and `lost_drone_rejoin_front` (temporary thruster loss in v10) use a
  temporary localized jet instead: the before/after comparison of these two is not like for like.
* **Vacancy by silence.** A drone not heard for `t_silent_s` = 45 s is dead and its slot vacant, whatever the observer's mode and position, for
  expected neighbours only (within `neighbour_range_m`; far pairs are often silent). Basis, calibrated on the simulated channel with no analytic bound: on expected-neighbour links the longest silence in the
  72 v10 MEDIUM runs was 29.3 s by a reviewer's proxy (31 s over all pairs, on far pairs of `formation_six`, for which vacancy by silence is not used). A drone heard again is alive and its slot required again. A receiver hearing nobody at all is deaf
  (`AcousticTracker.deaf`): silence then says nothing and the B0 displacement timer applies. Displacement alone never makes a slot vacant
  while the beacon works.

## 3. Payload v2 (hardware assumption H6, still strictly one-way)

No request, reply, acknowledgement, leader or control command. 64 bits: frame (2), x, y (24), depth (8), bound (6), gate index (2), gate phase (2),
reserved (4), CRC-16 (16). Air time at 1 kbit/s: 64 ms. Slot budget for n = 6 at the 1 s period: 40 ms chirp + 64 ms packet + 33 ms propagation
spread = 137 ms of 167 ms. Data-packet loss is per link and three times higher on NLOS links (`sim/beacon.py`). The restriction of the earlier briefs
was a design choice, not a physical limit.

* **Bound encoding**: conservative, 0.1 m steps to 3.9 m, then 0.5 m steps to 15.4 m; 63 means invalid.
* **Gate phase**: `GP_NONE` / `GP_COMMITTED` / `GP_ADMITTED` / `GP_PASSED`, a statement of fact about the sender.
* **Verified passage** (`perception/passage.py`): after admission, the crossing signature (the sum of the LEFT and RIGHT ranges matches the opening)
  followed by the DVL displacement beyond the exit; otherwise a REAR echo of the frame at the consistent range; otherwise the DVL odometry alone,
  only after `T_ODO_MAX_S`. `GP_PASSED` is held until the next gate is engaged.

## 4. Gate logic (perception only; automaton unchanged)

**The status never enters the precedence.** After the first independent reviews the relation logic was made independent of the beacon:
a FRONT-only echo inside the band `[r_near - nav_err_m, r_far]` makes no precedence relation, unconditionally, and always feeds the latch: an echo is never without effect (formal review
round 2, F1: the first version left a 1 m band with neither); a merged echo keeps its WAIT. M3 and S2 now model this predicate.

* A drone is **excluded as an occupant** only by `status_excluded` (`perception/acoustic_gate.py`): a fresh status not engaged or for another gate, or
  `GP_PASSED` for this gate. Evidence order: a later accepted fix in the CR beats an older status. Freshness is per layout:
  `min(2.5 s, t_min - latency)`, `t_min` the shortest travel time from the queue line to the CR at the speed bound.
* **Positive occupancy**: `GP_COMMITTED` / `GP_ADMITTED` of another drone keeps the gate BUSY.
* **Sender rule** (`passage.report_phase`): a drone inside the keep-out box (the CR grown by the hull and the declared navigation error) reports a claim whatever its history, about the gate it is at: carried in
  uncommitted, or back after a passage, it does not report that it is not there.
* **A positive echo is discarded only on fresh evidence** about every other drone (`no_fleet_drone`); silence or an old report never discards one (round 2, F2).
* **Silence excludes nobody from a gate decision.** A `GP_NONE` drone is excluded only by a fresh report; after a failure the latch is released by the sonar fallback (`t_occ_max`). Silence never clears a committed or admitted drone.
  A failure under the bar therefore blocks the gate for the survivors for good, by design; `gate_stalled_occupant` accepts this outcome.
* **Keep-out filter** (`Flows.keepout_filter`): the mission velocity of a drone not committed has no along-axis component towards the CR. Mission modes only, the avoidance output is not edited. The status logic does not rest on it.

## 5. Navigation and gate localization

* The map fit acquires up to the queue line (`acq_min = |queue_s| + 0.5`); matched echoes must be incompatible with every other drone's fix region.
* A measured gate crossing corrects the **lateral** estimate only (the along-axis correction caused a contact in development, med2 gate_single s902),
  and is skipped above 1.2 m. The correction moves the stored fixes with the estimate (`AcousticTracker.shift`).
* `NavBound` is never restarted (reviewer finding): the bound stays monotone.
* **Along-axis correction from one FRONT echo** (RC7, `StructureAiding._axial`): with every other drone excluded as the source of the echo, unique and consistent over 4 captures, along the axis only; it never validates the localization.
* A dead drone stays left out of the identity premise of the map fit (`Perception._other_regions`), against the reviewer's suggestion: there the echo moves the own estimate, by at most `MAX_CORR_M` per correction (the sum is not bounded until a validated fit: RC12), and the corrected estimate sets the structure window, so a hull silent for 70 s can be taken for a bar. This narrow path is accepted and disclosed; removing it would disable the fit in every failure run.

## 6. Formation

* **Static reference.** With `v_clock` = 0 the DROP_BACK has no purpose. The drone goes straight to its slot; a reversal instead made a DROP_BACK/PASS
  limit cycle (most of the INCOMPLETE runs of med2).
* **Lateral residual away from the slot** (RC6, `Perception.lateral_far`). For a neighbour holding its slot by its own report, the identified fix gives
  `f.rel - d_k - (slot - p)`: the relative navigation error, independent of the own displacement. With `slot` including the correction, the residual goes to zero
  with it. Its median feeds the existing lateral integrator `lam` (bound `progress_max`, 2 m), and the rejoin lane follows `lam`. Near the slot the
  ordinary consensus acts as before. Only the mission reference moves: the separation layer and its formal checks are untouched.
* **The rejoin lane follows a running give-way** (RC8), laterally and in depth.

## 7. Verification

* Unit tests: `tests/test_status_payload.py` and the updated suites.
* Z3: `formal/check_status.py` S1-S10 with mutations; included in `formal/check_properties.py`.
* Development batches: `analysis/MEDIUM_CAMPAIGN_LOG.md`. The failures are stochastic, so the comparison with v10 is statistical and on held-out seeds.
* Order: MEDIUM-ALL fresh seeds, CLEAN and LOW regressions, freeze of the candidate and of the criteria, final validation.
