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
  expected neighbours only (within `neighbour_range_m`; far pairs are often silent). Basis: the longest silence between two drones in the
  72 v10 MEDIUM runs was 31 s. A drone heard again is alive and its slot required again. A receiver hearing nobody at all is deaf
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
* A dead drone stays left out of the identity premise of the map fit (`Perception._other_regions`), against the reviewer's suggestion: there the echo moves the own estimate, by at most `MAX_CORR_M`, and the corrected estimate sets the structure window, so a hull silent for 70 s can be taken for a bar. This narrow path is accepted and disclosed; removing it would disable the fit in every failure run.

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
