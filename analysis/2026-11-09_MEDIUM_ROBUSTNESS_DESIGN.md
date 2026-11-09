# MEDIUM-ALL robustness: root causes of the v10 failures and proposed design

Branch `feature/b2-medium-robustness`, from `main` (120b400). Baseline: tag `b2-freeze-v10` (9866242), kept immutable.
Status: PROPOSAL, before implementation and independent review.

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

## 2. Failure model and vacancy

* **`TOTAL_FAILURE` replaces `THRUSTER_FAILURE`** in every scenario of the new matrix. From `t_on` the drone: runs no
  controller, emits no ping and no packet, receives nothing, has zero thrust, and rises at the buoyant terminal speed.
  The engine's BlueROV2 is exactly neutral, so the ascent is applied as a vertical per-agent current command calibrated
  by `calibration/probe_buoyant_ascent.py` against the target `v_rise` (0.15 m/s, the estimate for the +0.2 kg net
  buoyancy of the ballasted vehicle). Sim-side only (`config.FailureModel`); nothing reads it onboard.
* `lost_drone_timeout`, `lost_drone_mutex` and `gate_stalled_occupant` become total failures. `lost_drone_rejoin` and
  `lost_drone_rejoin_front` (temporary thruster loss in v10) can no longer be a failure of the drone: a temporary
  thruster loss is not in the failure model. They become a temporary localized jet that displaces a healthy drone beyond
  its authority (same structure as `formation_gust`). The before/after comparison of these two scenarios is therefore
  not a like-for-like comparison and is reported as such.
* **Vacancy by silence (replaces the vacancy timer).** Each drone keeps the time of the last reception of every other
  drone (identified by its pinger code, the fix need not be accepted, a CRC-valid or absent payload both count). A drone
  not heard for `t_silent` = 30 s is dead: its slot is declared vacant, **whatever the observer's mode and position**.
  A slot is re-included when its drone is heard again (and near its slot, as today). `t_silent` must exceed the channel's
  longest silent stretch: in MEDIUM the longest absence between two drones in all 72 v10 runs is 31 s (`formation_six`,
  reception rate 45 %, line of 6, the extreme pairs at 17.5 m, shadowed by the hulls in between); median of the runs 9 s.
  A false death needs a silent stretch that is impossible to tell from a dead drone. `t_silent` is therefore larger than
  the channel's longest absence: **45 s** (the existing `t_rejoin` plus 15 s, and bounded by the deadline term). Alive
  drones are never declared vacant by displacement: there is no vacancy without silence.

## 3. Payload v2 (hardware assumption H6, still strictly one-way)

No request, reply, acknowledgement, leader or control command. Each drone keeps broadcasting a periodic packet of 64 bits
(8 bytes, was 56): frame (2), x, y (24), depth (8), bound (6), **gate index (2), gate phase (2)**, reserved (4),
CRC-16 (16). Air time at 1 kbit/s: 64 ms; slot budget for n = 6 at the 1 s period: 40 ms chirp + 64 ms packet + 33 ms
propagation spread = 137 ms of 167 ms. The restriction of the earlier briefs (identity only; later position and bound
only) was a design choice, not a physical limit of the channel.

* **Bound encoding** non-uniform, conservative (rounded up), 0-3.9 m in 0.10 m steps then 0.50 m steps to 15.4 m, 63 =
  invalid only for a truly invalid bound. So RC2's disappearance of the payload at 6.2 m is removed.
* **Gate phase** (per gate index, the sender's own automaton and flow state, monotone within an attempt):
  `NOT_ENGAGED` (never admitted to this gate) / `COMMITTED` (stage-1 commit, held before the CR, not admitted: outside the
  CR by construction, invariant I5) / `ADMITTED` (stage 2 granted, possibly inside the CR) / `PASSED` (verified, below).
  The field is a statement of fact about the sender, not a command.
* **Independently verified passage.** `PASSED` needs a validated gate localization at admission AND the along-axis
  DVL displacement since admission of at least the distance to `exit_s` AND a REAR sonar echo of the frame at the range
  consistent with that displacement (seen in 97 % of the captures beyond the gate in the MEDIUM logs of v10). It is
  therefore independent of the long-term navigation estimate and of its drift.

## 4. Gate logic (perception only; sonar precedence and the automaton are unchanged)

Status is used only to EXCLUDE a drone from being an occupant, and as positive occupancy. It never grants precedence.

* A drone j is **status-excluded** for gate g iff its latest CRC-valid status, not older than `t_status` = 5 s, says
  NOT_ENGAGED or PASSED for g (or it is engaged with a later/earlier gate index than g). `t_status` is below the
  shortest possible time from the queue line to the CR at the maximum speed (3.4 m at 0.65 m/s = 5.2 s).
* **Positive occupancy:** a status COMMITTED or ADMITTED of any other drone, latched until a newer status of that drone,
  keeps the latch BUSY. A drone silent for `t_silent` whose last status was ADMITTED stays an unknown occupant for
  ever: **silence never clears a possibly occupied gate** (the v8 `silent_gone_s` release is removed). A silent drone
  whose last status was COMMITTED was outside the CR by construction and is excluded after `t_silent`.
* **Phantom echoes.** A corridor echo is not a fleet drone when every other drone is excluded (status or fix regions as
  in v10): the latch stays free. This extends the existing phantom rule (B2-PE) with status.
* **Precedence relations and echoes that are no drone.** A sonar target creates a precedence relation only if some fleet
  drone could be that target: for every other drone j either status/fix exclude j from the target's sector cone and
  range (in the receiver's own body frame, from the range-bearing measurement, so no navigation bound is involved), or
  the relation stands. With no information about a drone the relation stands (conservative). RC1(a) is removed while the
  sonar pattern rule itself, ties and the static rank are unchanged.
* Stage-2 entry (`entry_ok`) and the A1/A2/A3/A4 acoustic rules use the same exclusion. `heard_inside` and
  `last_known_inside` skip a status-excluded drone.

## 5. Navigation and gate localization

* The map fit beside the opening (`identified`, C5 restriction) needs every other drone excluded as the source of each
  matched echo. v10 required small payload balls (never available in MEDIUM). Proposed: every matched echo must be
  incompatible with every other drone's region (own frame, from the fix: sector cone and range including the one-sided
  multipath excess), or the drone is status-excluded and located by a fresh fix. This restores the relocalization at
  the hold point when the drone drifted in front of a bar, the mechanism behind the s804 livelock.
* `NavBound` keeps being the sum of worst cases and never decreasing EXCEPT at a verified gate passage: the drone was
  physically inside the opening, so the bound restarts from the opening geometry
  (`hypot(2 (w/2 - hull), dz)`, plus the local odometry since), and the estimate is corrected to the fit. The bound
  stops growing without limit along the mission. The bound remains a stated assumption.

## 6. Formation

* `DROP_BACK`: when the reference is not moving (`v_clock` = 0, end of the path or a planned hold), the "let the fleet
  pass" rule has no purpose: the drone may reverse to its entry point (bounded speed, the separation layer unchanged).

## 7. Verification plan

* Unit tests for every item; Z3 (new suite or extension of `formal/check_acoustic.py`): status exclusion is sound under
  the freshness premise (a drone in the CR is not excluded; `t_status` < minimum travel time); the status latch never
  releases on silence; positive occupancy blocks commitment; the extended phantom/relation exclusion never removes a real
  drone (randomized, envelope); mutations of each.
* Experiments: MEDIUM-ALL first: repeat the ten scenarios, then fresh seeds with larger N, then CLEAN and LOW for
  regressions; freeze; final validation on held-out seeds. The acceptance criteria (v6) are frozen before the final
  campaign and include: every safety item unchanged; failure model change declared.
