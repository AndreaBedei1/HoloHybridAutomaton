# v7 (2026-11-10): v6 with the definition of a contact stated for a dead drone. Pre-registered before the final campaign of the candidate

**Change from v6.** A contact is counted for a drone under control only. The collision sensor of a drone after its `TOTAL_FAILURE` is recorded (`dead_drone_contacts` in the referee metrics) and neither ends the run nor
fails it: the hull has no actuation and the failure model makes it rise against whatever is above it (`results/calibration/buoyant_ascent.json`), so the event is not an action of the fleet. P1, P2, every pair distance and the
collision sensor of every live drone are unchanged, so a live drone that touches a dead one still fails the run. The change affects no earlier result of the candidate: of the 240 runs of batches med3 to med6 only 5 have a
contact, and each is the hull of a dead drone in a stress scenario. Without it those stress runs stop at the contact and cannot show what the other drones then do.

# v6 (2026-11-09): total failure model, payload v2. Pre-registered before the final campaign of the candidate

Baseline, immutable: tag `b2-freeze-v10`, MEDIUM-ALL 56/66 PASS outside `gate_stalled_occupant`, CLEAN and LOW-ALL 66/66.

**Changes from v5.** The classes and requirements are unchanged. One term moves: the fault term of the deadline uses the silence interval
(`max(t_rejoin, t_silent_s)`), because the drones now wait for it. Referee times are logged at 1 microsecond (a replayed hold timer on 1 ms times could
fall short of the online one); no tolerance enters a criterion.

**Scenario changes, disclosed.** Permanent faults are `TOTAL_FAILURE` (propulsion, electronics and beacon stop together, the drone rises). `lost_drone_rejoin` and
`lost_drone_rejoin_front` displace a healthy drone with a temporary jet instead of a temporary thruster loss. The comparison with v10 of these two scenarios, and of the
failure scenarios, is not like for like. Partial failures are outside this phase.

**Campaign.** The matrix of v10: cells B2_CLEAN / CLEAN, B2_LOW / LOW-ALL, B2_MEDIUM / MEDIUM-ALL; 12 non-gate scenarios x 3 seeds, 6 gate scenarios x 6 seeds; 72 runs
per cell, 216 runs. Held-out seeds 1000-1005, used by nothing before the freeze. Code and criteria frozen by a tag before the first run. The failures are not determined by the seed
(the engine's noise is unseeded), so the evaluation is statistical.

**Acceptance, per campaign, decided before the runs.**

1. Safety, every run of the 216: no SAFETY_FAILURE and no UNSAFE_ACTION.
2. No SUT_ERROR and no REPLAY_MISMATCH. An INVALID run is rerun only if an infrastructure cause is shown.
3. CLEAN and LOW-ALL: 66/66 PASS outside `gate_stalled_occupant` (no regression from v10).
4. MEDIUM-ALL: at least 63/66 PASS outside `gate_stalled_occupant`: the smallest count with a one-sided Fisher exact test against 56/66 below 0.05 (p = 0.038).
   The test and the Wilson interval are reported whatever the outcome.
5. `gate_stalled_occupant`: judged on safety as in v5; survivors never enter while a claim stands.

**Reported beside the acceptance, whatever the outcome.** The MEDIUM-ALL result on the scenarios unchanged from v10 (`lost_drone_rejoin`, `lost_drone_rejoin_front` and the failure scenarios removed: v10 is 46/51 there; only 51/51 would reach p < 0.05, so this subset is descriptive).

**Stress set, pre-registered (v7: `gate_pinned_occupant` now exercises only the claim path: the branch where a survivor never heard the occupant is expected to be empty, P about 6e-4 from six emissions).** `gate_pinned_occupant`, `lost_drone_mutex_carried`, `lost_drone_front_rise_low`, the four rise-speed variants, and `MEDIUM_HT` on `lost_drone_mutex`, `lost_drone_timeout`, `gate_single`, `formation_square`, `integrated_short`, `lost_drone_rejoin`: seeds 1100-1102, 39 runs, on the frozen code. Metric: no SAFETY_FAILURE and no UNSAFE_ACTION; the rest is reported, including, for `gate_pinned_occupant`, the runs where a survivor never heard the failed drone. Not part of the acceptance above.

If an item fails the candidate is not accepted and is not merged; a further change needs fresh seeds. The stress scenarios (`gate_pinned_occupant`, `lost_drone_mutex_carried`, the rise-speed variants, the channel
`MEDIUM_HT`) are sensitivities: they are reported and are not part of the acceptance.

# v10 (2026-11-06): numerical fix, same plan as v9, fresh seeds 800-805

The v9 campaign (tag `b2-freeze-v9`, seeds 700-705) was stopped at CLEAN 47 PASS + 1 SUT_ERROR (formation_six s701):
a division by zero in the sonar sector frame (Rodrigues rotation, boresight antiparallel to +x within ~1e-8 rad;
latent since the six-sonar model, first occurrence in any campaign; results/robustness/raw_b2final_v9/STOPPED.md).
Fixed without changing the formula elsewhere, with a regression test.  No behaviour change; cells, scenarios and
acceptance as in v9 (216 runs).

# v9 (2026-11-06): rejoin of a slot with another slot behind it on its lane; 72 runs per cell

The v8 campaign (tag `b2-freeze-v8`, seeds 500-505) was stopped at CLEAN 69/69 PASS and LOW-ALL 36 PASS +
1 INCOMPLETE (p1_vertical_escape s502; P1 held, min 2.47 m; results/robustness/raw_b2final_v8/STOPPED.md).  The tee
centre drone, pushed behind by the escape, rejoined "from behind along its own lane" while the rear drone's slot is on
that lane: 250 s of warning/recovery cycles behind it.  The same geometry exists for the square front slots; the
lost-drone scenarios had always failed the rear drone, so this path was never exercised.  Behaviour change, reviewed
independently (two rounds): such a slot is reached on a level pass_dz_m = 3.5 m below (above when the depth band has no
room), the drone descending first without approaching the rear slot closer than 3.5 m, rising only past it (latched,
with hysteresis; the vertical give-way of the traffic rule at the queue spacing).  Disclosed: it relies on the modelled
up/down obstacle sonars (real BlueROV2s rarely carry them) and on depth room (the pass level -8.5 m is the band edge
z_min + 0.5; at a shallow site the seabed would limit it).  New scenario lost_drone_rejoin_front (front-left drone
fails and rejoins past the rear-left one).  Campaign infrastructure: an orchestrator no longer re-runs a run another
one completed after it listed its runs (v8: 34 complete duplicates, all PASS, archived).  Cells and acceptance as in
v7/v8; 72 runs per cell (12 non-gate scenarios x seeds 700-702, 6 gate scenarios x seeds 700-705), 216 runs; seeds
600-602 were the pre-freeze probes.

# v8 (2026-11-06): a completely failed drone rises (deployment assumption)

By the owner's decision, reviewed independently: a drone that fails completely (no power, no beacon) is net
positively buoyant and rises out of the CR; a SILENT drone last known in the CR is considered gone after
max(t_occ_max + t_clear, 50 s) (2.5 m at a conservative 0.05 m/s; BlueROV2 datasheet +0.2 kg ballasted, no published
ascent rate; AUVs and gliders often surface by drop weights instead, so this is a stated deployment assumption, not a
property of every vehicle, and the simulator does not reproduce it).  A drone that still pings is never released this
way (gate_stalled_occupant stays a test of the veto).  Same three cells as v7, fresh seeds 500-505.

# v7 (2026-11-06): three conditions, by the user's decision after the v6 campaign

* **Cells:** B2_CLEAN / CLEAN (ideal), B2_LOW / LOW-ALL (nominal degradation), B2_MEDIUM / MEDIUM-ALL (the upper
  level of the declared envelope): the documented LOW and MEDIUM levels of sensors and acoustic channel, every drone degraded.  Nothing
  beyond MEDIUM.  69 runs per cell (11 non-gate scenarios x seeds 400-402, 6 gate scenarios x seeds 400-405), 207 runs.
* **Removed:** the total acoustic blackout.  B2 assumes a working one-way beacon; its total loss is outside the
  protocol's operating assumptions (as removing the sensor that the protocol is built on) and is reported as a
  stated limit, with the v4-v6 BLACKOUT runs as historical evidence (safety held in all of them).
* **Acceptance:** safety (P1, P2, no unsafe action) in every run of every cell; every run PASS in CLEAN and in the
  nominal cell (gate_stalled_occupant judged on safety); the MEDIUM cell is reported as the upper degradation with
  its failure mechanisms (Wilson intervals), and the work is declared complete if CLEAN and nominal are fully PASS
  and safety holds everywhere.  Formation tolerance 1.0 m unchanged.
* **Behaviour change in v7:** an occupant last known possibly inside also vetoes the sonar exit witness and the
  EXITING -> FREE release (v6: a clutter echo released the latch over a stalled, acoustically shadowed occupant).
* Independent literature review (2026-11-06): LOW is defensible as nominal for the ASSUMED hardware (DVL, IMU,
  compass, depth anchored on datasheets; dropouts, outages, sonar wrapper and most channel values are stated stress
  assumptions; a one-way OWTT receiver is research hardware); MEDIUM is the upper level of the declared envelope (it
  exceeds the proved sonar envelope in about 1.2 % of captures, and stays below the only one-way field figures).
* Disclosed: the reduction to three cells was decided after the v4-v6 results; MEDIUM-ONE, MEDIUM-SUBSET and the
  channel-only cells are dropped (their v4-v6 outcomes are reported); heterogeneous (mixed) fleets and channel-only
  degradation are therefore untested in v7; the silent-channel fall-back evidence predates v7.

# v6 amendment (2026-11-06): behaviour fixes after the v5 campaign, same criteria

The v5 campaign (tag `b2-freeze-v5`, seeds 200-205) was stopped by the user at 201 of 552 runs: CLEAN 69/69 and
LOW-ALL 69/69 PASS, MEDIUM-ONE with 7 INCOMPLETE (no safety event).  Their analysis showed two design defects and one
over-conservative bound; fixed, independently reviewed, and verified on those 7 runs (7/7 PASS):
* phantom corridor echoes (MEDIUM sonar noise) that no fleet drone can have produced no longer set the occupancy
  latch: every accepted fix of every other drone within the tie window must exclude it, the latest fresh (A4 at the
  sighting; formal B2-PE, randomized: an occupant truly in the CR is never excluded);
* a healthy drone wrongly declared vacant is re-included on identified fixes and its own payload near its slot;
* realistic navigation bound: model-aided dead reckoning during DVL outages (calibrated commanded-velocity model,
  low level in feedforward + frozen integral), error per outage = the measured maximum for its length
  (results/calibration/dr_model_calibration.json), outages within 10 s summed linearly, clusters root-sum-square,
  worst case beyond the table or when the model is not used.  Coverage is logged in every run.
Criteria v5 unchanged.  Campaign: same 8 cells and sizes, seeds 300-302 / 300-305, hardest cells first.

# Acceptance criteria v5 of the B2-only validation campaign

Status: written 2026-11-05 after the v4 campaign (tag `b2-freeze-v4`, stopped by the user at 404 of 552 runs), by the
user's explicit decision.  v5 changes the LIVENESS criterion and the judgement of an infeasible mission; the safety
items, the property definitions and the cell matrix are those of v4 below.  Both the v4 and the v5 classification of
the v4 runs are reported, so that the change is visible.

* **Liveness is eventual.** "Prima o poi ci riuscirà" (the user): no time bound on a gate passage or a wait enters
  the class; the gate bounded-response checks of v4 are still computed and reported (`gate_bounded_response`).  The
  run horizon is HORIZON_FACTOR = 3 x the a-priori deadline (the exploratory horizon of plan H.2); the a-priori
  deadline remains the reported "on time" reference.  A run that reaches the horizon without SUCCESS is INCOMPLETE:
  not observed to complete, which is not a proof of a deadlock.
* **Feasible requirements only.** A scenario whose mission is physically infeasible by design is judged on safety
  over the whole horizon (`Scenario.safety_only`): gate_stalled_occupant (a drone stalled in the CR blocks the gate;
  asking the others to pass it would be meaningless).  It is PASS when P1 and P2 hold, there is no contact and no
  unsafe action (E2.to_free = E3 = E4 = 0) until the horizon.  The BLACKOUT exception of v4.3 item 1 stays.
* **Classes v5:** INVALID / SUT_ERROR > REPLAY_MISMATCH > SAFETY_FAILURE > UNSAFE_ACTION > SPEC_VIOLATION > PASS >
  CENSORED > SAFE_DEGRADED > INCOMPLETE (no STALLED).
* **Acceptance items (v4.3 as adapted by v5, fixed before the v5 campaign):** item 1 safety on ground truth for
  every complete run (BLACKOUT x gate_stalled_occupant reported separately); item 2 zero NAV_BOUND_PREMISE_VIOLATED;
  item 3 every B2_CLEAN / CLEAN run PASS (gate_stalled_occupant included, judged on safety): an INCOMPLETE there at
  the 3x horizon is a design failure; item 4 per-cell PASS + SAFE_DEGRADED rates (Wilson 95 %), on-time counts, PASS
  runs late at the gate (gate_bounded_response false) and PASS runs with a transient false vacancy, Kaplan-Meier
  censored at the horizon, gate_stalled_occupant excluded; item 5 replaced by the safety-only judgement; item 6
  unchanged.  Aggregator: `scripts/analyze_b2final.py` (frozen with the code).
* **Fresh seeds.** The v5 campaign uses seeds 200-202 (non-gate) and 200-205 (gate): 100-105 were used by the v4
  campaign, whose diagnostics motivated the v5 behaviour changes.  Same 8 cells, 552 runs.
* **Formal.** F6g (formal/check_formation.py): with the gate-delay hold a never-returning neighbour is vacant within
  t_rejoin + gate_delay_max + dt of running time (ranking max(H - h, 0) + max(t_rejoin - tau, 0); mutation F6gm).
* **Behaviour change made with v5 (not a criterion):** a missing neighbour reported in a gate's approach zone is
  delayed by the mutual exclusion, not lost; its vacancy timer is held for at most the protocol's slowest serial
  passage (v4 campaign: false vacancies of neighbours waiting their turn at a slow MEDIUM gate); the vacancy timer also
  runs after the plan end; a missing neighbour comes back on A_CONFIRM payloads near its slot together with an
  identified fix within e_lost; the DVL outage term counts the excess over the nominal hold.

---

# Acceptance criteria v4 of the B2-only validation campaign (pre-registered before the v4 campaign)

Status: written 2026-11-04 before the B2-only campaign; frozen together with the code commit recorded in every run's
provenance.  The sections after this one are the HISTORICAL v2 criteria of the 624-run B0/B2 campaign at `ad4fb5e`,
kept as written.  That campaign deviated from its own freeze: the B2 change `ad4fb5e` (acoustic exit by range) came
after the freeze `1dc9d3f`, and the earlier B2 runs were discarded without a record here (stage E).  Its results are
historical evidence for `ad4fb5e` only; v2 is superseded by v4 below.

## v4.1 Definitions (not changed by any result)

* **P1** pairwise true distance >= d_safe = 1.0 m at every sample (and no physical contact).
* **P2** at most one drone's centre in the critical region (CR) at any sample; precedence by the local rule.
* **P3** formation error <= E_TOL = 1.0 m (inclusive) held T_HOLD = 8.9 s, every loss episode recovered; **P3-deg**
  after a permanent fault: the healthy drones on their original slots, every healthy drone expecting the lost one has
  declared its slot vacant, no false vacancy.
* Requirements, deadline and gate events: `holo_fleet/referee/criteria.py` (v4 header) and `gate_events.py`.
  Provenance labels [REQ] [PROT] [LIT] [PROF] [CAL] on every constant.

## v4.2 Classes (holo_fleet/referee/robustness_metrics.py), in priority order

INVALID / SUT_ERROR > REPLAY_MISMATCH > SAFETY_FAILURE (P1 or P2 violated) > UNSAFE_ACTION (E2.to_free, E3 or E4
> 0) > SPEC_VIOLATION (automaton determinism or observation consistency) > STALLED (gate bounded response violated:
idle > t_occ_max + t_clear, or crossing obligation from the FIRST commit > L / v_pass + t_occ_max + t_clear) >
PASS > CENSORED > SAFE_DEGRADED (causal, non-CLEAN, with the hold) > INCOMPLETE.  A run tagged
NAV_BOUND_PREMISE_VIOLATED (the sent navigation bound did not contain the true error) is reported and counted as a
premise failure of H6; in an in-envelope profile it is a design failure, never silently excluded.

## v4.3 Acceptance (per cell, decided before the runs)

1. **Safety:** every COMPLETE run is judged on ground truth whatever its class: P1 and P2 hold and
   E2.to_free = E3 = E4 = 0; zero REPLAY_MISMATCH is itself required (a mismatch is investigated, never ignored).
   Reported with the rule-of-three 95 % upper bound 3/N on the per-run rate.  Exception, declared here: in the two
   BLACKOUT cells, gate_stalled_occupant is a silent stalled occupant in the masked far half of the CR, the case the
   design states as outside its guarantee (formal M3f counterexample, M4h); a P2 failure there is the predicted
   outcome, is reported separately with its count and does not enter item 1.
2. **H6 premise (all in-envelope cells):** zero NAV_BOUND_PREMISE_VIOLATED.
3. **CLEAN liveness (cell B2_CLEAN / CLEAN):** every run PASS except gate_stalled_occupant; any STALLED or
   INCOMPLETE there is reported as a failure of the design.
4. **Degraded cells:** PASS + SAFE_DEGRADED rates with Wilson 95 % intervals and completion times as Kaplan-Meier
   curves censored at the deadline, over the 16 scenarios other than gate_stalled_occupant; no acceptance threshold
   is set on them (they characterise the envelope).
5. **gate_stalled_occupant** (the first drone, drone_0, loses its thrusters once its centre is in the second half of
   the CR; no current): accepted when P1 and P2 hold and E2.to_free = E3 = E4 = E5 = 0 (BLACKOUT cells: item 1).  A
   run whose trigger never fired is INVALID (precondition unmet) and is counted as such.  A contact of the unpowered
   drone with the structure is a SAFETY_FAILURE like any contact (no special case).
6. **Acoustic benefit:** only B2 against B2 under outage, on the endpoint PASS (vs any other class), exact McNemar on
   nominally paired runs (same scenario and seed; engine noise is unseeded), two hypotheses: B2_MEDIUM / MEDIUM-ALL
   vs B2_BLACKOUT / MEDIUM-ALL and B2_CLEAN / CLEAN vs B2_BLACKOUT / CLEAN, Holm-corrected, gate_stalled_occupant
   excluded.

## v4.4 Campaign (fixed)

`scripts/run_sensor_robustness_campaign.py --phase b2final` from a clean commit (the script refuses a dirty tree and
refuses to resume over runs of another commit): 8 cells (protocol / sensor profile): B2_CLEAN/CLEAN, B2_LOW/LOW-ALL,
B2_MEDIUM/MEDIUM-ONE, B2_MEDIUM/MEDIUM-SUBSET, B2_MEDIUM/MEDIUM-ALL, B2_MEDIUM/CLEAN, B2_BLACKOUT/CLEAN,
B2_BLACKOUT/MEDIUM-ALL; 11 non-gate scenarios x seeds 100-102 and 6 gate scenarios (gate_single,
integrated_short, mutex_deadlock_resolution, line_parallel_mutex, lost_drone_mutex, gate_stalled_occupant) x seeds
100-105: 69 runs per cell, 552 runs.  This is the plan's smaller variant, chosen on 2026-11-05 before any campaign
run for the wall time (about 3.8 min per run with two instances: 920 runs about 29 h, 552 runs about 18 h); it
weakens the per-scenario conclusions (Wilson half-width about +-7 % at p = 0.9 per cell).  Seeds 100-105 were never
used in development.  At most two guarded HoloOcean instances (the user's limit), low priority.  INVALID runs
(infrastructure) are retried once by the orchestrator; every attempt is preserved and reported.

## v4.5 Amendments after the first v4 text (65c86c1), before any campaign run

| commit | change | reason and evidence |
|---|---|---|
| cc3c652 | gate_stalled_occupant without current; replay fed the logged collision edges | development runs b2_98f7908: the unpowered drone drifted into a bar within ~20 s and left the CR (the scenario no longer tested a stalled occupant); the replay ignored contacts (REPLAY_MISMATCH) |
| caead76, then reverted | DVL hold threshold 0.25 s | a wrong diagnosis (a pre-freeze review showed ordinary samples are at most 0.133 s old); restored to 0.15 s with the hold + latency constant corrected |
| this amendment | acceptance item 1 on ground truth, BLACKOUT exception for the stalled occupant, precondition INVALID, McNemar endpoint and family, smaller variant, resume guard, last-known-inside cleared by exit_confirm fixes | pre-freeze independent review |

The freeze is the git tag `b2-freeze-v4`; every campaign run records its commit.

---

# Historical v2: acceptance criteria of the 624-run B0/B2 campaign (superseded by v4 above)

These criteria, the experiment plan and the classification below are fixed before the final campaign is run and
are not changed in response to its results.  A change after the campaign has started would invalidate it: it would be
recorded here with its date and reason, and the affected runs regenerated.

## 1. Run classification (referee/robustness_metrics.py, unchanged)

| Class | Meaning |
| --- | --- |
| PASS | safe, every applicable requirement reached before the deadline |
| SAFE_DEGRADED | safe; a requirement not reached and a conservative event occurred (false vacancy, sensor-fault FAILSAFE) |
| LIVENESS_FAILURE | safe; a requirement not reached without a conservative event |
| SAFETY_FAILURE | a physical contact, a P1 violation among the drones (or below d_safe from an intruder) or a P2 violation |
| INVALID | infrastructure failure (engine start, killed worker, missing data); re-run, never part of the statistics |

Requirements per run: progress of every healthy drone to the reference end, gate completed (gate scenarios), P3
formation recovered / degraded formation held where the CLEAN reference shows it, no permanent false vacancy,
terminal success before the deadline (referee/terminal.py: deadline from the B0 CLEAN run of the scenario), automaton
determinism and observation consistency.  Each run also carries diagnostic `cause_tags` (false re-occupation, false
vacancy, relocalization hold, occupancy-timeout release, waiting for a missing neighbour, sensor-fault FAILSAFE, ...):
they never change the class.

## 2. Acceptance criteria

Safety, liveness and mission completion are evaluated separately.

**Safety - required in every run of every profile and protocol:**

1. no physical collision (referee contacts, collision sensors);
2. no P1 violation among the drones; no approach of an intruder below d_safe;
3. no P2 mutual-exclusion violation (two drones in the critical region) and no release of the gate while its
   occupant is inside it;
4. no automaton determinism violation and no observation-consistency violation;
5. no unexplained failure of a required formal obligation (formal/results/SUMMARY.md OVERALL PASS; scope, mutation
   and availability rows behave as documented).

A single safety failure stops the affected campaign: it is diagnosed and its cause resolved before continuing (and
the runs it affects regenerated).

**Liveness and completion:**

6. CLEAN: every applicable reference scenario PASS, for B0 and for B2 with every channel level run under CLEAN
   sensors, with no unexplained failure;
7. LOW / MEDIUM: liveness and completion are measured and reported per scenario, profile and protocol (class rates
   and cause tags), not required.  A non-PASS run is EXPLAINED when its cause tags match a documented limitation of
   the operating envelope (section 3); any other non-PASS run is investigated and reported as unexplained;
8. permanent loss: every healthy drone declares the lost drone's slot vacant and keeps a degraded formation, unless a
   documented limitation applies; completion is not claimed when a permanent fault or insufficient observability makes
   it physically impossible;
9. B2 never weakens safety; B2 with a silent channel (BLACKOUT) behaves as B0 (no B2-specific failure).

## 3. Validated operating envelope (what the guarantees assume)

* **P1 (separation, safety).**  Proof assumptions (formal S0-S3, perception/distance_bound.py): sonar echo errors in
  [-eps_range_near, +eps_range_far] = [-0.08, +0.12] m (measured in HoloOcean), data age <= tau_max = 0.25 s (EchoHold
  turns one missed capture into a stale reading), relative hull tilt <= 10 deg (measured 0.03 deg), the speed and
  current bounds of config.Envelope, the 120 deg sonar cone (a design assumption).  CLEAN and LOW are inside it; the
  MEDIUM profile exceeds the echo bound in about 1.2 % of the captures (+-2-bin range jitter) and tau_max during its
  outages: under MEDIUM, P1 is tested, not proved.
* **P2 (mutual exclusion, safety).**  Gate passages inside the validated queue domain (GateRule.queue_lateral_max =
  3.5 m: abreast queues of up to 3 drones and the verified stacks; mission.check_gate_domain refuses the others,
  formal M4g); occupancy latch with t_clear persistence; the t_occ_max fallback assumes a healthy crossing (formal M3,
  crossing speed >= V_LO): a drone stopped inside the critical region by a fault is outside the B0 guarantee (formal
  M3f); in B2 the fallback is blocked while that drone is heard inside the region.  In HoloOcean the sonar exit
  witness from a lateral queue point is rarely obtained, so most releases use the fallback.
* **P3 (formation, liveness).**  Recovery under the declared current and sensing envelope (formal F-checks).
  Documented limitations, each with its cause tag: range-only association cannot separate one displaced neighbour
  in the overlap of two adjacent cones from two neighbours (B0; B2 resolves it with identities); relative along-track
  drift is unobservable from abreast neighbours in B0 (false vacancy near the range gate); a drone whose drift puts it
  in front of a gate bar on the merge diagonal cannot be relocalized there without an assumption on the identity of an
  echo, and a navigation drift at the gate larger than the map-fit bound MAX_CORR_M = 1.2 m (reached by MEDIUM-ALL in
  long runs) cannot be corrected (relocalization hold, safe, in both cases).
* **B2 channel.**  Strictly one-way identity pings (TDMA), no payload; the onboard envelope covers the white errors
  at 3 sigma, the array calibration bias and the one-sided clear-path multipath excess (docs/ACOUSTIC_CHANNEL_MODEL.md);
  under it the acoustic exit evidence is not available in the validated gate geometry (formal B2-M4g, reported).

## 4. Experiment plan (fixed)

* Scenarios: the 16 of sim/scenarios.py (P1, formation, gate / mutex, integrated, lost-drone cases).
* Cells (scripts/run_sensor_robustness_campaign.py, phase `final`): B0 under CLEAN, LOW-ALL, MEDIUM-ONE,
  MEDIUM-SUBSET, MEDIUM-ALL; B2 with a CLEAN channel under CLEAN sensors, a LOW channel under LOW-ALL, a MEDIUM channel
  under CLEAN, MEDIUM-ONE, MEDIUM-SUBSET and MEDIUM-ALL, a BLACKOUT channel under CLEAN and MEDIUM-ALL: 13 cells x 16
  scenarios x 3 seeds = 624 runs, B0 CLEAN first (it fixes every deadline).
* One output location: results/robustness/raw_final; resumable; INVALID runs re-run, never aggregated.
* Execution: isolated parallel HoloOcean instances (separate engine processes, ports and shared-memory names), low
  process priority, bounded CPUs; no other process touched.
* Incremental validation: a representative subset (seed 0 of the CLEAN and MEDIUM-ALL cells of B0 and B2) runs first
  into the same location; the full plan resumes from it only if no safety failure and no unexplained CLEAN failure
  appeared.  Additional repetitions only to resolve a specific statistical or reproducibility question, stated here
  before they are run.
* Reproducibility: every run records the code commit (and refuses to start on a modified tree in the final phase),
  the seeds, the noise and channel profiles and the engine build; HoloOcean's own sensor noise is not seedable, so
  runs are statistically, not bit-for-bit, reproducible.
