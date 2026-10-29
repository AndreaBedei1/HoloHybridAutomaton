# Acceptance criteria of the final campaign (frozen before the campaign)

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
