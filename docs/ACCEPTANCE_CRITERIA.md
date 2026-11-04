# Acceptance criteria v4 of the B2-only validation campaign (pre-registered before any campaign run)

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

1. **Safety (all cells):** zero SAFETY_FAILURE and zero UNSAFE_ACTION.  Reported with the rule-of-three 95 % upper
   bound 3/N on the per-run rate.
2. **H6 premise (all in-envelope cells):** zero NAV_BOUND_PREMISE_VIOLATED.
3. **CLEAN liveness (cell CLEAN / CLEAN):** every run PASS except gate_stalled_occupant (below); any STALLED or
   INCOMPLETE in CLEAN is reported as a failure of the design.
4. **Degraded cells:** PASS + SAFE_DEGRADED rates with Wilson 95 % intervals; completion times as Kaplan-Meier
   curves censored at the deadline; no acceptance threshold is set on them (they characterise the envelope).
5. **gate_stalled_occupant:** the first drone loses its thrusters in the middle of the CR (ground-truth trigger).
   Accepted when P1 and P2 hold and E2.to_free = E3 = E4 = E5 = 0; the mission requirements are physically
   unreachable (the gate is blocked) and are not part of its acceptance; its class is reported as it is.
6. **Acoustic benefit:** only B2 against B2 under outage (cells MEDIUM-ALL/MEDIUM vs MEDIUM-ALL/BLACKOUT, and
   CLEAN/CLEAN vs CLEAN/BLACKOUT), exact McNemar on nominally paired runs (engine noise is unseeded), Holm-corrected.

## v4.4 Campaign (fixed)

`scripts/run_sensor_robustness_campaign.py --phase b2final`: 8 cells (protocol / sensor profile): B2_CLEAN/CLEAN,
B2_LOW/LOW-ALL, B2_MEDIUM/MEDIUM-ONE, B2_MEDIUM/MEDIUM-SUBSET, B2_MEDIUM/MEDIUM-ALL, B2_MEDIUM/CLEAN,
B2_BLACKOUT/CLEAN, B2_BLACKOUT/MEDIUM-ALL; 11 non-gate scenarios x seeds 100-104 and 6 gate scenarios
(gate_single, integrated_short, mutex_deadlock_resolution, line_parallel_mutex, lost_drone_mutex,
gate_stalled_occupant) x seeds 100-109: 115 runs per cell, 920 runs.  Seeds 100-109 were never used in
development.  At most two guarded HoloOcean instances, low priority.  If the time available cannot hold the 920
runs, the smaller variant of the plan (3 and 6 seeds, 552 runs) is used, chosen and recorded here BEFORE the first
run, never after seeing results.

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
