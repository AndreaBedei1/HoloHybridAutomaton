# B2 consolidation: implementation status

Branch `b2-consolidation` (from `b2-acoustic-aided` at `a6efd0f`). Plan: `analysis/2026-11-03_B2_REVIEW_AND_PLAN.md`,
Section G. The user approved it on 2026-11-03 and asked for autonomous execution of G.1-G.7, with independent reviewers
in every phase, literature-grounded criteria and a final independent validation campaign.

## Done (not yet validated in simulation)

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

## Next

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
