# Review closure: the combined P2 gap of st_lost_middle s5201, premise C, acoustic latency

Branch `review-p2-closure`, from `main` 114897e (`b2-v11-validation`). The tag `b2-freeze-v11` and its results are not
modified. Dates: 2026-11-17/18. Runs at four HoloOcean instances, below-normal priority, private engine roots.

## 1. P1 of the review: the combined gap

**Mechanism of s5201 (v11).** Drone 2 was held by the stage-2 check before the CR, committed; drone 0, also committed,
was behind it. The sonar lower bound placed drone 0 at 1.96 m (true 3.05 m), inside the warning band. The separation
filter found no velocity meeting v_open for the REAR patterns and fell back to the escape, which drove drone 2 forward at
0.49 m/s into the CR without admission. Each separate proof (S1-S2, M3, S2, F0-F2) held; their composition did not
exclude the entry, because the avoidance layer could move a held vehicle along the gate axis.

**Reproduction under v11** (`results/review/raw_repro_v11`, `audit_repro_v11.json`): INCOMPLETE, the concurrent
commitment again (CONCURRENT), no pushed entry, P2 held. The push is not reproduced at will: the engine noise is
unseeded.

**Change (controller).** The gate guard (`control/flows.py gate_guard`, `control/escape.py`; `GateRule.avoidance_guard`,
default True). Inside the keep-out box (s >= -2.189 m, lateral reach 2.39 m) a vehicle not admitted may use avoidance
velocities only with `v . a <= GUARD_GAIN (s_hold - s)`, s_hold = -1.179 m. Infeasible cases are explicit:
separation warning: `GUARD_HOLD` (station keeping where the hold began: `v = -min(0.25, 1.0 max(0, s - s_start)) a`) or
`GUARD_RETREAT`; collision avoidance: `OVERRIDE`, logged (`GATE_GUARD_OVERRIDE`).

**Formal** (`formal/check_gate_guard.py`, 21 checks, all as expected):

| Check | Kind | Result |
|---|---|---|
| GG1 / GG1m | exhaustive numeric on the deployed planner / mutation | no advance beyond the bound except an override; the v11 planner advances |
| GG2 / GG2b | exhaustive numeric (pair lemma) | pair opening 0.153 m/s at the hold points; 0.051 m/s with both 0.1 m beyond |
| GG5 / GG5o / GG5b | k-induction (k = 18) / BMC | P1 of a guarded pair at 0.743 m/s^2; SAT at 0.5 m/s^2 (as S1o); both beyond the bound: bounded only |
| GG3, GG3i / GG3a, GG3ai | inductive braking-curve invariant | estimated centre within s_hold + 0.30 m (margin 0.079 m to the CR face); at 0.5 m/s^2 within s_hold + 0.32 m (margin 0.059 m) |
| GG3k1, GG3k2 (+ initial sets) | inductive | a guard hold stays within 0.20 m of its start (start <= 0.05 m/s), 0.40 m (<= 0.20 m/s) |
| GG3m, GG3h | mutations (BMC 8 s) | without the guard, and with a zero-velocity hold, the bound is crossed |
| GG4 n = 2, 3, 4 / GG4o | composition proof / declared limit | never two in the CR with concurrent commitments, late claims and avoidance stops; an override can put a second vehicle in the CR |

The separate proofs are not used to claim the composition: GG4 checks the composition model directly, and GG4o and the
GG3/GG3k worst case (about 0.3 m of the estimated centre into the CR) are declared limits monitored at run time.

**Adversarial HoloOcean tests** (`st_rear_pusher` pusher at s = -3.6 m, `st_rear_pusher_close` at -2.6 m; a scripted
vehicle on the centre lane pushes the waiting drone toward the gate; seeds 7000-7002):

| Controller | Runs | P1 min (m) | P2 | CR entries not admitted | with another inside | Contacts |
|---|---|---|---|---|---|---|
| v11, no guard (B2NG_MEDIUM) | 6 | 2.98-3.38 | held | 162 | 0 | 1 (s7001 close, t = 694.9 s, not with a vehicle: SAFETY_FAILURE) |
| first guard (zero-velocity hold) | 6 | 2.91-3.37 | held | 22 | 0 | 0 |
| guard with station keeping (final) | 6 | 3.02-3.38 | held | 3 | 0 | 0 |

All 18 end INCOMPLETE by design (the pusher stays in front of the gate). The first guard drifted into the CR face with
zero-velocity holds; this led to station keeping and to the mutation GG3h.

## 2. P2 of the review: premise C by the order of events

The 32 s monitor of v11 is replaced by `holo_fleet/referee/premise_audit.py`: SATISFIED, CONCURRENT, VIOLATED,
INCONCLUSIVE, from the commit edges, the TDMA emission of the first ping carrying the claim, its earliest delivery and
the blocking evidence (latch BUSY or EXITING, or the stage-2 check refusing). The 32 s is the horizon of the bounded check
S2h, not a delivery deadline; the freshness cutoff `status_fresh_s` (1.946 s for st_gate_pair, 2.5 s otherwise) bounds the
use of a report.

A first version read the phase of a ping from the record of the ping time; a ping is sent after the control step before
it, so the audit was one step late for slot-aligned pings. Fixed with a regression test (commit 0a0b757); every audit
below is with the fix.

| Batch | Runs | SATISFIED | CONCURRENT | VIOLATED | INCONCLUSIVE | CR entries not admitted | P2 false |
|---|---|---|---|---|---|---|---|
| final campaign v11 | 448 | 1010 | 1 (s5201) | 0 | 0 | 2 PUSHED (s5201) | 0 |
| post-freeze pilot | 64 | 81 | 0 | 0 | 0 | 0 | 0 |
| post-freeze held-out | 100 | 171 | 0 | 0 | 0 | 0 | 0 |
| adversarial (v11, first guard) | 12 | 24 | 0 | 0 | 0 | 184 | 0 |
| adversarial (final guard) | 6 | 12 | 0 | 0 | 0 | 3 | 0 |
| held-out with the guard | 70 | 141 | 9 | 3 | 9 (B0) | 0 | 0 |
| latency | 30 | 60 | 0 | 0 | 0 | 0 | 0 |
| four vehicles | 10 | 60 | 0 | 0 | 0 | 0 | 0 |

The 3 VIOLATED pairs (st_lost_middle s7304 and s7310, twice) follow 1-2 claim pings that did not arrive under the MEDIUM
channel losses (reception rate about 0.6): premise C failed there, and the stage-2 check held the second vehicle until
the CR was seen free. The INCONCLUSIVE pairs are B0 runs (no claim to deliver).

## 3. P3 of the review: acoustic latency

Packets per configuration (communication sweep of the post-freeze validation and the paired experiment of this review):
latency 0.5, 1 and 2 s: 100 % stale (dropped before decoding, so the claims are lost and the gate runs on the sonar
only); jitter 1 s: 83 % stale; the other configurations about 99 % accepted. The 2 s latency of the sweep is far above
the 0.4 s acceptance limit (`AcousticConfig.latency_max_s`).

Paired experiment just below and above the limit, gate_single and lost_drone_mutex, LOW sensors, seeds 7300-7304
(`results/review/report/latency`):

| Configuration | Runs | Complete | Accepted | Stale | Mean gate wait gate_single (s) | Mean last gate gate_single (s) |
|---|---|---|---|---|---|---|
| B2_LOW (latency 0.1 s) | 10 | 10 | 98.9 % | 0 % | 30.8 | 75.1 |
| latency 0.35 s | 10 | 10 | 63.6 % | 35.5 % | 35.5 | 89.6 |
| latency 0.45 s | 10 | 10 | 0 % | 100 % | 44.3 | 101.2 |

At 0.35 s a third of the packets are already stale (latency plus propagation and processing beyond 0.4 s). Every run
completed with P1 and P2; the cost is waiting at the gate. The freshness limits were not raised.

## 4. Held-out validation of the changed controller

Rule committed before any held-out run (`results/review/HELDOUT_RULE.txt`): six gate scenarios, B2_MEDIUM MEDIUM-ALL
and B0 LOW-ALL, seeds 7300-7304, and st_lost_middle B2_MEDIUM seeds 7310-7319; 70 runs.

- P1 and P2 held in all 70; no CR entry not admitted; guard events in 7 runs (4 GATE_GUARD_HOLD of the separation warning, 16
  GATE_GUARD_OVERRIDE of the collision avoidance), none followed by a CR entry.
- B2: 39 of 40 PASS; st_lost_middle s7303 INCOMPLETE (a concurrent commitment with a wrong validated fit and a
  relocalization hold: both vehicles stayed out of the CR to the deadline). v11 final campaign on the same scenarios:
  74 of 75.
- B0: gate_single, line_parallel_mutex, mutex_deadlock_resolution 5 of 5, integrated_short 4 PASS and 1 P3_LATE,
  lost_drone_mutex and st_lost_middle 0 of 5 (P3_FAILURE: false vacancies without a beacon, as in v11).

**Paired comparison with v11** (st_lost_middle, B2NG_MEDIUM MEDIUM-ALL, the same 15 seeds; `results/review/raw_paired_ng`):

| st_lost_middle, 15 seeds | PASS | runs with a non-satisfied pair | CONCURRENT / VIOLATED pairs | CR entries not admitted | P1 min (m) |
|---|---|---|---|---|---|
| v11, no guard | 15 | 10 | 10 / 1 | 0 of 30 | 1.93 |
| guard | 14 | 10 | 9 / 3 | 0 of 28 | 2.07 |

The rate of concurrent commitments is a property of this scenario (one vehicle lost in the middle of a line, the other
two out of sonar range of each other), not of the guard: the final campaign had only 5 runs of it. The push of s5201 did
not recur in either set. The guard's cost on these seeds is one INCOMPLETE run (s7303, above); with 15 runs a difference
of one is not distinguishable from noise.

## 5. Four vehicles at one gate

`gate_four` (stacked square, the same gate G06), seeds 7400-7404: B2_MEDIUM 4 PASS and 1 P3_LATE (s7403), B0 LOW 5 PASS.
P1 (minimum 2.88 m) and P2 held; premise C 60 SATISFIED; no CR entry not admitted.

## 6. Regressions

Unit suite 596 passed, 1 skipped. Formal suite 321 of 321 as expected: the 300 checks of the post-freeze suite and the
21 of the gate guard. The counts of the post-freeze report are corrected: 80 held-out communication runs (not 85; the
other 20 held-out runs are the emergence study) and 300 formal checks before this review.
