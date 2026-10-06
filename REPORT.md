# REPORT - leaderless, communication-free UUV fleet from verifiable local hybrid automata

<!-- EXEC_SUMMARY -->

## 1. Contribution in one paragraph

We move from one hybrid automaton to the composition `H_fleet = H_1 || H_2 || H_3` of **identical** local
automata, with **no supervisor and no messages**: the only coupling is the water, perceived through each
drone's own sonar. Three fleet properties are stated, encoded in Z3 on abstractions **whose guards are
literally the runtime code** (a logic-backend trick: the same Python guard functions are executed with
floats in the controller and with Z3 terms in `formal/`), proved under explicit assumptions, and then
validated in HoloOcean by a ground-truth referee that the controllers cannot see. Every assumption of
the proofs is measured back on the simulation logs (`results/ASSUMPTIONS.md`).

## 2. Formal verification (Z3 4 / `formal/check_properties.py`)

All suites return their expected verdict (UNSAT = the property holds on the abstraction; SAT is expected
only for **mutation tests**, deliberately broken designs that must produce a counterexample, which shows
the checks are not vacuous). Full table: [formal/results/SUMMARY.md](formal/results/SUMMARY.md).

| suite | what is proved | checks | result |
|---|---|---|---|
| Local determinism & priority | for every mode and every legal observation exactly one edge is enabled (determinism + completeness); fault -> FAILSAFE; d < d_ca -> COLLISION_AVOIDANCE from **every** mode (also GATE_PASS); warning band -> SEPARATION_WARNING; GATE_PASS is entered uncommitted only with at_queue & !occ_busy & has_prio; no PASS->YIELD fallback; no commit under a separation hazard | 64 + 2 mutations | 66/66 |
| **P1** separation | pairwise radial model with perception error+staleness eps' = eps + c_max tau, speed caps, calibrated braking, unrejected differential drift: **BMC (12 s) and k-induction (k = 11) prove G(d >= d_safe) for all time**; the smallest warning distance that still verifies is 2.165 m (configured 2.3 m); local lemmas: the SW safety filter can open at v_open on two threats <= 90 deg apart, the 3D escape opens every threat at >= v_open/v_escape, in any triangle with sides >= d_safe at most one vertex is >= 90 deg (so every pair has a member able to open); a <= 0.8 s sensor blackout starting at d >= 2.08 m is safe | 9 (incl. 2 mutations) | 9/9 |
| **P2** mutual exclusion | the pairwise decision classes are exhaustive/exclusive; **no simultaneous commit** with independent errors <= eps; **no mutual waiting** (deadlock freedom) and robust waits are justified; priority **persists** while the committed drone leaves the queue line (hold/monotonicity assumptions); late arrivals cannot commit first (timing); a committed drone past the queue line is perceived in the occupied zone; queued and exited drones are outside the CR; hence "at most one committed drone" is an **inductive invariant** and G(occupancy <= 1) | 14 + 3 mutations | 17/17 |
| **P3** formation recovery | ranking-function proof on the deployed consensus/lane-keeping law with saturations, adversarial perception noise and tracking residual: the formation error decreases by a certified delta in every band, the recovered sets are invariant; **worst-case recovery from an 8 m along-track spread and 2 m lateral error: T = 59 s <= 60 s** | 9 + 1 mutation | 10/10 |

Numbers used by the proofs (all in `holo_fleet/config.py`, plant values from `scripts/calibrate_plant.py`):
d_safe 1.0 m, d_ca 1.6 m, d_warning 2.3 m, eps_rel 0.30 m, tau_max 0.2 s, v_max 0.40 m/s, v_escape 0.50 m/s,
a_brake 0.5 m/s^2 per drone (measured >= 0.5-0.76), w_rel 0.15 m/s, v_open 0.20 m/s; gate margins
mu_s 1.5/0.7 m, mu_l 2.3/0.35 m, mu_z 0.6 m; formation e_ok 0.55 m, e_lost 1.1 m, T 60 s.

**Honest scope of the proofs.** They are about discrete-time abstractions of the closed loop. The
abstractions share the guard code with the runtime but abstract the plant (velocity-level kinematics
with calibrated bounds) and the perception (bounded error, bounded staleness, symmetric detection).
Whether HoloOcean stays inside those assumptions is an *empirical* question, answered per run in
section 4. No claim is made about runs that leave the envelope (they are flagged).

<!-- EXPERIMENTS -->
