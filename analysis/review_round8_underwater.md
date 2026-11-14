# Round-8 review, underwater: `3f35900` against `8fdcbb4`

Read the diff, `scope.py`, rule D, the stress batch, GUARANTEES, the log, and the runs of `med11` (40), `st1` (34) and `reg2` (9 complete so far), with `med9`/`med10` as baselines. Read-only, no simulator. My scripts live outside the repository and ran at below-normal priority; a few one-line JSON reads and one sub-second binomial calculation ran at normal priority. I did not open the formal review. Tags: **[M]** measured, **[D]** derived by me.

## BLOCKER

**B1. Under any current the scope removes the gate phase, and a failure there now ends in a P2 violation, not a stop.**
- Geometry [D]: the obstruction box is ±1.19 / ±1.64 / ±1.64 m. The reach is `1 + 32.8 w_h` m (`scope.py:53`). The hold point (s ≈ -2.5) is inside E above 0.01 m/s of current, the n = 2 queue point above 0.06 m/s, and every abreast-3 queue point above 0.11 m/s.
- Exposure [M]: 42-71 % of live drone-time is inside E in the 22 `st1` gate runs at 0.2-0.3 m/s, and 6-13 % in still water.
- Coverage [M]: all five failure scenarios fail at t_on = 1 s, at s = -14.8 m (`scenarios.py:460`, `:519`). No failure has run at the queue, at the hold or past the exit, although in still water these are in scope (the hold point is 1.31 m from O).
- Consequence: with `adrift` removed, rule D releases the claim after 70 s (`acoustic_gate.py:155`, `:177`). A hull pinned under the top bar, in the masked CR, is then admissible. That is RC9 (`med7` s981: P2 violated at 1.18 m), now unhandled.
- Fix, either:
  - (a) A dead drone keeps the gate while its last fix, grown by its bound, lies in E (declared current). The result is a safe stop; s960, s1068 and s1071 are unaffected.
  - (b) State in the paper that a failure in E can violate P2, with the exposure figures and s981.
  - In both cases the final campaign needs in-scope failures near the gate.

## MAJOR

**M1. s1103 contains a wrong *validated* localization. The log says "never validated" (`MEDIUM_CAMPAIGN_LOG.md:280`).** [M]
- At 440.0 s drone_1 is committed and truly at l = -2.28 m (beside the opening, pushed there by avoidance), with the estimate at -1.23.
- A `safe` fit (clearance +0.398, `structure_aiding.py:377`) applies +1.1 m. `loc_ok` is true from 440.0 to 441.4 s and the latch is FREE. Only the stage-2 HOLD and avoidance kept the drone out.
- The error goes from 1.02 to 2.17 m. Two along-axis corrections (-0.44, -0.58 m) follow, and the error then stays at 2.1-2.25 m for the remaining 290 s (largest 2.49 m).
- This breaks `nav_err_m` = 1.0 m (`config.py:128`, the premise of G2 and S9) while committed. The validated fit also cleared the unconfirmed sum (`:411-414`), so the undo cannot act.
- Over 122 applied validated corrections in `med9`-`med11` and `st1` (mean -0.30 m) [D], this is the only one worse by more than 0.3 m.
- Fix: referee tags for gate-frame error above `nav_err_m` while committed or admitted, and for a validated fit that raises the true error by more than 0.5 m. Re-describe the open class. Consider refusing to validate a large step when the pre-fit estimate is beside the opening.

**M2. The open class is compass bias integrated over an unaided approach, and rule D prolongs it.** [M, D]
- At s = -3 m, over 225 drone-passages: mean |e_l| is 0.37 m with a failure (n = 107) against 0.18 m without (n = 118). The share above 0.5 m is 8 % against 1 %.
- The sign of e_l is fixed by the ±2 deg bias in about 90 % of passages; 2 deg over 15 m gives 0.52 m.
- The identity rule waits for rule D (70 s), but drones reach s = -3 at 28-45 s.
- In the logs this is a liveness problem, but the margin to the 1.0 m premise is 0.29 m (worst PASS: 0.71 m, `st_lost_middle` s2000). HIGH (4 deg) gives about 1 m.
- A metal frame and thruster currents are not modelled (`PARAMETER_PROVENANCE.md:228`).
- Fix:
  - a bias sweep of 2, 3 and 4 deg on the `lost_drone_mutex` family, at least 10 seeds per level;
  - |e_l| at s = -3 reported per passage, with the INCOMPLETE rate conditional on it;
  - optionally, release the identity premise for a silent drone whose last fix plus drift reach cannot reach any gate cone (the geometry of E, with no 70 s wait).

**M3. Premise V is a premise of P2, tested on a channel that cannot break it.**
- The P2 table (`GUARANTEES.md:43`) lists no premise. A live drone declared dead loses its claim and its status as a possible occupant, and the far CR is masked.
- Measured [M]: the longest live silence at a hearing receiver is 25.2 s (`st_formation_six_oblique`), at most 14.2 s in gate runs and at most 7.1 s in `reg2`.
- `med11` has no silence log, so "checked on every batch" (`GUARANTEES.md:77`) is not yet true.
- MEDIUM loss is independent per ping, with common 4 s outages (`acoustic_channel.py:105-107`). Long gaps come only from blockage, whereas a static queue beside a reflecting frame produces persistent multipath fades in real water, during holds that last minutes (400 s in s1103).
- With the project's `MEDIUM_HT` (Pareto, minimum 6 s, shape 1.2) [D]: P(OFF > 70 s) = 5.2 % per OFF period, and some link of 3 drones exceeds 70 s with probability 0.46 in 300 s.
- A live vehicle with a failed transducer is indistinguishable from a dead one, and is outside the failure model.
- Fix: list V under P2; add a `LIVE_DECLARED_DEAD` tag; run the gate family under `MEDIUM_HT` (at least 10 runs); state the safety-for-liveness trade of rule D.

**M4. Failure-model numbers.** [D]
- (a) `v_rise_min_mps` = 0.05 exceeds the docstring's own model value: 0.047 m/s from 33 w + 190 w² = 1.96 N (`config.py:387-396`). The +0.2 kg is unmeasured.
- (b) Flooding, a common way all electronics stop together, sinks the hull: everything is then in E (`scope.py:51`). Declare "power loss, hull intact".
- (c) `coast_m` = 1 m. The engine coasted 0.45 m. Published BlueROV2 surge damping gives about 0.26 m (Heavy) up to about 1.1-1.3 m (standard frame) at the through-water speed, which reaches 0.7 m/s under a head current.
- Fix: measurable deployment requirements (a bench rise test or a drop weight); compute the coast at the largest through-water speed. +0.5 kg net gives about 0.096 m/s and halves the drift term of E.

**M5. n = 2 with a failure.** A receiver hearing nobody declares no one dead (`acoustic.py:314`), so a dead, committed partner holds the survivor for ever. Two vehicles is the commonest field configuration and F2 claims n = 2 recovery. Test it or declare it.

**M6. Evidence.**
- 74 runs (`med11` and `st1`) with no safety failure give a 95 % upper bound of 4 % per run; 1 of 20 INCOMPLETE in the family gives 22 %.
- Rule D and the scope were chosen after s960, s1068 and s1071, so `med11` is development evidence.
- Engine noise is unseeded, so a rerun on the same seed is not a paired test.
- Dead-declaration latency was 69.1-70.1 s in all 30 gate failure runs [M].

## MINOR

- m1. Stress batch:
  - every current is spatially uniform. There is no jet or shear through the opening (w_rel_max is a P1 premise) and no vertical current at the gate (w_down enlarges E);
  - v_pass is fixed at 0.5 m/s, so 0.21 m/s head is the limit, and a reduced-v_pass probe would show whether the envelope widens;
  - 2 seeds per scenario;
  - sonar crosstalk (`PARAMETER_PROVENANCE.md:93`), the hard-edged beam and per-ping-independent acoustic loss belong in the threats to validity.
- m2. Stale text: `GUARANTEES.md:40` still cites the drift reach of a silent drone; `config.py:356` says "never excluded from a gate decision"; the s1103 note says "0.5-0.9 m", but the true values are 0.46 m at s = -3 and 2.32 m at most.
- m3. Any reception counts as heard, so a modem pinging after a host crash keeps a dead drone alive (liveness only). Require that the host writes the 2-bit frame counter.

## Final campaign: pre-registrable criteria (underwater side)

Frozen commit, engine hash recorded, held-out seeds, every run counted, INVALID rerun on the same seed.

1. **Safety:** 0 SAFETY_FAILURE, UNSAFE_ACTION or SUT_ERROR; P1 at least 1.0 m; at most one drone in the CR; Clopper-Pearson bounds reported (0 in at least 300 runs gives at most 1 %).
2. **Automatic premise monitors:**
   - (i) `LIVE_DECLARED_DEAD` = 0 under CLEAN, LOW and MEDIUM, with p50, p99 and max of live silence; HOLD if the max exceeds 35 s;
   - (ii) gate-frame error at most 1.0 m while admitted: 0 violations; while committed, at most 1, and with no consequence;
   - (iii) wrong validated fit: 0;
   - (iv) `NAV_BOUND_PREMISE_VIOLATED` = 0;
   - (v) every failure position outside E at the actual current.
3. **Liveness:**
   - `lost_drone_mutex` family, at least 60 fresh runs, at most 2 INCOMPLETE (upper bound 10 %; power 0.88 at a true 2 %);
   - the standard set: at least 17 of 18 per profile and seed, all PASS under CLEAN and LOW;
   - every INCOMPLETE assigned to a declared open class with its precursor; any other cause means HOLD.
4. **Near-gate failures in scope (still water), at least 5 seeds each:** at a queue point, committed at the hold point, and past the exit. P1 and P2 required. Plus n = 2 with a failure.
5. **Stress (held-out):** at least 5 seeds per scenario. P1 and P2 required everywhere; the mission is required inside the envelope; probes are mapped as a completion rate.
6. **Sensitivity, reported only:** compass bias of 3 and 4 deg, `MEDIUM_HT`, a jet at the opening, a vertical current at the gate.
7. **Report:** E exposure per scenario, |e_l| at the gate against bias, and corrections by type with their before/after error.

**Freeze: no.** Conditions: B1 decided ((a) in code or (b) in the claims), V listed under P2, monitors 2.i-2.iii added, s1103 re-described. Then yes, provided the final campaign carries criteria 1-5 on held-out seeds.
