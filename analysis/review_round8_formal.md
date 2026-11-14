# Review round 8 - formal verification (rule D, exclusion zone, check_fleet)

Scope: `8fdcbb4..3f35900` (rule D, `holo_fleet/scope.py`, S4/S11/S12, `formal/check_fleet.py`, `docs/GUARANTEES.md`), med11 and st1 logs. Every Python run was started at below-normal priority (`Start-Process ... PriorityClass='BelowNormal'`). Nothing in the repository was written except this file; `formal/results` was not regenerated. The reviewer's scripts are in the session scratchpad: `fleet_variants.py` (a copy of `Fleet` with switches for code behaviours), `s12_variants.py`, `zone_geom.py` and `fair_gap.py`.

Reproduced: S12 UNSAT and S12m SAT; all 16 checks of `check_fleet.run()` as expected (F2 at n = 4 takes 99 s).

## BLOCKER

**B1. Rule D re-opens RC9 for failures inside E, and deleting the scenarios hides this.**
A dead drone is dropped from every gate input: claims (`acoustic_gate.py:177`), possible occupant (`:155`, `:327`, `:363`), heard-inside (`:301`), and the phantom discard (`:135`). The bench shows a hull under the top bar staying pinned in the CR (`results/calibration/buoyant_ascent.json`, `pinned_in_cr` for |s| <= 0.2). Here is the path:
1. After 70 s the hull is dead.
2. The acoustic-only episode clears (`gate_perception.py:243`).
3. Stage 2 finds nothing in the masked part of the CR (`:275`).
4. A drone with priority is admitted. That is RC9 (med7 s981): a P2 violation and a contact.

The reproducer (`lost_drone_mutex_carried`) and `gate_stalled_occupant` were deleted in daeae63. GUARANTEES:82 says "no special handling"; the design is in fact now unsafe there. GUARANTEES:40 still cites the removed drift reach.

E is also large. I computed it with `scope.py` on G06: E starts at s = -2.18 m at w_h = 0, -5.46 at 0.1, -8.74 at 0.2, -12.0 at 0.3 and -18.6 at 0.5 m/s, with the same extents beyond the gate. The queue points (n = 2: -4.18; n = 3: -5.41) are in E from w_h = 0.1, and the waiting point (+6.5) from 0.2. At the stress currents, P2 therefore assumes that no drone fails during the gate phase.

*Fix (preferred):* exclude a silent drone only if its last known region is outside E for the declared current. That region is the last accepted fix region plus slack, or the payload ball, grown by `coast_m`; for a never-heard drone, use its launch position. Otherwise keep the claim (safe, INCOMPLETE). E then becomes a liveness scope only, and premise-V violations by drones last heard near the gate are covered as well.
*Alternative:* state P2 as conditional on "no failure in E", and keep the deleted scenarios as out-of-scope demonstrations with their predicted outcome.

**B2. F2 does not hold for the code at n = 2 or for the last survivor (concrete deadlock).**
`check_fleet.py` leaves out two behaviours of the code:
- (i) Rule D needs the receiver to hear another drone (`acoustic.py:312-314`), so at n = 2 `dead()` is never true. The model applies rule D unconditionally (`check_fleet.py:91`).
- (ii) Failures in C are forbidden (`:119`). But at zero current, the committed transit from the queue point down to s ≈ -2.2 m is outside E. A committed drone reports GP_COMMITTED (`passage.py:62`), and that claim holds until rule D.

I added both behaviours to the model:
- n = 2 from the launch: SAT at 78 steps, trace `AA AA AQ AC ... QC QF QF ...`. The survivor stays queued for ever.
- n = 3 with two failures: SAT, `QFF` for ever.
- The same n = 2 model with check_fleet's rule D: UNSAT.
- n = 3 with one failure in C: UNSAT at K = 30.
- (i) alone, with failures in A, Q, B, D: UNSAT.

In the code, `claims()` keeps the dead drone and the latch stays BUSY (`gate_perception.py:213-214`). No scenario covers this.
*Fix:* model (i) and (ii). Then either restrict GUARANTEES:83 to "at least one other live drone heard", or change the code (B1's guard does not help, because the transit is outside E). Add `st_gate_pair` with a failure of the committed drone at s ≈ -3 m.

## MAJOR

**M1. The assume-guarantee interface is stronger than what the local checks give.**
F0 assumes a latch that is BUSY instantly and for as long as another live drone is in C or X (`check_fleet.py:98-106`). The model allows stops of any length in C and X before L (`:118`). The local checks give less:
- M3 is bounded at 42 s and needs a speed floor; S2h is bounded at 32 s.
- Beyond those bounds, S2hs and M3hs give counterexamples (a stop with no claim heard).
- In the code, two drones can be committed at once; P2 then rests on M3/S2 timing.

So "at most one in C∪X" holds in the abstraction, but GUARANTEES:83 states it of the system. A premise is also missing. **Premise C:** every queued receiver hears a claim within the S2h bound of 32 s. V does not imply C: V allows 69 s, and the observed live silences reach 25.2 s (st1) and 31 s.
*Fix:* state the interface contract as an assumption, with its support (S1, S4, M3, S2h, C), and monitor C in every run.

**M2. The liveness abstraction leaves out the blocking paths of the code.**
- Under the invariant, the give-up branch (`check_fleet.py:127-129`) is unreachable: `live_x` needs a second drone in C∪X. So admission never blocks in the model.
- The main observed liveness failure is therefore outside F2: s1103, where `loc_ok` was never validated (`flows.py:331`; log:280). So is the commit-hold-give-up livelock (GUARANTEES:41).
- With no echo, the release is immediate (no `t_occ_max`, `t_clear` or `exit_confirm`).
- `T_APP` is not derived.
- `T_RISE` = ceil(2.0/v) (`:60`) uses an unexplained 2 m. A hull 3.5 m to the side leaves the 30-degree LEFT cone after about 2.5 m of rise (about 51 s). At a shallower gate, a surfaced hull can stay within `queue_bracket_m` for ever.
- The code's precedence counts any LEFT echo within 7.5 m, not only a hull at Q.

"No deadlock, no livelock" and "110/150/190 s" therefore describe a protocol skeleton.
*Fix:* state these as premises of F2 (stage-2 admission, the release delays, hull clearance of the precedence brackets, the last checked numerically per layout and gate depth), or present F2 as protocol-level.

**M3. The logs refute the fairness premise at its stated window.**
F2 assumes that within every 10 s there is an instant at which every live drone is fresh at every receiver. I measured this on st1 (`gate.silence`), with any reception within 2.5 s counting as fresh, which is weaker than an accepted fix.
- Longest gap: 26.1 s (`st_gate_line3_cross` s2000) and 13.5 s (`st_lost_right_tail` s2001).
- 2 of the 28 runs with n <= 4 exceed 10 s; the six-drone runs reach 93-97 s.

With a 35 s window, F2 at n = 3 is SAT at K = 30 and UNSAT at K = 40. V is per link, while F2 needs simultaneity.
*Fix:* derive the window from accepted-fix freshness, report K(window), and monitor it in every run.

**M4. S12 and `scope.py` leave out the vertical coast and the rise lag, and the above-the-top branch has no proof.**
S12 requires h0 > 0 (`check_status.py:358`), while `scope.py:48-50` puts every point above the top outside E. I re-ran the encoding with a 3-D coast of at most `coast_m`; all three cases are outside E by `scope.py` and enter O:
- h0 = 0, d0 = 0 with a 0.5 m downward coast: SAT.
- W = 0.3: a hull 0.06 m below the top at d0 = 1.39 m (reach 1.375 m) enters O at t = 5 s.
- A 3 s lag before the rise speed: d0 = 1.5 m (reach 1.47 m) enters at t = 4 s.

The stacked queue (dz = ±1.75 m, above O's half-height of 1.64 m) and committed drones descending to the axis are exposed.
*Fix:* rise needed = max(h_top - dz + cz_max, 0) + v·τ, keeping the points within cz_max above the top. Drop h0 > 0 from S12 and measure τ (the bench reports terminal speeds only).

**M5. No run checks the scope premise.**
Only `tests/test_scope.py:47-64` exists: one heuristic point per scenario.
*Fix:* compute from the truth, in every run: a premise flag from the failure position and the current bound, the minimum distance of each dead hull to O, and each drone's exposure (the fraction of time it spends in E).

**M6. The scope was set post hoc.**
E was decided after s960, s1068 and s1071 (log:237), and the scenarios inside it were deleted. The paper must say so, and E (the formula and the current of each scenario) must be pre-registered before the final campaign.

## MINOR

- m1. F0s is checked under fairness (`check_fleet.py:94-96`, `:181`). Without fairness, and with (i) and (ii), it is still UNSAT for n = 2, 3, 4. Add these checks to the suite.
- m2. The code requires hearing "any other" drone (`acoustic.py:314`), not "the others". State V per link: every live sender is heard at every working receiver within every 70 s window. Add a false-death mutation (P2 must fail).
- m3. `formal/results` is not committed (`SUMMARY.*` modified, `fleet.json` untracked). Regenerate it at the frozen commit, with the commit hash and the z3 version.
- m4. Present S12 as consistency of the zone formula with the stated kinematics; the physics are premises. Report K in steps.

## Formal and empirical claims (Q3)

**Within the abstraction.** F0 and F2 (BMC from an over-approximation of the reachable states, with D absorbing and fairness in every window) together give a valid unbounded recovery argument. S12 holds given its kinematics.

**Randomized, not proofs.** S1, S4 and S11.

**Empirical premises.** V, C, fairness, E's physics (coast, lag, rise speed, current, flow through the frame), T_RISE and T_APP, stage-2 admission, and the n >= 3 condition of rule D.

## Pre-registrable acceptance criteria (Q5)

1. **Formal.** At the frozen commit the suite passes, including:
   - F0 without fairness;
   - F2 with (i) and (ii), with the window taken from data;
   - S12 with vertical coast and lag;
   - the false-death mutation (SAT).
   The results are committed with the commit hash.
2. **Safety.** No P1 or P2 violation, dead hulls included. A violation in a run whose premises hold rejects the freeze. A run that fails a premise is reported as a premise failure, never as evidence.
3. **Premise monitors in every run, all reported:**
   - NAV_BOUND;
   - scope: the failure position against E, and no dead hull ever in O;
   - V: the maximum live per-link silence below 70 s, with its distribution;
   - C: a claim heard within 32 s of the commit;
   - fairness: the longest all-fresh gap no longer than the F2 window;
   - no live drone in `gate.dead`.
4. **Liveness.** In runs whose premises hold, every live drone re-forms within K·DT plus the code delays declared beforehand. Report Kaplan-Meier curves and the INCOMPLETE runs by mechanism. Pre-register the threshold and n, with a one-sided 95 % Clopper-Pearson bound.
5. **Coverage.** Include:
   - n = 2 with a failure in the committed transit;
   - the last survivor;
   - a failure 0.5 m outside E at the maximum current;
   - a stacked queue with a descending committed drone;
   - out-of-scope runs, reported separately against their predicted outcome.
6. **Seeds and provenance.** Fresh seeds; frozen code and engine hashes; stop rules written before the first run.

**Freeze: no.** Conditions:
- resolve B1 (scope guard, or a declared conditional P2 with demonstrations);
- resolve B2 (model and code, or a declared limit);
- restate GUARANTEES:83 for M1-M3;
- correct S12 and `scope.py` (M4);
- add the scope monitor (M5);
- re-run the formal suite.
