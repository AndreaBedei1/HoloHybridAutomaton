# Review round 9 - formal verification (rule D guard, check_fleet, P3 contract)

Reviewed at 176b8b8 (`de4cc5b..176b8b8`). Every Python run was started at below-normal priority. Scratch scripts are kept outside the repository, in the session scratchpad (`r9/rule_d_purge.py`, `r9/s12.py`). Nothing in the repository was written except this file, and `formal/results` was not regenerated. I stopped one stray process of my own, a mistaken `python -` waiting on stdin; no other process was touched.

Reproduced:
- S12 UNSAT; S12m and S12l SAT.
- `tests/test_scope.py` and `tests/test_p3_contract.py`: 9 passed.
- The `exclusion_zone` encoding matches `scope.py:52-58` term by term (t*, the coast, the drift factor). A constant `wd` and `v` dominate time-varying ones, so the encoding is sound.

## BLOCKER

**B1. The zone guard of rule D does not work in operation. The round 8 B1 fix is ineffective.**
- `AcousticTracker.update` drops fixes older than `_KEEP_S` = 30 s (`acoustic.py:52`, `:213-214`).
- `t_silent_gate_s` is 70 s, so when `dead()` is first true the list `fixes[j]` is always empty.
- `acoustic_gate.py:198` then returns True ("never heard") without consulting `last_accepted`.
- Every silent drone is therefore declared dead after 70 s, wherever it was last heard.

I reproduced this with the repository's own harness (`test_status_payload._observer`). The only change is that the receiver keeps hearing drone 2 through `update()` once a second, instead of `tests/test_scope.py:90` setting `tr.heard[2]` directly.
- Hull last heard in the CR (w_h = 0): `dead = True`, claims `{}` at t = 72 s.
- Hull last heard 6.5 m before the gate (w_h = 0.3): `dead = True`, claims `{}` at t = 72 s.
- At t = 40 s the fix list is already empty, while `last_accepted` still holds the fix.

S4 (`check_status.py:193`) has the same harness shortcut, so S4 and the test pass vacuously for the "dead in the zone" case. In consequence:
- GUARANTEES:42 and :79 ("holds the gate for ever") are false for the code.
- The RC9 path of round 8 is open again for failures in E.
- It is also open for a premise-V violation by a live drone silent near the CR.

The campaign evidence of failures outside E is unaffected: there the result is the same.

*Fix:*
- `f = last_accepted.get(j) or (fx[-1] if fx else None)`, and "never heard" means `j not in tracker.heard`.
- Add an integration test and an S4 case that drive `update()` for at least 70 s.
- Add a mutation that restores the bug and must be caught.

## MAJOR

**M1. The margin of rule D does not bound where the drone failed** (`acoustic_gate.py:203-209`).
- *Motion margin.* It adds `v_bound · t_lost_s` = 0.7 m from the last *accepted* fix. The drone was heard alive up to `heard[j]`, which can be much later, after rejected or stale receptions. The motion term must be `v_bound · (heard[j] - f.t + t_lost_s)`.
- *Rejected fix as fallback.* A rejected fix (possibly a reflected arrival) is used when nothing was accepted. For that case use the launch position or keep the claim.
- *Depth.* The fix depth (from range and elevation) is not a measurement. Evaluate the zone on the deepest sample of `f.pts`.
- *Own bound.* The receiver's bound is unbounded, while the scenario premise uses a fixed `ONBOARD_MARGIN_M` = 1.5 m (`tests/test_scope.py:14`); the own bound of 1 m plus 0.7 m already exceeds it. State the margin budget as a premise and log the margin used per decision.

**M2. F2 is contradicted by the code and the claim is unqualified.**
- med12: `st_lost_at_queue` and `st_lost_committed` (n = 3, one failure in Q or C, inside the F2 premises) ended 0/6. The surfaced hull keeps the sonar-only precedence yielding for ever.
- The model releases precedence after T_RISE (`check_fleet.py:136`), which is the T_RISE concern of round 8 M2.
- GUARANTEES:45 declares this as an empirical limit, but GUARANTEES:86 and `check_fleet.py:28` still state recovery for n >= 3 without it.

*Fix:*
- Add premise H to F2: a dead hull leaves the precedence brackets of every queued receiver.
- Add a declared-limit check F2s4 (T_RISE = infinity for a Q/C hull, expected SAT).
- Make the P3 contract and the GUARANTEES row conditional on H. Present F2 as protocol-level.

**M3. Fairness and premise C are still unmonitored.**
- `referee/` has no all-fresh-gap monitor and no claim-latency monitor (round 8, criterion 3).
- The 35 s window is not derived from the campaign: med12 MEDIUM_HT had a live silence of 44.2 s on one link, which already implies an all-fresh gap above 35 s.
- *Fix:* compute both offline from `gate.silence` and the claims, and report them per run. A run that violates either is not evidence for F2 or P2.

**M4. The P3 implementation diverges from the document.**
- (a) Premise N uses `cfg.ref.e_ok` = 1.0 m (`p3.py:142`, `config.py:293`); the document says 0.5 m (`cfg.form.e_ok`, `config.py:192`). This matters: only e* + e_ok = 0.44 + 0.5 <= E_TOL links the ranking certificate to the tolerance. At 1.0 m a P3 miss can be caused by navigation with N held.
- (b) Premise A: only the "hears nobody" clause is implemented (`p3.py:154-159`). "No live sender silent for t_silent_s" is missing.
- (c) Premise S:
  - "n = 2: none while committed" is not checked (`p3.py:172`).
  - `NOT_ADMISSIBLE` (`:279-280`) is not handled by `refine_v8`. It falls through to P3_FAILURE (gate passed) or INCOMPLETE. An inadmissible run is thus counted as a genuine failure, against the document's premise S.
  - The referee uses margin 0 at the true position, while the code uses the onboard margin. Failures in the band between are safe stops counted as INCOMPLETE.
  - *Fix:* add a class OUT_OF_PREMISE_S (or INVALID) and apply the n = 2 rule. Report the band runs separately.
- (d) The `p3.py:11` docstring says D_b = max_k D_a(k); the code (`:266-268`) and the document use e0 of the fleet.

**M5. Attribution without causality: a controller deadlock can be classed PREMISE_VIOLATED.**
- *Gate premise* (`robustness_metrics.py:514`): any single sample of committed navigation error > nav_err_m, or any entry of a live drone in `gate.dead`, anywhere in the run, turns an INCOMPLETE gate into PREMISE_VIOLATED. There is no 2 s rule, no time link and no drone link. `self_induced` is one global flag taken from the first excess of any drone (`:554-558`).
- *W* (`p3.py:126-130`) judges feasibility of the controller's own `v_cmd`, which is endogenous: a controller that saturates or commands an escape makes W fail. Judge instead the feasibility of the velocity the protocol needs (v_slot_max along the path) under the true current.
- *N*:
  - The self-induced test looks 30 s before the first bad sample *inside* the window (`:144-147`), not before the onset of the excess episode. A self-induced excess that began before the window counts as external.
  - There is no magnitude test: a 0.6 m excess "explains" a 3 m miss.
- *O* (`:131-135`, `:152`) is proximity, not obstruction. A real obstruction already shows up as P1 pre-emption and extends the deadline, so O can only act where nothing obstructed the drone. Require the hull within the swept corridor from the drone to its slot, or drop O.
- *Masking* (`refine_v8:520`): parts all PREMISE_VIOLATED returns PREMISE_VIOLATED even when other requirements (`progress`, `not waiting`) also failed. Require every failed requirement to be explained.

*Fix:* attribute a premise only if it is violated within the failed part's window, for the drone concerned (or its expected neighbours), and its magnitude can account for the miss: for example, the excess at least the residual minus E_TOL. Add tests for each loophole.

**M6. The class wording overclaims relative to the proof.**
The document correctly says that D = e0/v_close + 5 + T_HOLD is a nominal requirement, not the certified bound (`P3_CONTRACT.md:28`). Yet P3_FAILURE is called "a genuine liveness failure" (`:49`, `refine_v8` docstring). From e0 = 2.4 m the certificate gives:
- (2.4 - 1.11)/0.014 + (1.11 - 1.0)/0.0115 ≈ 102 s, against D = 37.9 s.

A late run with premises held is consistent with the proof. Split the class:
- **P3_LATE:** deadline missed, recovered by D_cert.
- **P3_FAILURE:** not recovered by D_cert = certified ranking time + T_HOLD + extensions, or by the run end with the run uncensored.

Only P3_FAILURE contradicts liveness.

**M7. `formal/results` at 176b8b8 is stale.** The committed SUMMARY is from 2026-11-09; the working copy is modified and `fleet.json` is untracked (round 8 m3). Regenerate at the frozen commit with its hash and the z3 version.

## MINOR (acceptable as declared limits)

- m1. **Relative composition.** Errors are measured after mean translation. With per-drone residuals e* + e_ok, the deviation from the mean can reach 2(n-1)/n · 0.94 m > E_TOL for n >= 3 in the worst case. State this, or rely on the relative corrections.
- m2. **Recovery is reached, not kept.** P3 requires the band to be reached and held once. Divergence afterwards is only judged at the end (integrated_short s1100). Report the largest excursion after recovery.
- m3. **Vacancy deadline when censored.** The vacancy part (`p3.py:240`) has no censoring flag.
- m4. **One gate only.** P3a and P3b use `gates[0]` only. State this, or loop over the judged gates.
- m5. **F0s and fairness.** F0s at depth 1 carries no fairness constraint, which in effect resolves round 8 m1. Say so in the docstring.
- m6. **Coverage.** No scenario shows F2s2: n = 2 with a committed failure, with a predicted INCOMPLETE.
- m7. **v_close.** v_close = 0.10 m/s is [PROT]. Check it against the measured catch-up speeds of med13.

## Answers

**1. Round 8.**
- B1: resolved in design, **not in code** (B1 above).
- B2: resolved in the model. The deaf-receiver guard and the committed claim are modelled as in the code (`check_fleet.py:109-123`); F2s2 and F2s3 are declared.
- M1: stated as premise C, but not monitored.
- M2: partially. The localization block is now declared, but the surfaced-hull path refutes F2 (M2 above).
- M3: the window was widened but neither derived nor monitored.
- M4: resolved (S12 is consistent with `scope.py`).
- M5: the scope check per run is resolved, and in-zone failures are INVALID.
- M6: declared.
- The unbounded argument (F0 inductive + F2 from any invariant state) is valid **within the abstraction and under its premises**, now including H.

**2. P3 contract.** The structure is sound: three parts, deadlines from measured e0, premises monitored on ground truth, and the self-induced rule as a principle. As implemented, it is not yet free of loopholes:
- A deadlock can be classed PREMISE_VIOLATED by the gate premise, W, N or O (M5).
- An inadmissible run is misclassed (M4c).
- The N threshold contradicts the document (M4a).
- No real failure is hidden as NA in what I checked: P3a and P3b are NA only when the gate is missed, which v3 fails anyway, or when only one healthy drone remains.
- The deadline-to-certificate relation is stated correctly in the document, but the class names contradict it (M6).

**3.** BLOCKER B1; MAJOR M1-M7 before freezing; m1-m7 may be declared.

## Pre-registrable acceptance criteria (round 8 list, adjusted)

1. **Formal.** At the frozen commit, committed with the hash and the z3 version:
   - the whole suite;
   - S4, plus a rule-D integration test through `update()` over at least 70 s, plus the purge mutation, which must fail;
   - F2 with premise H, and F2s4 SAT.
2. **Safety.** No P1 or P2 violation, dead hulls included. A violation in a run with its premises held rejects the freeze.
3. **Premise monitors in every run:**
   - NAV_BOUND, scope (E with margin 0 and with the onboard margin), V, C and the all-fresh gap (at most 35 s);
   - N at 0.5 m;
   - W on the protocol-required velocity.

   A run that violates a premise is reported per premise and never counted as evidence. If PREMISE_VIOLATED exceeds 10 % in a profile, that profile is reported as outside the envelope.
4. **Liveness.** In admissible runs with their premises held:
   - P3_FAILURE (certified bound) = 0;
   - P3_LATE and INCOMPLETE reported by mechanism, with one-sided 95 % Clopper-Pearson upper bounds;
   - at least 59 admissible runs per family, for a 5 % bound at 0 events.

   Kaplan-Meier curves of the recovery time against both D and D_cert.
5. **Coverage.** Include:
   - n = 2 committed failure (predicted INCOMPLETE);
   - the last survivor;
   - a failure 0.5 m outside E at the maximum current;
   - an in-zone demonstration whose predicted outcome is a safe stop with P2 held, which exercises the fixed rule D;
   - the stacked queue;
   - `st_lost_at_queue` and `st_lost_committed` as declared-limit probes (predicted INCOMPLETE).
6. **Seeds and provenance.** Fresh seeds, frozen hashes, stop rules written before the first run.

**Freeze: no.** Conditions:
- fix and test B1, with S4 driven through `update()`;
- M1 margin;
- M2 premise H in F2, GUARANTEES and P3;
- M3 monitors;
- M4 implementation aligned with the document, and the S class;
- M5 causal attribution with loophole tests;
- M6 class split;
- M7 results regenerated and committed;
- re-run the referee on med11-med13, and the P2 regression scenarios near the gate.

## Re-check at 1aa207a

Scope: only B1 and M1-M7, plus the v8 criteria. Every run was at below-normal priority, and the scripts are in the scratchpad (`r9b/`). Nothing was written but this section.

Reproduced:
- My purge scenario, with drone 2 heard through `update()` every second:
  - hull in the CR, and 6.5 m before the gate at w_h = 0.3: the fix list is empty from 40 s, `last_accepted` holds the fix, and the claim is kept at 72, 75 and 100 s;
  - hull 25 m away: released at 72 s;
  - never heard: released.
- S4: 0 of 300 violations. S4b: 25 of 120 (caught). S4m: 29 of 60.
- `test_scope` and `test_p3_contract`: 10 passed.
- With the new precedence term: F0s and F2 UNSAT for n = 2, 3, 4; F2s4 SAT.

**B1 - RESOLVED.** Rule D reads `last_accepted`. Never heard means absent from `heard`; never located holds the gate. S4 runs on real receptions. Minor: S4 does not cover the "heard after the fix" motion term or the never-located branch.

**M1 - RESOLVED.**
- Motion is counted from the fix to the last reception, plus t_lost.
- There is no rejected-fix fallback.
- The vertical margin raises h0, which is conservative.
- The uncapped own bound only adds conservatism. Its band is declared (`st_lost_after_exit`, v8 items 7 and 10) and logged in `gate.rule_d`.

Minor: the payload branch makes rule D rest on H6, which the GUARANTEES rule-D row omits. Its motion term starts at `f.t`, not `f.t_tx`.

**M2 - PARTIAL.** Premise H is in `check_fleet.py` and in P3_CONTRACT, and F2s4 is SAT. The composition row, GUARANTEES:89, still claims recovery for n >= 3 without H or F2s4. This is a documentation fix.

**M3 - PARTIAL (acceptable).** The fairness gap is monitored per run, and premise C is declared unmonitored. A fairness excess has no class consequence, which is conservative because the thresholds count complete runs.

**M4 - RESOLVED.**
- N uses e_ok = 0.5.
- Both clauses of A are checked.
- An n = 2 committed failure is inadmissible, and NOT_ADMISSIBLE is classed PREMISE_VIOLATED.
- The band runs fall into the declared rule-D class.
- The docstring is fixed.

**M5 - PARTIAL (acceptable).** Under the validation profiles, N and gate-nav are tags, which closes the main loophole; `self_induced` is now per drone and W uses `v_mis`. Still present:
- no time or magnitude link beyond the validation profiles;
- O is proximity;
- `refine_v8` still masks a failed `not waiting`.

None of this can create a pass: items 5-7 count PASS or P3_LATE over all runs.

**M6 - RESOLVED (conservative).** P3_LATE means met within the run; P3_FAILURE means never met. A P3_FAILURE censored before D_cert (≈ 102 s at e0 = 2.4 m) does not contradict the certificate.

**M7 - OPEN by plan.** The results are to be regenerated at the freeze; the working copy is still modified and `fleet.json` is untracked.

**v8 criteria - acceptable.**
- No in-zone probe: the owner's scope decision. The holding branch is exercised end to end by `st_lost_after_exit` (a safe stop, judged on P1 and P2) and at unit level by S4 case 3, S4b and `test_scope`.
- No n = 2 committed run: minor m6, covered by F2s2 and by the PREMISE_VIOLATED class.
- Premise V has a consequence, safety is judged on all 448 runs, and the stop rules are fixed before the first run.

**Freeze: yes**, under these conditions:
1. Before the tag, GUARANTEES:89 is made conditional on premise H with F2s4, and the rule-D row names H6.
2. M7 is regenerated and committed at the frozen commit, with its hash and z3 5.1.0.
3. Item 10 reports every PREMISE_VIOLATED run with its premise, drone and window.
4. Any P3_FAILURE censored before D_cert is reported as such.
