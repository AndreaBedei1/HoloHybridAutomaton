# HoloHybridAutomaton: B2-only architecture review, root-cause analysis and consolidation plan

Analysis phase, 2026-11-03. Branch `b2-acoustic-aided` at `a6efd0f`.

Nothing in the implementation, configuration, results or Git history was changed. The only additions are this
untracked folder `analysis/`:

* this report;
* the read-only diagnostic scripts in `analysis/scripts/`, which read the logs of `results/robustness/raw_final`;
* three raw outputs, `forensics_v.txt`, `p3_failures.txt` and `witness_crossings.txt`;
* the two independent reviews, `review_A_formal_hybrid.md` and `review_B_marine_experimental.md`;
* the literature review in `analysis/litreview/`.

Every number below can be reproduced with those scripts, run with `C:\Users\andrea.bedei3\.conda\envs\holo_fleet_ha\python.exe` from the repo root.

Conventions:

* **CR** is the critical region of the gate: |s| <= 0.8 m, |l| <= 1.0 m, |dz| <= 1.0 m in the gate frame.
* **s** is the along-gate coordinate; negative values lie before the gate.
* **"truth"** means the referee's ground truth.
* **"onboard"** means the drone's own state log.

Findings are labelled by kind:

| Label | Meaning |
|---|---|
| **[WRONG]** | demonstrably wrong |
| **[INEFF]** | merely inefficient |
| **[LIMIT]** | an expected limitation |
| **[UNDET]** | cannot yet be determined |

---

## 0. Executive summary

1. **Safety held, but P2 rests partly on kinematics, not on the formal argument.**
   * In the 624-run campaign there were no collisions and no P1 or P2 violations. The minimum separation was 2.26 m, against d_safe = 1.0 m.
   * Of the 536 commits in the 195 gate runs, none happened while another drone was truly inside the CR.
   * The smallest time between an occupant leaving the CR and the next drone entering it was 11.5 s.
   * The seven premature releases are real, but they produced no unsafe action. The occupant was a healthy drone already
     0.1-0.5 m from the exit face, and EXITING still blocks a commit. The next drone then needs 7-12 s to travel from its
     queue point to the CR.
   * Safety in these cases therefore came from kinematic margins: a healthy crossing plus the travel distance from the
     queue. It did not come from the soundness of the exit witness. The witness accepted transient false DYNAMIC echoes
     under MEDIUM sonar noise **[WRONG]**.
   * In the five B2 cases, every occupant had been heard within the last 2.5 s and the acoustic exit test had not been
     passed, yet the sonar witness is never checked against acoustic evidence **[WRONG]**.
   * More generally, under MEDIUM 69 of 260 sonar exit witnesses match no drone within 1 m (0 of 95 under CLEAN and
     LOW). At least 62 of them released the latch after the true exit, on time but without evidence.
   * The latch also misses crossings: in 9 MEDIUM runs a queued drone kept FREE while another drone crossed the whole
     CR, because the occupant's echoes were classified STRUCTURE. The observer never had priority, so precedence (M1/M2)
     was the barrier **[WRONG]**.
   * Under criterion v2 §2 item 3 as written, the 7 premature-release runs are safety failures. The classifier labelled
     them PASS. This report does not change their class; it reports the discrepancy (Section B.4).

2. **The dominant cause of incomplete B2 missions is gate liveness, not formation control.** In 11 of the 28 non-PASS B2
   runs the primary cause is at the gate:
   * In 8 runs a healthy, committed drone stalls 1-3.5 m before the CR for 100-260 s, until the deadline. Navigation
     error near the gate (0.3-1.0 m) leaves the bars unexplained, so they are treated as obstacles. The map fit cannot
     produce an accepted correction, and the relocalization hold has no bound and no recovery **[WRONG]**.
   * In 1 run a phantom corridor echo keeps the latch BUSY for 204 s, although the acoustic tracker had positively
     excluded both other drones **[WRONG]**.
   * In 1 run the acoustic "heard inside" test blocked the timeout fallback for at least 36 s after the CR was empty
     **[WRONG]**.
   * In 1 run a relocalization delay produced a false vacancy that the run did not wait to see resolved (evaluation
     inconsistency, see point 6).
   * No healthy drone ever stopped inside the CR. The dwell time inside the CR has a median of 3.1 s, p99 5.0 s and a
     maximum of 12.5 s.

3. **The second cause is a design defect of the formation clock, not the 0.5 m threshold.** In 11 runs, false
   missing-neighbour detections slow the drones' private formation clocks (`tau`) asymmetrically. Nothing ever
   re-synchronises them. The resulting along-track offsets (1.3-3.5 m) match `v_plan * (tau_k - mean tau)` in sign and
   magnitude, while each drone believes it is within 0.02-0.2 m of its slot **[WRONG]**. In 9 of these 11 runs the error
   was growing at the deadline.

4. **Five further P3-deg failures** come from inconsistent vacancy decisions: some survivors declare the slot vacant,
   others do not, and slots are falsely re-occupied **[WRONG]**. One more run is a slow recovery hampered by separation
   warnings **[LIMIT]**.

5. **Neither more time nor the 1.0 m tolerance would have changed most outcomes.**
   * More time would plausibly help in at most 3 of the 28 runs.
   * With a 1.0 m tolerance, 2 runs (final errors 0.60 m and 0.80 m) would satisfy P3. Both would still fail, because a
     drone is still "waiting" for a neighbour that is present.
   * The other 23 runs are stalled or diverging.
   * The 1.0 m tolerance should be a referee and acceptance criterion with explicit hysteresis and a terminal
     requirement. Section E.3 explains why, and why the controller thresholds should stay as they are.

6. **The evaluation has inconsistencies that must be fixed before any new campaign** **[WRONG]**:
   * All sensor-CLEAN cells used the long exploratory horizon instead of the deadline. As a result, 4 of the 48
     "PASS" results of B2 with a MEDIUM channel and CLEAN sensors finish after the deadline that applies to every other
     cell.
   * The deadline of every protocol came from the B0 CLEAN runs.
   * The terminal monitor and the classifier use different requirements.
   * The P3 episode definition is vacuous for an error that stays between e_ok and e_lost.
   * The SAFE_DEGRADED class mixes progressing runs with deadlocks: any sensor-fault FAILSAFE entry turns a missed
     requirement into SAFE_DEGRADED (`robustness_metrics.py:334-342`), so "0 LIVENESS_FAILURE" carries no information.
   * Criterion 3 (no release while occupied) is not enforced by the classifier.
   * The B2 code changed after the freeze: ad4fb5e (acoustic exit by range) came after 1dc9d3f, and the earlier B2 runs
     were discarded without a record in ACCEPTANCE_CRITERIA.
   * The claimed B2 benefit (B2_MEDIUM vs B0 under MEDIUM-ALL, p = 0.0074) does not survive the proper control:
     B2_MEDIUM vs B2_BLACKOUT gives 7 vs 3 discordant runs, p = 0.34. The pairing is nominal because engine noise is
     unseeded.

7. **B2 currently makes the gate slower than sonar-only operation** **[WRONG]** (liveness only):
   * In B2_CLEAN, acoustic exits arrive a median of 52 s after the true exit. With a silent channel, the timeout fallback
     arrives after 26 s.
   * Meanwhile the B2 "heard inside" test blocks the timeout.
   * The 90th percentile of the queue wait is 88.0 s in B2_CLEAN and 55.5 s in BLACKOUT-CLEAN (corrected after
    fixing the parsing of the `at_queue` string).
   * 12 of 15 B2_CLEAN gate runs have an idle interval of at least 20 s (a healthy drone queued, the CR empty, nobody
     committed), up to 68.2 s. The figure is 1 of 15 in B0 CLEAN and 0 of 15 in BLACKOUT-CLEAN.

8. **Formal verification.** The local automaton properties are sound for the encoded model: determinism, completeness,
   priority, the precedence uniqueness M1/M2, and the P1 chain S0-S4 under its envelope. The P2 and P3 arguments,
   however, carry assumptions the campaign shows to be violated or unmodelled:
   * the exit witness is assumed sound;
   * the corridor detection is assumed perfect (9 missed crossings);
   * the occupant is assumed to cross the CR while healthy;
   * the private formation clocks are assumed consistent;
   * the navigation error is assumed bounded and covered by the map aiding.

   The P1 chain also carries assumptions that the campaign does not fully meet:
   * the echo envelope is exceeded under LOW as well as MEDIUM (per-run mean 0.86 % and 3.6 % beyond +eps_far; data
     age up to 0.3 s under LOW, above tau_max = 0.25 s), so P1 under LOW and MEDIUM is tested, not proved;
   * S1 assumes both drones brake. With one drone braking at the configured a_brake = 0.5 m/s², the BMC is SAT at
     d_warning = 2.4 m. At the calibrated per-drone capability of 0.74 m/s² it is UNSAT.

   "P3 proved" holds only for tracking the drone's own estimated slot. Section F gives the full property-evidence matrix.

The prioritised plan is in Section G and the validation plan in Section H. The plan has seven steps:

1. instrumentation and corrected metrics;
2. B2-only consolidation;
3. a two-stage gate entry with witness and veto fixes;
4. bounded relocalization and recovery;
5. re-synchronisation of the formation clocks;
6. identity-based vacancy decisions;
7. frozen v3 acceptance criteria, then targeted diagnostics, then the frozen campaign.

---

## A. Current architecture: how B2 works, what is correct, what still depends on B0

### A.1 How B2 works (as implemented)

**Sensing.** Each drone carries:

* six wide-cone single-beam sonars, one per body axis, with a near-field widening;
* dead reckoning from DVL, IMU, compass and depth (`perception/nav.py`);
* map-aided relocalization near the gate (`perception/structure_aiding.py`).

**Beacon.** Each drone broadcasts a one-way TDMA identity ping with no payload and no reply (`sim/beacon.py`). The
receiver `perception/acoustic.py` builds a fix from it:

* range from the one-way travel time, which needs synchronised clocks;
* azimuth and elevation from an inverted-USBL-style array;
* an error region from the declared envelope;
* per-identity tracks with kinematic and depth gating.

**How acoustic data enter the system:**

* **Formation perception** (`perception/formation_perception.py`, `perception/perception.py`) uses them for:
  * the present / displaced rules (A-P, A-D);
  * relative residuals (`acoustic_corr`);
  * reserving the echo near a heard drone for that drone.
* **The gate occupancy latch** (`perception/gate_perception.py` + `perception/acoustic_gate.py`) uses them in two ways:
  * as a second way into EXITING ("acoustic exit", rules A1-A3), consulted only when the sonar witness gave nothing;
  * as a veto on the t_occ_max fallback (`heard_inside`).
* **They are never used** by separation, collision avoidance, precedence or the sonar witness. This is checked
  statically by B2-S and `tests/test_b2_isolation.py`.

**Automaton** (`ha/spec.py`). Ten modes with guard-encoded priority: FAILSAFE > CA > SW > MUTEX_PASS > MUTEX_YIELD >
MUTEX_APPROACH > FORMATION_RECOVERY > formation keeping. The gate edges are:

* **commit** needs `at_queue ∧ has_prio ∧ ¬occ_busy` (`spec.py:153-154, 234-242`);
* once committed, **pass_continue** needs only `calm ∧ committed ∧ ¬passed` (`spec.py:217-224`). The occupancy belief
  is not re-checked after the commit.

**Pass flow** (`control/flows.py:236-269`). The pass path runs from the queue point, to the gate axis, through the gate.
A relocalization hold at s_hold = -(0.8 + 0.229 + 0.15) = -1.18 m stops a committed drone until `loc_ok` is true. The
hold alternates with a back-off every RELOC_PERIOD_S = 6 s. There is no bound and no exit other than `loc_ok`.

**Formation.** Each drone has a slot on a shared path, driven by a shared FormationClock, evaluated at the drone's own
private formation time t - tau (`control/formation_time.py`):

* a perceived missing neighbour slows the drone's own clock (reduced progress) for at most t_rejoin per episode;
* `tau` is kept afterwards, and nothing re-synchronises it across drones;
* after t_rejoin the slot is declared vacant (DEGRADED_FORMATION) and may later be re-occupied.

### A.2 What is already correct (with evidence)

* **Determinism, completeness and the hazard priority of the guards** are proved by z3 for every mode: D1, D2, H1-H3,
  G1-G3 and L1, 93 checks in total, `results/robustness/formal_all3.out`. There were no runtime determinism or
  observation-consistency violations in the 624 runs.
* **P1 chain:**
  * the association-free distance bound S0 (randomized, about 35 000 scenes);
  * the BMC S1a plus k-induction S1b, escape tables S2a-d, S3 and the blackout check S4;
  * empirically, a minimum true separation of 2.26 m in 624 runs and no contacts.
* **Precedence:** never two PRIORITY drones, and exactly the leftmost drone leads (proof M1, M2 for n = 2-6).
* **B2 isolation and acoustic envelope soundness** for the exit and exclusion decisions: B2-S, B2-A1 and B2-A2.
* **No commit while the CR was truly occupied:** 0 of 536 commits. There was no simultaneous occupancy, and the minimum
  entry gap was 11.5 s (`analysis/scripts/gate_metrics.py`).
* **No healthy drone stopped inside the CR.** Over 536 visits the dwell time inside the CR had a median of 3.1 s, p95
  3.3 s, p99 5.0 s and a maximum of 12.5 s. FAILSAFE and CA interrupted crossings for about 20 s each over the whole
  campaign, and never permanently (`analysis/scripts/cr_dwell.py`). This supports t_occ_max = 25 s and V_LO = 0.12 m/s
  for healthy crossings.

### A.3 Dependencies on B0 that a B2-only consolidation must address

| Area | Dependency | What B2-only needs |
|---|---|---|
| Configuration | `config.py:283-291`: `BeaconConfig.mode = "NONE"` by default, so B0 is the default runtime, and every test, demo and formal script runs B0 unless told otherwise | `ACOUSTIC_RB` becomes the default; acoustic outage is an operating condition (the BLACKOUT channel), not a protocol |
| Perception | B2 is layered on top of the sonar-only perception. With a silent channel it is exactly B0 (`perception.py:42`, `gate_perception.py:24-26`) | Keep the sonar-only path as the **outage-degraded mode of B2**. Removing B0 means removing B0 as a *protocol*, not removing sonar-only operation |
| Gate soundness | `acoustic_gate.py:30-36` rests on "the B0 assumption that every entry into the CR crosses the FRONT corridor visibly (formal M3/M4)". The acoustic exit only adds a way into EXITING; acoustic evidence never vetoes the sonar witness | Re-state the B2 gate argument with acoustic evidence as both positive and negative evidence (Section G.3) |
| Formation | Reduced progress and the vacancy logic were designed for range-only sensing without identity. B2 adds identity only as hints (A-P, A-D, reserve) | Use identified fixes for presence, absence and re-occupation decisions and for clock re-synchronisation (G.5, G.6) |
| Evaluation | The deadline of every protocol comes from the B0 CLEAN runs (`run_sensor_robustness_campaign.py:169-170`); `plan()` always inserts B0 CLEAN; `analyze_campaign.py` pairs B2 with B0; `scripts/analyze_b2.py` | Derive deadlines from B2 or from geometry (G.7); remove the B0 pairing; archive the B0 results as a historical baseline with a manifest |
| Tests | Defaults are B0; the B2 logic tests use `B2_LOGIC` | Make B2 the default; keep explicit outage (silent channel) tests |
| Formal | M3, M4 and S* encode the sonar-only logic, which stays necessary as the outage mode. B2-A1, B2-A2, B2-M4g and B2-S are add-ons | Re-run everything; group the evidence as "B2 nominal" and "B2 under acoustic outage" |
| Documents | GUARANTEES, ACCEPTANCE_CRITERIA, B2_ACOUSTIC_AIDED_COORDINATION (open issue O-10), README, REPORT.new.md: "B0 guarantee", B0 vs B2 tables. `GUARANTEES.md` still says B2-M4g is "not available", while `formal_all3.out` reports it as holding for n = 2 and 3 **[WRONG]**. The README reports "186 / 186" checks; `formal_all3.out` has 205 **[WRONG]**. GUARANTEES does not state the P1 premises (mutual detection, both drones braking, assumption D) | Rewrite the evidence statements. Call B2 "one-way acoustic identity beaconing (broadcast-only, no handshake)", never "communication-free" |

**Terminology.** B2 is not communication-free. A one-way identity ping with synchronised clocks transmits identity and,
through the shared clock, timing. It is a broadcast-only, payload-free communication channel that serves perception:

* no positions, velocities, commands, priorities or automaton states are exchanged;
* there is no request-response and no acknowledgement;
* decisions stay local.

It also needs pre-mission clock synchronisation and a calibrated receive array (`ACOUSTIC_CHANNEL_MODEL.md` H1-H5).

---

## B. Independent scientific review (kept separate from the analysis above)

Two reviewer subagents worked from the repository alone, at `a6efd0f`, in read-only mode, without this report or the
team's conclusions:

* **Reviewer A**, formal methods, hybrid systems and multi-robot coordination: `analysis/review_A_formal_hybrid.md`;
* **Reviewer B**, marine robotics and experimental methodology: `analysis/review_B_marine_experimental.md`.

Both recommend **major revision**: as it stands, neither would accept the work for T-RO, Autonomous Robots, IEEE JOE or
Field Robotics. This section keeps their findings separate from the team's assessment. B.1 says which of their claims
the team re-checked, B.2 and B.3 list where they agree and differ, and B.4 gives the team's position.

### B.1 Verification of the reviewers' claims by the analysis team

Every number marked "confirmed" was recomputed from the logs or the formal scripts with the scripts named. The new
scripts are `analysis/scripts/witness_crossings.py` (output `analysis/witness_crossings.txt`), `mcnemar_cells.py` and
`gate_idle.py`. The one-sided S1 and the 65 s M3 runs used scratch copies of the formal encodings, with no change to
`formal/`.

| Claim | Reviewer | Team check | Status |
|---|---|---|---|
| Any sensor-fault FAILSAFE entry turns a missed requirement into SAFE_DEGRADED, so LIVENESS_FAILURE is practically unreachable | A B6, B B1 | `robustness_metrics.py:334-342` | **confirmed** |
| Acceptance criterion 3 ("no release while the occupant is inside") is a safety requirement, but `safe` ignores `false_release_total`; the 7 premature-release runs are PASS | A B1, B B7 | Section D.2; classifier code | **confirmed**. Under criterion v2 as written, these 7 runs are safety failures that the classifier did not report |
| ad4fb5e (acoustic exit by range) was committed after the freeze 1dc9d3f; stage E restarted with "58 done, 566 to do" | A B8, B B3 | git log; campaign log | **confirmed** |
| The share of echoes beyond +eps_far exceeds the stated envelope under LOW as well as MEDIUM | B B9 | sensor audit in `run_metrics.json` | **confirmed**: per-run mean 0.86 % (maximum 8.4 %) under LOW-ALL and 3.6 % (maximum 15.6 %) under MEDIUM-ALL; maximum data age 0.3 s under LOW, above tau_max = 0.25 s. False-DYNAMIC: mean 2.7 % under MEDIUM-ALL |
| `at_queue` is logged both as a bool and as the string "False", so the classifier's queue test never skips | B C1 | `robustness_metrics.py:161` | **confirmed** (the same bug had inflated this report's first false-BUSY figures, since corrected) |
| With one-sided braking (`a_pair = a_brake`), the 4 s S1 BMC becomes SAT from d_warning = 2.4 m | A B4 | scratch rerun of `check_separation.bmc` | **confirmed for the configured a_brake = 0.5 m/s²**: SAT, minimum distance 0.996 m; the smallest d_warning that verifies is 2.56 m. **With the per-drone capability stated in `config.py:50` (calibrated ≥ 0.74 m/s²), the one-sided case is UNSAT at 2.4 m.** The provenance of that calibration was not re-checked **[UNDET]** |
| Sonar exit witnesses under noise often match no drone | A B2 | `witness_crossings.py` | **confirmed qualitatively; counts differ by definition.** Team: CLEAN 0 of 60 and LOW 0 of 35 sonar witnesses are more than 1 m from the occupant's range; under MEDIUM, 75 of 260 are, and 69 of 260 match no drone within 1 m. Reviewer A: 61 of 281 and 56 (LOW and MEDIUM pooled, another occupant definition). At most 7 of the 69 came while the occupant was inside (the E2 cases), so at least 62 released the latch on time but without evidence |
| In 9 MEDIUM runs a queued drone kept its latch FREE while another drone crossed the whole CR; it was protected only because it had no priority | A B3 | `witness_crossings.py` | **confirmed**: 9 crossings in 9 runs (4 B0, 2 B2_BLACKOUT, 3 B2_MEDIUM), each 3.1-3.3 s inside the CR, observer never with priority. In B0/MEDIUM-ALL/gate_single/s1 every FRONT echo of the occupant (5.4-5.6 m) was classified STRUCTURE |
| A committed drone can hold or escape inside the CR (FAILSAFE, CA, SW), dwell up to 12.5 s | A B3 | `cr_dwell.py` | **confirmed** (Section A.2: dwell maximum 12.5 s, about 20 s of interruptions in the whole campaign, never permanent) |
| B2_MEDIUM vs B2_BLACKOUT under MEDIUM-ALL: 7 vs 3 discordant, p ≈ 0.34; B2_BLACKOUT vs B0: 10 vs 3 | A B9, B B5 | `mcnemar_cells.py` (exact) | **confirmed**: 44 vs 40 PASS, 7 vs 3, p = 0.344; B2_BLACKOUT vs B0 40 vs 33 PASS, 10 vs 3, p = 0.092; the headline B2_MEDIUM vs B0 is 13 vs 2, p = 0.0074 |
| Gate runs with an idle interval ≥ 20 s (healthy drone queued, CR empty, nobody committed): B0 CLEAN 1/15, B2_CLEAN 12/15 | B B6 | `gate_idle.py` | **confirmed**: B0 CLEAN 1/15 (longest 20.3 s), B2_CLEAN 12/15 (longest 68.2 s), B2_BLACKOUT CLEAN 0/15 (longest 17.5 s) |
| Chattering is not reported: MEDIUM-ALL mean 33 events per run, maximum 180 | B C3 | `automaton.chattering_events` | **confirmed in order of magnitude**: B2_MEDIUM/MEDIUM-ALL mean 26.0, maximum 179; B2_BLACKOUT/MEDIUM-ALL 25.1 and 96; B2_CLEAN 3.0 and 12 (B2 cells only) |
| The README says "186/186" checks; the summary has 205 | A C2 | `formal_all3.out` | **confirmed**: 205 PASS lines, 0 FAIL |
| M3's 42 s horizon is shorter than the slowest admissible crossing; a rerun at 65 s is still UNSAT | A C3 | scratch rerun, `latch_bmc(steps=650)` | **confirmed** for n = 2 (l = +1.75) and n = 3 (l = +3.50 and 0.00), about 1 min each |
| The references and requirements predate C3/C5/P2/B2 (e759156, 12 Oct) | A B7, B B2 | — | not re-checked |
| MEDIUM navigation error reaches 1.66 m, beyond MAX_CORR = 1.2 m | B C4 | — | not re-checked; consistent with the 0.3-1.0 m errors at the gate in Section E.1 |
| The STRUCTURE misclassification is plausibly driven by `structure_tol_far_m` | A C8 | — | not re-checked **[UNDET]** |

### B.2 Where the two reviewers agree

1. **Classification.** SAFE_DEGRADED absorbs liveness failures, "0 LIVENESS_FAILURE" is an artefact, and the 7
   premature releases break the authors' own criterion 3.
2. **Evaluation design.** Requirements and deadlines are self-referential and come from B0 or older references. The
   CLEAN horizon is generous. The protocol changed after the freeze. The development scenarios and seeds were reused,
   so there is no held-out evaluation.
3. **Statistics.** The B2 benefit at p = 0.0074 does not survive the proper control (B2 vs B2_BLACKOUT, p = 0.34) or a
   multiplicity correction. The "paired" design is nominal because engine noise is unseeded.
4. **P2 rests on unmodelled margins.** The exit witness does not identify the occupant and is unsound under MEDIUM.
   The latch model assumes sound witnesses, perfect detection and a monotone crossing. A stalled occupant is never
   tested.
5. **P1 is proved only for an abstraction**: an idealised hard-cone sonar (assumption D), echo bounds that LOW and
   MEDIUM exceed, and a cooperative two-sided reaction. The boundary is never exercised (minimum separation 2.26 m).
6. **Terminology.** B2 is one-way acoustic communication, not "communication-free". Several documents contradict the
   evaluated code (acoustic exit "unavailable", the B2-M4g status, 186 vs 205).
7. **Missing work.** An N-drone timed model of the gate protocol (UPPAAL-style), adversarial P2 tests (occupant fault
   inside the CR), held-out scenarios and seeds, and completion-time distributions.

### B.3 Where they differ

* **Emphasis.**
  * Reviewer A concentrates on the formal side: the faithfulness of M3, the one-sided P1 case, the 93 automaton
    checks being cheap, and the missing N-drone composition.
  * Reviewer B concentrates on external validity: no single-beam sonar wider than about 25-30° exists, there is no
    crosstalk model, and the acoustic channel model is optimistic. It also stresses experimental design and the hidden
    B2 stalls.
* **P2 verdict.** A: "violated under the authors' release criterion in 7 runs". B: "partially supported". Both agree
  that double occupancy never occurred.
* **Novelty.**
  * A credits the touch-set distance bound, the occupancy-latch protocol and the provenance and mutation
    infrastructure.
  * B adds that items 1 and 2 are tied to a sensor that does not exist, and that B2 adds no sensing novelty beyond
    Rypkema et al. (2022).

### B.4 The team's position on the reviews

* **Accepted without reservation:** the classification and evaluation defects, the post-freeze deviation, the
  statistics, the documentation inconsistencies, the witness unsoundness and the missing stalled-occupant test. They are
  already in Sections D, E and G, or added there now.
* **Accepted with a nuance:**
  * *The 7 premature releases.* They are failures of criterion v2 §2 item 3 as written, and the historical tables must
    report them that way. They are not P2 violations: no double occupancy occurred, and no commit or entry happened while
    the CR was occupied. The E1-E8 metrics of Section D.5 separate the two. Adopting them is a criterion change that
    must be declared as v3, never a silent reclassification.
  * *One-sided braking.* The S1 proof as encoded needs both drones to brake at a_brake = 0.5. The same model with one
    drone braking at the calibrated per-drone capability (≥ 0.74 m/s², `config.py:50`) still verifies at
    d_warning = 2.4 m. The fix is therefore to state and prove the one-sided case with a documented capability, or to
    restrict the claim to mutual detection. No threshold changes.
  * *"The 93 checks are largely true by construction."* They are cheap. They still check the deployed guard code, and
    the mutations show that they catch encoding errors. The substantive gap the reviewer identifies is real: the N-drone
    timed composition and the timers living in perception code are not verified.
* **Out of scope of this consolidation, reported as limitations [LIMIT]:**
  * the idealised wide-cone sonar and the absence of crosstalk;
  * the optimistic acoustic channel model;
  * external baselines (reservation or token protocols, CBF avoidance).

  A P1 sensitivity study to assumption (D) and a re-run of the acoustic residual-risk analysis are proposed as separate,
  separately reported experiments (Section H). They are not changes to the frozen noise profiles.
* **Added to the plan because of the reviews:**
  * held-out seeds and a held-out gate scenario (H.3, H.4);
  * the B2 vs B2_BLACKOUT ablation as the only test of the acoustic benefit (H.4);
  * M3 with spurious-witness, missed-detection and in-CR-stall nondeterminism (I);
  * the one-sided S1 variant (I);
  * chattering and idle-gate reporting (G.1).

---

## C. Literature review (summary)

The full review is `analysis/litreview/LITERATURE_REVIEW.md`, with its search scripts, the raw API responses and
`included.bib`.

**Method.** The review is structured and PRISMA-inspired:

* 11 topics (T1-T11, plus a cross-cutting topic on estimation under intermittent observations), 46 queries;
* sources: OpenAlex, Crossref and Semantic Scholar, plus 10 web searches, 7 Crossref bibliographic lookups and a
  known-item list of 99 DOIs;
* 867 unique database records, 119 assessed at eligibility (Crossref record and abstract), 62 included;
* every included DOI checked against its Crossref record and resolving at doi.org. Four DOIs first typed from memory
  failed this check and were replaced by verified ones.

**Limitations** (Section 1.6 of the review):

* a single screener;
* Semantic Scholar was rate-limited for 44 of the 46 queries, and IEEE Xplore, Scopus and Web of Science were not
  queried;
* 36 of the 62 works entered through known-item searches, which favours well-known work;
* content statements rest on abstracts, not full texts;
* the novelty statements below are therefore provisional.

**Overlap with `references.bib`.** Only 7 of the 62 included works are already cited, all in acoustic navigation and
simulation. The project bibliography has no entries for hybrid verification, mutual exclusion, deadlock, formation
recovery or failure detection.

**What the literature already covers:**

* one-way-travel-time and inverted-USBL relative navigation (Eustice et al. 2011; Rypkema et al. 2017, 2019, 2022);
* heartbeat and accrual failure detection (Chandra and Toueg 1996; Hayashibara et al. 2004);
* critical sections with a stop-before-entry rule and precedence (Pecora et al. 2018; Mannucci et al. 2019; a central
  coordinator in both);
* priority-layered and Simplex architectures (Brooks 1986; Sha 2001);
* barrier-function and decentralised doorway passage with deadlock treatment (Grover et al. 2023; Zheng and Li 2024;
  Chandra et al. 2025);
* coordinated path following with synchronised progress (Ghabcheloo et al. 2009, with communicated coordination states).

**Closest work.** Rypkema, Schmidt and Fischell (2022, doi:10.55417/fr.2022026) use the same synchronous-clock range
and angle sensing. There, a single reference beacon broadcasts and also selects the behaviour; there are no critical
regions and the vehicles do not ping each other. This agrees with reviewer B: B2's *sensing* is not new.

**What appears new relative to the screened records** (to be re-checked on IEEE Xplore and Scopus before any manuscript
claim):

1. payload-free identity pings from every vehicle, used together for relative localisation, presence detection and
   gate-occupancy evidence;
2. mutual exclusion of a physical region with no request, grant or release message;
3. formation degradation and recovery driven by perceived absence.

The campaign shows that the benefit of item 1 for the gate is not yet demonstrated (Sections D.4, B.1).

**Mapping of the diagnosed defects to published techniques.** The labels are the review's own:

* **WELL-SUPPORTED:** demonstrated in a comparable setting;
* **SPECULATIVE:** a plausible transfer not demonstrated here.

| Defect (section) | Technique | Label | Plan step | How it is used here |
|---|---|---|---|---|
| Witness accepts clutter; latch misses crossings (D.2, B.1) | m-of-n track confirmation (Blackman 2004; Musicki et al. 1994) | WELL-SUPPORTED | G.3 (b) | at least 3 captures with consistent range evolution before an echo can be an exit witness |
| Occupancy belief fuses sonar and acoustics ad hoc (D.1, D.4) | Bernoulli / IPDA existence filter for "gate occupied" (Ristic et al. 2013); SPRT for "gate clear" (Wald 1945) | WELL-SUPPORTED as estimators; SPECULATIVE for correlated sonar returns | not in the minimum plan | a candidate replacement for the latch once G.3 is evaluated; it needs a detection and clutter model per profile, so it is deferred rather than adopted now |
| Exit by absence of evidence; B2 releases late (D.4) | identity-keyed positive exit evidence (ping track crossing the exit plane) | SPECULATIVE | G.3 (c), (d) | already partly implemented (A1-A3); the plan tightens the veto and releases on positive exclusion of every plan drone |
| No occupancy check close to the CR; stalled occupant (D.2, D.3) | stop-before-entry invariant with a commit point (Pecora et al. 2018; Mannucci et al. 2019) | WELL-SUPPORTED for the invariant (with a coordinator); SPECULATIVE for the local version | G.3 (a) | stage 2 at the existing hold point s_hold, evaluated locally |
| Unbounded relocalization hold, bar traps (D.4, E.1) | bounded waits and leases (Gray and Cheriton 1989; Čáp et al. 2016); waiting zones outside the corridors (Draganjac et al. 2016) | SPECULATIVE (the sources use messages) | G.4 (c), (d) | watchdog, bounded retry and an explicit abort before the CR entry plane |
| Reactive layers stall near the gate (E.1) | deadlock-aware perturbation of the avoidance layer (Grover et al. 2023) | WELL-SUPPORTED for ground robots; SPECULATIVE underwater | not adopted | the G.4 abort handles the observed cases without touching the P1 layers; a perturbation would reopen S1-S3 |
| Private clocks diverge (E.2) | coordination-state synchronisation from sensed progress (Ghabcheloo et al. 2009) | SPECULATIVE (sensed substitute not demonstrated) | G.5 (b) | bounded rate correction from the perceived along-track offsets to identified neighbours |
| False missing detections with fixed timeouts (E.2, E.3) | accrual failure detection on the known TDMA schedule (Hayashibara et al. 2004); hysteresis for re-inclusion | WELL-SUPPORTED in distributed systems; SPECULATIVE underwater | G.5 (a), G.6 | "missing" requires the absence of fresh identified fixes near the slot; re-inclusion only from an identified fix |
| N-drone gate protocol never verified (I; both reviewers) | timed-automata model checking (Alur et al. 1995; UPPAAL-style); verified multi-agent conflict resolution (Tomlin et al. 1998; Loos et al. 2011) | WELL-SUPPORTED as methodology | I | an N = 2-3 timed model of queue, latch, stage 2 and abort with bounded missed detections, spurious witnesses and in-CR stalls |

**Not adopted, with reasons:**

* **Pulse-coupled clock alignment** (SPECULATIVE): it can interfere with the TDMA slot schedule.
* **MHT association of anonymous echoes with acoustic tracks:** high complexity, and no demonstration for wide-cone
  single-beam sonars.
* **CBF or velocity-obstacle avoidance:** it would replace the verified P1 layer, outside the brief.

---

## D. Gate analysis

### D.1 What the code does (verified)

* **Witness first, acoustics second.** The sonar exit witness is evaluated first (`gate_perception.py:186-188`). The
  acoustic exit is evaluated only if there is no sonar witness (`:189-190`). `heard_inside` is consulted only for the
  t_occ_max fallback (`:196-197`). A sonar witness therefore moves the latch BUSY → EXITING without any acoustic check,
  even when a fresh accepted fix places the occupant inside the CR. The observation in the request is **confirmed**.
* **EXITING still blocks a commit.** `occ_busy = occ_state != "FREE"` (`:207`). EXITING becomes FREE only after
  t_clear = 1 s with no corridor echo (`:200-201`).
* **The witness does not identify the occupant.** It is any persistent obstacle echo in any sector that satisfies
  W1-W3 (`exit_witness.py`), i.e. not explainable by a hull in the CR, ahead of the observer, and reachable from the CR
  at v_bound. "Persistent" means DYNAMIC (2 of 3 captures), or seen at a similar range in the previous update
  (`gate_perception.py:149-150`).
* **A committed drone continues without `occ_busy == False`** (`spec.py:217-224`). This is **confirmed**. Commit
  happens at the queue point, at s ≈ -4.9 to -5.6 m, so between commit and CR entry the drone travels about 3.8-4.8 m
  with no occupancy re-check. Only the separation layer and the relocalization hold can stop it.

### D.2 The seven premature releases, case by case

These are releases (BUSY → EXITING) by a queued drone while another drone was truly inside the CR. The sources are
`analysis/scripts/gate_forensics.py`, `gate_metrics.py` and `echo_track.py`. The full timelines are in
`analysis/forensics_v.txt`.

Column definitions:

* **Releaser at the time** is the gate-frame position (s, l) in metres. All releasers were at their queue point.
* **"limit"** is the CR far corner plus eps_range_far, as seen from the releaser's FRONT sonar. An echo beyond it
  satisfies W1.
* **"ctr"** is the true range to the occupant's centre.
* **Occupant leaves CR** is the time after the release.
* **Latch FREE** is when the releaser's latch reached FREE.
* **Next commit** is the drone that committed next and when (any drone).
* **Committer enters CR** is when that drone truly entered the CR.
* **Min distance** is the closest committer-occupant distance between commit and entry.
* **B2 acoustic at last BUSY step** uses the logged `gate.acoustic` diagnostics and `form.acoustic_fixed` (accepted fix
  younger than 2.5 s). "A2 not met" names the first possible occupant that A2 could not place outside.

| # | Run | Occupant (mode, true s) | Releaser at the time | Witness echo | Occupant leaves CR | Latch FREE | Next commit | Committer enters CR | Min distance | B2 acoustic at last BUSY step |
|---|---|---|---|---|---|---|---|---|---|---|
| 1 | B0 / MEDIUM-ALL / lost_drone_mutex / s0 | d1 MUTEX_PASS, +0.71 | d2 at queue (-5.65, -3.84) | FRONT 8.07 m (limit 8.07; ctr 7.20) | +0.2 s | 43.2 s (back to BUSY first) | d2 at 43.2 s | 67.5 s | 6.84 m | n/a (B0) |
| 2 | B0 / MEDIUM-ALL / mutex_deadlock_resolution / s1 | d0 MUTEX_PASS, +0.62 | d1 at queue (-4.93, +2.46) | FRONT 7.67 m (limit 7.19; ctr 6.03) | +0.4 s | 16.6 s | d1 at 16.6 s | 27.6 s | 6.65 m | n/a (B0) |
| 3 | B2_MEDIUM / MEDIUM-ALL / line_parallel_mutex / s0 | d1 MUTEX_PASS, +0.43 | d2 at queue (-5.39, -3.63) | FRONT 10.57 m (limit 7.73; ctr 6.60) | +0.8 s | 40.6 s | d2 at 40.6 s | 53.9 s | 5.91 m | fresh fixes of d0 and d1; A1 possible {0, 1}; A2 not met (d0) |
| 4 | B2_MEDIUM / MEDIUM-ONE / mutex_deadlock_resolution / s2 | d0 MUTEX_PASS, +0.27 | d1 at queue, in COLLISION_AVOIDANCE (-4.98, +2.48) | FRONT 8.88 m (limit 7.24; ctr 5.85) | +1.1 s | 15.9 s (same 0.1 s sample as the exit) | d1 at 16.0 s | 28.0 s | 6.58 m | fresh fixes of d0 and d2; possible {0, 2}; A2 not met (d0 = occupant) |
| 5 | B2_MEDIUM / MEDIUM-SUBSET / gate_single / s1 | d2 MUTEX_PASS, +0.60 | d1 at queue (-5.51, -3.45) | FRONT 8.07 m (limit 7.72; ctr 6.92) | +0.4 s | 72.1 s | d1 at 72.1 s | 84.5 s | 7.48 m | fresh fixes of d0 and d2; possible {0, 2}; A2 not met (d0) |
| 6 | B2_MEDIUM / MEDIUM-SUBSET / gate_single / s2 | d0 MUTEX_PASS, +0.54 | d1 at queue, in SEPARATION_WARNING (-5.43, -3.34) | FRONT 8.93 m (limit 7.59; ctr 6.81) | +0.6 s | 25.8 s | d2 at 47.4 s | 57.7 s | 7.87 m | fresh fixes of d0 and d2; possible {0}, d2 excluded; A2 not met (d0 = occupant) |
| 7 | B2_MEDIUM / MEDIUM-SUBSET / integrated_short / s1 | d0 MUTEX_PASS, +0.50 | d1 at queue (-5.53, -3.76) | FRONT 8.17 m (limit 7.92; ctr 6.96) | +0.7 s | 33.3 s | d2 at 35.1 s | 47.0 s | 7.17 m | fresh fixes of d0 and d2; possible {0, 2}; A2 not met (d0 = occupant) |

**What caused the releases.** In all seven cases the witness is an isolated FRONT echo. It appears as UNCONFIRMED and
becomes DYNAMIC on the next capture at the same range (±0.1 m), then vanishes within one or two captures. It lies
0.9-4.0 m beyond the occupant's centre range (`echo_track.py`).

No other drone was at that range inside the cone. The other drones were either at least 11 m away or about 90° off
axis. During the masked crossing the occupant itself returned only STRUCTURE-classified echoes.

The W2 reach condition admits such an echo only once v_bound·(t - t_last_corridor) is large enough. This explains why
the releases cluster in the second half of the crossing (occupant at s = +0.27 to +0.71).

All seven releases occurred under MEDIUM sonar noise: 7 of 260 sonar releases in MEDIUM profiles (2.7 %), against 0 of
95 in CLEAN and LOW. This is consistent with the false-DYNAMIC fraction that the sensor audit records under MEDIUM: a
per-run mean of 2.7 % of obstacle echoes in MEDIUM-ALL (`run_metrics.json`, `sensor.sonar.false_dynamic`).

The seven cases are the visible part of a wider problem (`witness_crossings.py`):

* Under MEDIUM, 69 of the 260 sonar witnesses have a range that matches no drone within 1 m, and 75 differ from the
  occupant's range by more than 1 m. No CLEAN (60) or LOW (35) witness does. At least 62 of these evidence-free
  witnesses released the latch after the true exit, so they were harmless only because of timing.
* The latch also misses whole crossings: in 9 MEDIUM runs (4 B0, 2 B2_BLACKOUT, 3 B2_MEDIUM), a queued drone kept its
  belief FREE for the entire 3.1-3.3 s crossing of the first passer. In the run inspected, every FRONT echo of the
  occupant was classified STRUCTURE. The observer never had priority at those times, so precedence, not the latch,
  prevented a commit. In B2_MEDIUM, a fresh fix inside the CR does not set the latch BUSY: `heard_inside` only blocks
  the timeout.

* **[WRONG]** The witness design assumes that every persistent unexplained echo beyond the CR is a hull. The formal
  latch model (M3) uses the same idealisation. MEDIUM clutter violates it.
* **[UNDET]** The physical origin of the echoes (noise false alarm, intermittent structure leaf, or something else)
  cannot be settled from the logs, which keep at most 4 echoes per sector with no intensity.

**The acoustic information at the release instant (B2, cases 3-7):**

* every occupant had an accepted fix younger than 2.5 s;
* in no case did the acoustic exit succeed (A2 failed every time);
* in 3 cases A2 failed for the occupant itself;
* in 2 cases it failed for a drone that had passed much earlier and was waiting about 6 m beyond the gate (the known
  range-margin limitation).

`heard_inside` is not logged. Given an accepted fix younger than 2.5 s of a drone truly at s = +0.27 to +0.65, it would
almost certainly have been true, but this cannot be confirmed **[UNDET]**.

The decisive point: the acoustic layer contradicted the exit in every B2 case, and its evidence could not be used.

**What happened next.** In none of the seven cases:

* did the next commit occur while the CR was occupied;
* did a separation warning or collision avoidance involve the committer and the occupant (minimum distance 5.9-7.9 m);
* did a drone approach the occupied region.

The occupants were healthy and moving at about 0.5 m/s, and left the CR 0.2-1.1 s after the release. The latch needed a
further 1 s (t_clear). The commit came 0.1-22 s after the true exit. The committer then needed 11-24 s to reach the CR,
by which time the occupant was 5.6-7.3 m beyond the gate.

**Why there was no simultaneous occupancy.** There were three layered margins, none of which is the formal argument:

1. the occupant was healthy and already near the exit face;
2. t_clear = 1 s;
3. the queue-to-CR travel distance, about 3.8-4.8 m at v_pass = 0.5 m/s, i.e. at least 7.6 s.

The case-4 latch reached FREE in the same 0.1 s sample in which the occupant crossed the exit face, which shows how thin
the belief-level margin is.

**Counterfactual (not observed).** If the occupant had stopped inside the CR, the same false witness would have released
the latch about 1 s later. The follower would then have committed and travelled 4 m towards a drone masked by the
structure. The only remaining barrier would be the separation layer, which uses the same structure-masked echoes. This is
the M3f scope, reached through the witness rather than the timeout. The campaign never tests a stalled occupant: in
`lost_drone_mutex` the fault hits drone_0 at t = 1 s, at s ≈ -14 m. **[WRONG] missing experiment.**

### D.3 Correct design: a two-stage gate protocol (evaluation)

Assessment of the proposed direction: approach authorisation, then an entry check before the CR.

* **Stage 1, approach authorisation, as today.** Commit at the queue point requires `has_prio ∧ ¬occ_busy`. Precedence
  (M1/M2) is unchanged.
* **Stage 2, entry admission, at the existing hold point s_hold = -1.18 m.** The flow already stops a committed drone
  there for relocalization (`flows.py:249-256`), so stage 2 generalises an existing mechanism rather than adding a new
  one. Entry is admitted only if all of the following hold:
  1. `loc_ok` (as today);
  2. **close-range sonar check:** no unexplained echo compatible with a hull inside the CR during the last N ≥ 3
     captures, from the hold point. This view is much better than the queue's: the CR is 0.4-2 m ahead on the axis, and
     the near bars at 68° are outside the 60° FRONT cone. Whether an occupant anywhere in the CR is unmasked from the
     hold point must be proved as a new M4-type obligation (M4h below);
  3. **B2 veto only:** no fresh accepted fix places another drone possibly inside the CR. There is no positive acoustic
     requirement, so a silent beacon never blocks entry and the absence of fixes means nothing;
  4. the occupancy latch is FREE.

  If any condition fails, the drone holds outside, observes and re-checks. Once inside, it completes the passage: no
  occupancy-based stop is added inside the CR, and only FAILSAFE, CA and SW keep their priority.
* **What it would have done in the seven cases.** It would not have changed their outcome, which was already safe. It
  removes the dependence on kinematic margins: a premature release can no longer lead to entry while the CR is visibly
  occupied from close range. In B2, the veto (condition 3) applied to the sonar witness would have blocked cases 3-7
  directly.
* **The veto must first be tightened for liveness** (Section D.4). Otherwise it adds waiting.
* **Formal consequence.** With M4h, the stalled-occupant case that M3f leaves open becomes a provable safety case:
  * an occupant observable from the hold point blocks entry, healthy or not;
  * the timeout and witness fallbacks then affect only liveness, never P2;
  * P2 no longer needs the healthy-crossing assumption.
* **Alternatives considered:**
  * re-checking `occ_busy` continuously during the approach: the queue-view latch is the weak signal, so this repeats
    the same error;
  * mandatory acoustic confirmation: rejected, because it deadlocks under outage;
  * longer t_clear or t_occ_max: slower and not evidence;
  * moving the commit point closer to the CR: it changes M1/M2/M4 geometry and the queue spacing.

### D.4 Gate liveness defects (healthy drones blocked outside the CR)

* **[WRONG] Unbounded relocalization hold and bar traps (8 runs, Section E).**
  * The committed drone holds at s_hold, or ends up 1-3.5 m before the CR in SW/CA cycles caused by an "unexplained"
    bar echo. This lasts 100-260 s, until the deadline.
  * Navigation errors at the gate were 0.3-1.0 m (lateral, and in one case 1.0 m along-track).
  * The map fit was either unmatched (0 sectors at the hold point, MEDIUM-ONE gate_single s0), ambiguous (u up to
    2.2 m), placed beside the opening, or of magnitude about MAX_CORR (1.05-1.2 m).
  * `flows.py` has no timeout, no escalation and no retry beyond the 6-s back-off. The committed drone blocks every
    queued drone, which see it in FRONT as a WAIT relation.
  * This is a deadlock classified as SAFE_DEGRADED.
* **[WRONG] Phantom corridor echo (MEDIUM-SUBSET integrated_short s2).** A recurring unexplained FRONT echo at about
  5 m keeps the latch BUSY for 204 s. The longest gap between corridor echoes is 24.1 s, below t_occ_max = 25 s, so the
  timeout never fires (`latch_age.py`). Meanwhile A1 positively excludes both other drones from the CR. That evidence
  says no fleet drone is inside, which is what P2 is about, but the design forwards it to "the sonar rules"
  (`acoustic_gate.py:186-187`).
* **[WRONG] `heard_inside` false positives (LOW mutex_deadlock_resolution s2).** The latch stayed BUSY for 61.5 s with
  no corridor echo, at least 36 s beyond t_occ_max. Both other drones had passed and were waiting about 6 m beyond the
  gate, on or near the line of sight through the CR. Three margins combine: the one-sided 3 m multipath excess, growth
  by v_bound·age with fresh_s = 2.5 s (up to 1.75 m), and nav_err_gate of 1 m plus R_OUT. Together they make their
  regions intersect the grown CR. The two passed drones were in turn waiting for this drone: a circular wait, broken only
  when they moved on.
* **[INEFF/WRONG] B2 slows the gate.**
  * **False-BUSY time** is the share of the time a queued drone believes the CR occupied while it is empty:
    0.45-0.68 of the queue time in every cell (`gate_events.py`, corrected for `at_queue` logged as a string). The
    share is 0.675 in B2_CLEAN and 0.513 in BLACKOUT-CLEAN.
  * **Release delay after the true exit** (`release_delay.py`):

    | Cell | Release kind | Median | p90 |
    |---|---|---|---|
    | B2_CLEAN | acoustic exit | 52.4 s | 70.4 s |
    | BLACKOUT-CLEAN | timeout | 26.1 s | 44.9 s |
    | sonar witness, any cell | sonar | about 2 s | 5-10 s |

  * **Queue wait, p90:** 88.0 s in B2_CLEAN, against 55.5 s in BLACKOUT-CLEAN.
* **[LIMIT] An immobile drone inside the CR.** A drone stopped inside the CR by a permanent propulsion failure blocks
  the gate forever under every design that keeps P2. The system must detect and report this explicitly, not wait
  silently:
  * B2 detects it while the drone keeps pinging;
  * with a silent beacon, stage 2 detects it from the hold point.

  A healthy drone is never forced through.

### D.5 Corrected gate metrics (proposed; P2 itself unchanged)

P2 stays `G(Σ_i inside_CR_i ≤ 1)` on ground truth. The new metrics separate perception errors, protocol errors, unsafe
actions and violations. The values below are recomputed offline on the 624 runs where available.

| Event | Definition (truth vs onboard) | Kind | Current campaign |
|---|---|---|---|
| E1 incorrect occupancy belief | queued drone: belief FREE while another drone is truly in the CR (false-FREE), or belief BUSY/EXITING while the CR is empty (false-BUSY), as a fraction of the queue time | perception | false-FREE ≤ 0.007 (MEDIUM), including 9 whole crossings missed by a queued observer; false-BUSY 0.45-0.68 |
| E2 premature release | BUSY → EXITING/FREE while another drone is truly in the CR | perception / protocol | 7 (all in MEDIUM), 5 of them in B2 |
| E3 approach authorisation while occupied | commit while another drone is truly in the CR | protocol | 0 / 536 |
| E4 entry authorisation while occupied | stage-2 admission while another drone is truly in the CR (new; today: CR entry while occupied) | protocol (unsafe action) | 0 |
| E5 simultaneous occupancy | P2 violation | safety violation | 0 |
| E6 collision | contact | safety violation | 0 |
| E7 detected and cancelled unsafe approach | stage-2 hold triggered while the CR is truly occupied (new) | protocol success | not measurable yet |
| E8 waiting and eventual passage | queue wait, hold time, passage completion of healthy drones, stall (no progress for T_stall) | liveness | wait p90 43-89 s; 88-100 % of healthy drones pass per cell; idle gate ≥ 20 s in 12/15 B2_CLEAN gate runs (0/15 BLACKOUT-CLEAN) |

Two additional reports are proposed: the entry gap (time between the occupant's exit and the next entry; current
minimum 11.5 s) and the dwell time inside the CR.

---

## E. Formation and mission-completion analysis (all 28 non-PASS B2 runs)

Sources: `analysis/scripts/p3_failures.py` (full output in `analysis/p3_failures.txt`), `gate_stuck.py`,
`latch_age.py` and `tau_check.py`.

Column definitions:

* **Final err** is the referee's true formation error at the end, in metres.
* **Trend** compares the mean of the first and last 10 s of the final 30 s.
* **Category numbers** follow the request's list: 1 time, 3 acoustic, 5 false missing, 6 false re-occupation,
  7 vacancy decisions, 8 navigation drift, 10 control authority, 11 gate, 12 evaluation, 13 other.

### E.1 Gate-related primary cause (11 runs)

| Run | Failed | Primary cause → secondary effects | Final err | Trend | More time? | 1.0 m? |
|---|---|---|---|---|---|---|
| BLACKOUT / MEDIUM-ALL / lost_drone_mutex s1 | progress, gate | 8 → 11: d2 has lateral nav error (true l ≈ -1.2 at s ≈ -3.2) and its relocalization is not accepted (12.5 s hold); the unexplained bar echo causes SW/CA cycles (SW 32 s, CA 13 s) and it never enters the CR → vacancy declared by d1 | 16.3 | rising | no (stall) | no |
| MEDIUM / MEDIUM-ALL / gate_single s0 | progress, gate, P3 | 8 → 11: d1 is beside the opening (true l ≈ -2.9, estimated -2.2, along-track error 0.45); a LEFT/FRONT unexplained bar echo at about 2 m keeps it in SW (98 s) and CA (9 s) for 210 s; the fit matches 2 sectors but is not applied beside the opening | 19.3 | flat | no | no |
| MEDIUM / MEDIUM-ALL / lost_drone_mutex s1 | progress, gate | 8 → 11: d2 relocalization hold 64 s, then CA 27 s; true l ≈ -0.8 | 15.8 | rising | no | no |
| MEDIUM / MEDIUM-ONE / gate_single s0 | progress, gate, P3 | 13 → 11: relocalization unobservable at the hold point (0 matched sectors, cost 3.24); the first drone holds 250 s at s ≈ -1.2 to -1.9 with only about 0.3 m true lateral error; the others YIELD for 260 s: the whole gate is deadlocked | 4.3 | flat | no | no |
| MEDIUM / MEDIUM-SUBSET / gate_single s0 | progress, gate, P3 | 8 → 11: d1 true l ≈ -1.0 against estimated -0.4/-0.5; FRONT unexplained echo at about 2.2 m; SW 78 s, hold 79 s | 19.3 | flat | no | no |
| MEDIUM / MEDIUM-SUBSET / integrated_short s0 | progress, gate, P3, waiting | 8 → 11: d1 true l ≈ -1.0/-1.2 against estimated -0.5/-0.9; ambiguous fit (u 1.5-2.2 m); SW cycles at s ≈ -3 for 180 s | 20.0 | flat | no | no |
| MEDIUM / MEDIUM-SUBSET / integrated_short s2 | progress, gate, P3 | 13 → 11: a phantom FRONT corridor echo at about 5 m keeps the latch BUSY for 204 s (longest gap 24.1 s < 25 s); A1 had excluded both other drones; d1 YIELDs for the rest of the run | 21.7 | flat | no | no |
| MEDIUM / MEDIUM-SUBSET / lost_drone_mutex s0 | progress, gate | 8 → 11: d2 holds 50.7 s, then SW/CA on a FRONT UNKNOWN echo at 0.7-2.3 m; true l -0.6/-0.9 against estimated -0.1/-0.5 | 15.9 | rising | no | no |
| MEDIUM / MEDIUM-SUBSET / mutex_deadlock_resolution s0 | progress, gate, P3 | 8 → 11: d2 reaches the CR entrance; FAILSAFE/CA on a FRONT echo at 0.19 m (bar) and backs off; its along-track error grows to about 1.0 m (estimated s -2.06, true -3.05); the fit offset of about 1.05-1.2 m (≈ MAX_CORR) is not applied; SW cycles at s ≈ -3 | 12.3 | rising | no | no |
| MEDIUM / MEDIUM-ONE / integrated_short s1 | no permanent false vacancy | 8 → 11 → 7 → 12: d1 holds 37 s (passes at 133 s), the others declare it vacant, one does not re-include it before the run ends at **SUCCESS** (186.6 s); the classifier then fails a requirement that the terminal monitor does not wait for | 0.35 | falling | possibly | n/a |
| LOW / LOW-ALL / mutex_deadlock_resolution s2 | P3 | 3 → 11 → 1: a false `heard_inside` holds the latch for at least 36 s beyond t_occ_max; d2 passes at 115.9 s (others at 21.5 and 42.1 s); too little time is left to rejoin | 2.70 | falling fast (6.95 → 3.36) | **yes, likely** | no |

No drone in this group stopped inside the CR. All were waiting or stalled outside: committed at s between -1.2 and
-3.7, or at the queue.

### E.2 Formation clocks: false missing neighbours leading to private clock divergence (11 runs)

**The mechanism** (`control/formation_time.py` docstring and code):

1. while a neighbour is perceived missing, the drone lowers the rate of its own formation time (reduced progress);
2. `tau` is kept afterwards, and nothing re-synchronises it across drones;
3. the only correction is the capped sonar progress consensus (sigma: progress_max 2 m, v_corr_max 0.1 m/s);
4. when the missing detections are false and asymmetric (different drones, different times, recurring), the `tau_k`
   diverge.

**The evidence.** At the end, each drone's along-track error matches `-v_plan * (tau_k - mean tau)` in sign and
magnitude, partly absorbed by sigma (`tau_check.py`). Examples:

* MEDIUM-ALL formation_six s1: d3 has tau = 22.7 s against 5.4-11.4 s for the others; the predicted lag is 3.65 m and
  the measured along-track error -3.34 m;
* LOW formation_six s0: d2 has tau = 10.8 s against 0-5.6 s; predicted 2.25 m, measured -1.9 m.

Each drone's own slot error stays at 0.02-0.2 m, so the error is invisible on board.

| Run | Failed | Primary → secondary | Final err | Trend | More time? | 1.0 m? |
|---|---|---|---|---|---|---|
| BLACKOUT / MEDIUM-ALL / formation_recovery_head_current s1 | P3, waiting | 5 (no acoustic) → tau divergence → d2 3.5 m off | 3.51 | rising | no | no |
| BLACKOUT / MEDIUM-ALL / formation_six s2 | P3, false vacancy | 5 → tau 3.6-15.8 s → along-track errors -1.23 to +1.81 m; 7 (4 false vacancies in the last 12 s) | 1.86 | rising | no | no |
| BLACKOUT / MEDIUM-ALL / lost_drone_rejoin s1 | P3, waiting | the temporarily faulted d2 accumulates tau = 19.7 s (others 4-6 s) and stays 2.3 m behind | 2.54 | rising | no | no |
| LOW / LOW-ALL / formation_six s0 | P3, waiting | 5 → d2 tau = 10.8 s → -1.9 m | 1.90 | rising | no | no |
| MEDIUM / MEDIUM-ALL / formation_six s1 | P3, waiting | 5 → d3 tau = 22.7 s → -3.34 m | 3.43 | rising | no | no |
| MEDIUM / MEDIUM-ONE / formation_gust s2 | P3, waiting | 5 → d2 tau = 27.3 s → -1.88 m; 7 (vacancy and re-occupation) | 1.99 | rising | no | no |
| MEDIUM / MEDIUM-ONE / formation_square s0 | P3, waiting | 5: d0 believes d3 missing while d3 is 0.6 m from its slot; tau differences mostly absorbed | **0.60** | falling | yes (P3) | **P3 yes, run still fails** ("still waiting") |
| MEDIUM / MEDIUM-ONE / formation_square s2 | P3, waiting | 5 → tau lags of about ±1.2 m; 3 drones in WAIT | 1.33 | flat | no | no |
| MEDIUM / MEDIUM-SUBSET / formation_six s2 | P3, waiting | 5 → d2 tau = 16.3 s → -2.07 m | 2.07 | rising | no | no |
| MEDIUM / MEDIUM-SUBSET / formation_square s0 | P3, waiting | 5: d0 believes d3 missing | **0.80** | falling | yes (P3) | **P3 yes, run still fails** ("still waiting") |
| MEDIUM / MEDIUM-ALL / p1_two_lines s2 | progress, waiting | 5 → reduced progress delays the mission end beyond the 3 m progress tolerance | n/a | n/a | unknown | n/a |

### E.3 P3-deg and recovery

| Run | Failed | Primary → secondary | Final err | More time? | 1.0 m? |
|---|---|---|---|---|---|
| BLACKOUT / MEDIUM-ALL / lost_drone_timeout s0, s1, s2 | P3-deg re-formed, vacancy, waiting | 7: only some survivors declare the slot vacant (s0: d0 and d3, then d0 re-occupies at 111 s; s1: none; s2: only d0); 6; secondary tau divergence among the survivors | about 20 (lost drone included) | no (flapping or never) | no |
| MEDIUM / MEDIUM-ONE / lost_drone_timeout s2 | same | 6/7: vacancy declared by d0 (49 s) and d3 (56 s); d0 re-occupies twice (82 s, 129 s) although the channel is active | 20.4 | no | no |
| MEDIUM / MEDIUM-SUBSET / lost_drone_timeout s2 | same | 6/7: vacancy declared by d0, d3 and d1; d1 re-occupies at 93 s | 19.8 | no | no |
| BLACKOUT / MEDIUM-ALL / p1_vertical_escape s1 | P3 | 10: d1 is displaced 4.7 m behind; its recovery is blocked by repeated separation warnings (186 of the last 600 steps); two drones declare it vacant (7) | 4.72 | slowly falling: possibly, minutes | no |

### E.4 Answers

**Root causes across the 28 runs:**

| Primary cause | Runs | Of which |
|---|---|---|
| Gate liveness | 11 | 8 relocalization or bar trap, 1 phantom corridor echo, 1 acoustic false "inside", 1 relocalization delay ending in a false vacancy at SUCCESS |
| Private clock divergence after false missing detections | 11 | |
| Inconsistent vacancy and re-occupation | 5 | |
| Recovery blocked by separation | 1 | |

**Would more time help?** Plausibly in 3 runs:

* LOW mutex_deadlock_resolution s2, which was converging fast;
* MEDIUM-ONE integrated_short s1, stopped at SUCCESS;
* BLACKOUT p1_vertical_escape s1, which was slowly converging.

The other 25 were stalled, flat or diverging at the deadline. Extending deadlines would hide deadlocks rather than
resolve them.

**Would a 1.0 m tolerance help?** It legitimately changes P3 in 2 runs (final errors 0.60 m and 0.80 m). Both still
fail "still waiting for a missing neighbour", which is a perception false missing. No B2 classification would change
with the threshold alone.

**Where the tolerance belongs: the referee and acceptance criterion, not the controller.**

* **Reasons:**
  * in every failing formation run the on-board slot error was 0.02-0.2 m: the drones already believe they are within
    0.5 m, and the large true errors come from clock inconsistency and navigation, not from the switching thresholds;
  * the formal balls of F3 (0.44 m along-track, 0.36 m lateral) lie inside the on-board e_ok = 0.5 m. Keeping the
    controller tighter than the mission tolerance is a margin for estimation error;
  * raising the on-board e_ok to 1.0 m with e_lost = 1.2 m would leave 0.2 m of hysteresis against on-board residual
    noise, so chattering between RECOVERY and FOLLOW could appear (`chattering_events` already counts it).
* **Proposed referee P3 criterion (v3, to be frozen before the next campaign):**
  1. **Loss episode:** a loss episode starts when e_ref > 1.5 m. This gives 0.5 m of hysteresis above the 1.0 m
     tolerance.
  2. **Recovery:** e_ref ≤ 1.0 m for t_hold = 5 s.
  3. **Terminal requirement:** at the end, e_ref ≤ 1.0 m held for t_hold. This removes the vacuity of the current
     definition, under which an error that stays between 0.5 and 1.2 m forever never opens an episode and passes.
  4. **P3-deg:** the same thresholds apply to the present formation.
  5. **Reporting:** along-track and lateral components per drone are reported.
* **Unchanged:** the slots, the target geometry, P1 and every separation threshold.
* **Disclosure:** this criterion change was decided after seeing results. It must therefore be:
  * frozen as Acceptance Criteria v3 before the next campaign;
  * applied to new runs;
  * reported side by side with v2 on the historical runs.

---

## F. Property-evidence matrix

| | Formally proven under assumptions | Empirically verified | Outside the verified envelope | Incomplete or failed | Unknown |
|---|---|---|---|---|---|
| **P1** separation | S1b k-induction, S1a BMC 4 s, S2a-d exhaustive, S3, S4 (z3), under the envelope: echo error within [-eps_near, +eps_far], tau_max 0.25 s, speed and current bounds, **both drones of a pair detecting each other and braking** (a_pair = 2 a_brake), and assumption (D), a hard-edged cone with full detection. One-sided braking: SAT at the configured a_brake = 0.5, UNSAT at the calibrated 0.74 (Section B.1). S0 is **randomized**, about 35 000 scenes: evidence, not proof | 0 violations and 0 contacts in 624 runs; minimum true separation 2.26 m (intruder 1.95 m): the boundary is never exercised | the echo envelope is exceeded under **LOW** (per-run mean 0.86 % beyond +eps_far, maximum 8.4 %; data age up to 0.3 s > tau_max) and MEDIUM (3.6 %, maximum 15.6 %), so P1 under LOW and MEDIUM is tested, not proved. Gate-structure contacts are outside P1's drone-drone scope: they rest on `struct_keepout` with the drone's own (drifted) navigation plus obstacle classification | none observed | P1 near structures with large navigation drift (no contact observed, no formal coverage) |
| **P2** mutual exclusion | M1/M2 precedence uniqueness (proof); M4a-f geometry (exhaustive numeric); M4g exit observability, geometric only; M3 latch BMC 42 s, assuming **a healthy crossing (V_LO)** and **sound witnesses** (only hulls produce witness echoes); B2-A1 randomized and B2-A2 exhaustive envelope soundness | max occupancy 1 in 195 gate runs; 0 / 536 commits while occupied; minimum entry gap 11.5 s | witness soundness violated under MEDIUM (7 premature releases; 69 of 260 sonar witnesses match no drone); perfect detection violated (9 whole crossings missed, precedence was the barrier); in-CR holds by FAILSAFE/CA/SW (dwell up to 12.5 s) are outside M3's monotone crossing; stalled occupant (M3f counterexample), not covered with a silent beacon, and in B2 the sonar witness can still release | none observed, but safety came from kinematic margins | P2 with a stalled occupant (never exercised in the campaign) |
| **P3** recovery | F1-F3 ranking functions: **the drone's own slot error relative to its own (possibly slowed) reference**, under the current envelope (F5) and the catch-up margin; F4 mode selection (proof) | CLEAN cells 100 % (but see the horizon issue); degraded cells 83-96 % PASS | navigation error, association errors and **inter-drone clock consistency** are not modelled. "G(lost → F recovered)" on true geometry is **not** implied by F1-F3 **[WRONG claim scope]** | 23 B2 runs not recovered at the deadline: clock divergence (diverging error), gate stalls, vacancy flapping | eventual recovery in diverging runs: no positive mechanism exists, so it is not expected; an extended-horizon rerun would settle it |
| **P3-deg** degraded formation | F6 BMC and F6r ranking: **each drone individually** declares a never-returning neighbour vacant within t_rejoin. No agreement across drones and no protection against false re-occupation is proved | B2_CLEAN lost_drone_timeout PASS; 5 failures under MEDIUM or BLACKOUT | false re-occupation from range-only echoes, stale fixes of a far lost drone (far dropout 0.5) | 5 runs with inconsistent vacancy sets | — |
| **Gate liveness** (no healthy drone blocked) | none: no ranking for the pass or hold phases; Q1 only says a calm queued drone commits or yields | 88-100 % of healthy drones pass per cell | the relocalization hold has no bound; bar traps; phantom echoes | 8 committed stalls and 1 queue stall (B2) | — |

---

## G. Prioritised implementation plan (minimum necessary, in dependency order)

The safety objectives P1 and P2 are not weakened anywhere. No new communication and no coordinator are introduced.

### G.1 Instrumentation, corrected metrics and offline re-evaluation (no behaviour change)

* **Problem.** Five gaps prevent a full diagnosis or a correct classification:
  * the logs cannot settle the witness echoes, the acoustic fixes at the decision instants or the reasons the map fit
    was rejected;
  * the classes hide deadlocks;
  * horizons differ between cells;
  * the terminal monitor and the classifier use different requirements;
  * the P3 definition is vacuous.
* **Components:**
  * `controller.py` (`last_record`);
  * `perception/acoustic.py` (per-fix log);
  * `perception/gate_perception.py` and `acoustic_gate.py` (full A1/A2 per drone, and `heard_inside`);
  * `perception/exit_witness.py` (witness echo history);
  * `perception/structure_aiding.py` (rejection reason);
  * `referee/robustness_metrics.py`, `referee/referee.py`, `referee/terminal.py`;
  * `scripts/run_sensor_robustness_campaign.py` (same horizon rule for all cells);
  * `scripts/analyze_campaign.py`.
* **Solution:**
  * gate events E1-E8 (Section D.5);
  * a stall detector: for each healthy drone, a progress function (along-path position, gates passed, formation error)
    that does not improve for T_stall while the drone is not in a timer-bounded wait;
  * classes PASS / DEGRADED_COMPLETE / INCOMPLETE_PROGRESSING / STALLED / SAFETY_FAILURE / INVALID;
  * the P3 v3 criterion of Section E.4;
  * one requirement set for the terminal monitor and the classifier;
  * one horizon rule for every cell (Section G.7);
  * fix the `at_queue` string test (`robustness_metrics.py:161`);
  * a causal SAFE_DEGRADED: a conservative event counts only if it explains the missed requirement (for example a
    FAILSAFE that is active at the deadline or covers the stall interval), never because it happened at some time;
  * criterion 3: E2 (premature release) is counted and reported. Under v2 it is a safety failure as written; v3 must
    say explicitly how E2 is classified (Section B.4);
  * report chattering events, idle-gate intervals and completion-time distributions per cell;
  * re-evaluate the 624 historical runs offline and report v2 and v3 side by side. The v2 table states that the
    7 E2 runs fail criterion 3.
* **Alternatives:** keeping the current classes. Rejected, because deadlocks would keep looking like degraded success.
* **Risks:** larger logs (acoustic fixes at 1 Hz per pair are small).
* **Formal impact:** none (referee-side only).
* **Tests:** metric unit tests on synthetic traces (a premature release, a stall, an episode in the vacuous band);
  regression on 3 archived runs with known answers (the cases above).
* **Success criteria:** the offline re-evaluation reproduces the counts of this report: 7 E2, 0 E3, and 9 STALLED B2
  runs (8 committed stalls plus the queue stall).
* **Previous results:** remain valid as historical evidence for `ad4fb5e`; the new metrics re-read them.

### G.2 B2-only consolidation (structural)

* **Problem:** B0 is the default runtime and the deadline reference; the documents and the campaign are organised as
  B0 vs B2.
* **Components:**
  * `config.py` (`BeaconConfig.mode = "ACOUSTIC_RB"` by default);
  * `runner.py`, `sim/holo_env.py`;
  * campaign and analysis scripts: remove the `B0` protocol, keep the BLACKOUT channel as the acoustic-outage
    condition; remove `scripts/analyze_b2.py`;
  * `tests/` (B2 default, explicit outage tests);
  * `docs/*`, `README.md`, `REPORT.new.md`;
  * archive `results/robustness/raw_final/B0` with a manifest as the historical baseline.
* **Solution:** a single operational architecture, B2, with a defined acoustic-outage degraded mode that is the
  sonar-only logic. Every outage behaviour is preserved and re-verified.
* **Alternatives:** keeping B0 as a mode. Rejected per the brief.
* **Risks:** the default-config change ripples through about 400 tests; outage coverage must not be lost.
* **Formal impact:**
  * re-run all checks;
  * re-label M3, M4 and S* as "outage mode, also valid in nominal B2";
  * correct GUARANTEES (B2-M4g holds for n = 2 and 3; the P3 scope; the P1 premises: mutual detection, both drones
    braking, assumption D, the echo envelope and its measured exceedance under LOW and MEDIUM);
  * correct the README check count (205) and the "communication-free" wording;
  * record in ACCEPTANCE_CRITERIA the post-freeze change ad4fb5e and the B2 runs discarded at stage E (documentation
    only; no result changes).
* **Tests:** full suite; `test_b2_isolation`; silent-channel equivalence (B2 with BLACKOUT gives the same decisions as
  the old sonar-only path on recorded inputs).
* **Success criteria:** the full suite and the formal suite pass; no B0 protocol remains in code or scripts; the
  documents use the correct terminology.
* **Previous results:** B0 results become an archived baseline only.

### G.3 Gate entry admission (two-stage) and witness/veto corrections (P2-relevant)

* **Problem:**
  * sonar witnesses can come from clutter;
  * acoustic contradiction is ignored;
  * P2 for a stalled occupant rests on nothing;
  * `heard_inside` and the acoustic exit are over-conservative in a way that costs liveness.
* **Components:**
  * `ha/spec.py` and `control/flows.py`: the entry check is made explicit, either as a new observation `entry_ok` with a
    sub-mode MUTEX_PASS_HOLD/TRANSIT, or as a guard on a new edge;
  * `perception/gate_perception.py`, `exit_witness.py`, `acoustic_gate.py`.
* **Solution:**
  * **(a) Stage 2 at s_hold:** `loc_ok ∧ close-range sonar CR-clear for N ≥ 3 captures ∧ ¬heard_inside(B2) ∧ latch
    FREE`; otherwise hold and observe. There is no positive acoustic requirement.
  * **(b) Sonar witness hardening:**
    * at least 3 captures with range evolution consistent with a hull leaving (not a single blip);
    * in B2, vetoed while `heard_inside`.
  * **(c) Tighter `heard_inside`, for liveness:**
    * a freshness window of one ping period plus latency (about 1.3 s) instead of 2.5 s;
    * require the CR intersection on 2 consecutive accepted fixes of one track;
    * exempt a track heard consistently beyond the exit face (A3 nominal) and moving away.
  * **(d) Release on positive acoustic exclusion of every other plan drone** (A1 or A2 for all of them). P2 concerns
    fleet drones; a non-drone object in the corridor remains a P1 obstacle for the separation layer.
  * **(e) Option, to be evaluated in H.2 before adoption: positive acoustic occupancy.** In B2, the tightened
    `heard_inside` of (c) also sets the latch BUSY. This is caution only (the safe direction) and addresses the
    missed crossings of Section D.2. Its liveness cost must be measured first, because the current `heard_inside`
    already produces false positives.
* **Alternatives considered:** mandatory acoustic confirmation (deadlocks under outage); longer t_clear or t_occ_max
  (slower, not evidence); continuous `occ_busy` re-check during the approach (same weak queue-view signal); moving the
  commit point (changes M1/M2/M4 geometry).
* **Risks:**
  * (a) adds a hold before the CR, bounded by G.4;
  * (b) delays sonar exits by 1-2 captures;
  * (c) and (d) rely on the acoustic envelope (multipath beyond 3 m is the residual risk, quantified in open issue
    O-12).
* **Formal impact:**
  * G1 (entry condition) re-encoded, plus new observation invariants (O1-O5, I1-I4) for `entry_ok`;
  * D1, D2, H1-H3, L1 and Q1 re-run for the new edge or sub-mode;
  * M3 re-encoded with stage 2, and with the premises the logs falsify replaced by bounded nondeterminism: spurious
    witnesses, missed corridor detections (up to the observed 3.3 s) and holds inside the CR by FAILSAFE, CA or SW;
    the horizon justified by the slowest admissible crossing (reviewer A: 51.7 s for n = 3, longer than the current
    42 s; a 65 s rerun of the current model is still UNSAT) or replaced by induction;
  * new **M4h** (an occupant anywhere in the CR is unmasked to the hold-point sonars);
  * new **M3s** (P2 without the healthy-crossing assumption: the stalled occupant is blocked at stage 2), which turns M3f
    into a liveness-only scope;
  * new **B2-V1** (veto completeness: a hull in the CR with an in-envelope fix gives `heard_inside`);
  * new **B2-X** (A1/A2 exclusion of all drones implies no drone in the CR), extending B2-A2;
  * B2-S re-checked: acoustics still never reach separation or precedence.
* **Tests:**
  * unit tests of stage 2, witness persistence, veto and exclusion release;
  * offline latch replays of the 7 premature releases and of the integrated_short s2 and LOW mutex_deadlock_resolution
    s2 traces from the logged targets (approximate: per-sector ranges are not logged);
  * a new scenario **gate_stalled_occupant** (thruster failure inside the CR).
* **Success criteria:**
  * E2 reduced, E4 = 0 by construction;
  * E7 > 0 in gate_stalled_occupant with P2 held;
  * no added stall in CLEAN;
  * B2_CLEAN gate waits no longer than BLACKOUT-CLEAN.
* **Previous results:** not valid for the new gate logic. Gate results must be rerun.

### G.4 Gate liveness: bounded relocalization, bar-trap recovery and an explicit abort (healthy drones)

* **Problem:** in 8 runs a committed healthy drone stalls before the CR for 100-260 s and blocks the gate.
* **Components:** `control/flows.py` (pass path and hold), `perception/structure_aiding.py`, `ha/spec.py` (abort edge),
  `mission.py` (pass path), `perception/acoustic.py` (exclusion query).
* **Solution, in order of preference:**
  1. **(a) Observability-aware approach.** Merge onto the gate axis earlier, at s ≈ -(acquisition distance) = -4.2 m,
     so that the side sonars see both pillars symmetrically before the drone can be in front of a bar. Verify first by
     replaying the logged geometry.
  2. **(b) B2 identity-assisted fit.** Accept a structure fit beside the opening, or a correction up to MAX_CORR, when
     B2 fixes positively exclude every other drone from the location of the matched echoes. This removes the echo-identity
     assumption that motivated the C5 restriction (`structure_aiding.py:30-36`: the wrong fits rested on "the echo of
     another drone taken for a bar").
  3. **(c) Bounded search.** If `loc_ok` is still false after T_reloc (for example 2 × RELOC_PERIOD), perform a bounded
     lateral sweep (±MAX_CORR) at a stand-off before the CR, inside `struct_keepout` and the separation layer.
  4. **(d) Watchdog with abort.** After N_retry failed cycles, abort the passage. This is a new edge MUTEX_PASS →
     MUTEX_APPROACH, allowed **only before the CR entry plane** (s < -(cr_half_len + R_OUT + margin)). The drone returns
     to its queue point and re-queues; if repeated, it goes to FAILSAFE outside the corridor. The outcome is explicit and
     logged; it replaces a silent deadlock.
* **Alternatives:**
  * accepting fits beside the opening without B2 exclusion (shown wrong by 0.4-1.2 m in the authors' replay);
  * forcing passage: rejected, because safety comes first;
  * a longer hold: no evidence that it ever ends.
* **Risks:**
  * (b) depends on fix freshness and on the envelope;
  * (d) changes the automaton (G2 "no PASS → YIELD fallback" must become "no fallback after the CR entry plane").
* **Formal impact:**
  * G2 restated;
  * D1, D2, L1 and H1-H3 for the new edge;
  * P2 preserved because an aborting drone is outside the CR by guard;
  * M4b and M4s merge clearance for the new path;
  * a new liveness ranking for the bounded retries (retry counter plus timer). The relocalization *success* itself stays
    empirical.
* **Tests:** offline replay of the 8 stall traces with the new aiding acceptance; unit tests of the watchdog and abort;
  targeted reruns (H.2).
* **Success criteria:**
  * no STALLED gate run in the targeted reruns;
  * every abort is explicit, outside the CR, followed by passage or an explicit degraded outcome;
  * no new contacts with the structure.
* **Previous results:** gate scenarios must be rerun.

### G.5 Formation clock consistency (the main P3 cause)

* **Problem:** private `tau` diverges after asymmetric or false missing detections and is never re-synchronised.
* **Components:** `control/formation_time.py`, `control/flows.py` (sigma consensus), `perception/formation_perception.py`
  and `perception.py` (presence evidence).
* **Solution:**
  * **(a) Positive-evidence slowdown in B2.** Slow down only if the missing neighbour has no fresh fix near its slot
    region, the region grown by v_bound·age with a bounded window. When it is heard, slow down only if it is behind
    along-track (it is the one to wait for).
  * **(b) Bounded re-synchronisation.** When neighbours are perceived again (identified fixes in B2, or the sonar
    front_residual), drive the drone's own rate so that the perceived along-track offsets to its neighbours converge to
    the template offsets. This is a local, communication-free progress consensus. The rate stays in
    [v_wait, v_clock + δ] and keeps the F1 margin.
  * **(c) Optional: bound the accumulated tau difference** that one missing episode can create.
* **Alternatives:** removing reduced progress (the lost drone would never catch up: Fm4 shows waiting is needed);
  raising the sigma caps (unsafe for same-lane pairs, per the docstring).
* **Risks:** consensus on wrong associations (mitigated by using identified fixes first); same-lane following
  interactions.
* **Formal impact:**
  * F1-F3 re-run with v_ref in [v_wait, v_clock + δ] and the catch-up margin re-checked (F5, Fm1);
  * a new pairwise contraction lemma: |τ_i - τ_j| decreases while both perceive each other within the envelope;
  * F6 unchanged.
* **Tests:** unit tests on recorded perception traces; replays of the formation_six and formation_square traces.
* **Success criteria:**
  * in the targeted reruns, max |τ_i - τ_j|·v_plan < 0.5 m at the end;
  * no clock-divergence failure;
  * the false-missing rate reported separately.
* **Previous results:** formation results must be rerun.

### G.6 Vacancy and re-inclusion with identity (P3-deg)

* **Problem:** inconsistent vacancy sets and false re-occupation (5 runs).
* **Components:** `perception/perception.py` (missing, vacancy, re-inclusion), `formation_perception.py`.
* **Solution:**
  * **re-occupation in B2** only by an identified sighting: a fix of that slot's drone inside the slot region. An
    unidentified echo never re-occupies a slot whose drone is heard far away;
  * **"positively absent":** the last fix grown by v_bound·age is still farther than e_lost from the slot, so stale fixes
    of a far lost drone still count;
  * **under outage:** the current sonar rules, documented as degraded.
* **Formal impact:** F6 unchanged; extend B2-A1 to slot regions; unit tests (as for C3).
* **Success criteria:** lost_drone_timeout passes in MEDIUM cells (targeted).

### G.7 Acceptance criteria v3, deadline policy and freeze

* **Deadline.** For every scenario and cell:

  ```
  deadline = 1.2 × T_bound
  T_bound  = T_plan(path length / v_plan + planned holds)
           + n_gate × (T_cross + T_release + t_clear + T_entry)
           + T_catch-up (e_max / (v_recovery - v_clock) + t_hold)
           + Σ t_rejoin over the scripted faults
           + a FAILSAFE allowance per profile (p99 of the measured FAILSAFE fraction)
  ```

  The terms are:
  * T_cross: queue-to-exit distance / v_pass, about 18 s;
  * T_release: bounded by t_occ_max + t_clear;
  * T_entry: the stage-2 hold bound.

  A run that misses this deadline has violated a protocol time bound, so it is a liveness failure (or STALLED, from the
  stall detector).
* **Same rule for every cell, including sensor-CLEAN.**
* **Exploratory diagnostics only** use a 3× horizon, to separate slow runs from stuck ones.
* **Reporting:** completion times as survival curves (Kaplan-Meier, censored at the deadline), not only pass or fail.
* **Freeze** ACCEPTANCE_CRITERIA v3 and the code commit before the final campaign.

---

## H. Final validation plan (B2 only)

* **H.1 Offline (no simulation).** Re-evaluate the 624 runs with G.1. Run unit tests and formal obligations for each
  change: G1-G3, D*, H*, L1, Q1, O*, M3, M3s, M4h, B2-V1, B2-X, F1-F6, F5, and the clock-contraction lemma.
* **H.2 Targeted exploratory diagnostics** (HoloOcean, 3× horizon, full instrumentation, before and after each change,
  5 repetitions per case because engine noise is unseeded). One or two representative cases per mechanism:

  | Mechanism | Representative cases |
  |---|---|
  | relocalization / bar trap | gate_single MEDIUM-ONE s0; lost_drone_mutex MEDIUM-SUBSET s0 |
  | phantom corridor echo | integrated_short MEDIUM-SUBSET s2 |
  | heard-inside | mutex_deadlock_resolution LOW s2 |
  | witness | gate_single MEDIUM-SUBSET s1 |
  | clock divergence | formation_six MEDIUM-ALL s1; formation_square MEDIUM-ONE s2 |
  | vacancy | lost_drone_timeout MEDIUM-SUBSET s2 |
  | stalled occupant | new scenario gate_stalled_occupant (CLEAN, MEDIUM, BLACKOUT) |

  This is about 9 cases × 5 repetitions × 2 versions ≈ 90 runs, one simulator at a time and at low priority. It is
  exploratory evidence, not a result.
* **H.3 Reference runs, held-out set and freeze.**
  * Run B2 CLEAN references (16 scenarios + gate_stalled_occupant, 5 seeds) at the candidate commit to measure the
    T_plan and T_cross terms.
  * Define, before any result is seen, a **held-out set** never used in development:
    * new seeds (for example 100-109) for every scenario;
    * at least one held-out gate scenario with a different queue size or approach geometry, built from existing
      scenario primitives, with no parameter change.
  * Freeze criteria v3, the held-out set and the code commit together, and record the commit in every run's
    provenance.
* **H.4 Frozen final campaign, B2 only.** 8 cells:
  1. CLEAN / CLEAN;
  2. LOW-ALL / LOW;
  3. MEDIUM-ONE / MEDIUM;
  4. MEDIUM-SUBSET / MEDIUM;
  5. MEDIUM-ALL / MEDIUM;
  6. CLEAN / MEDIUM (channel only);
  7. CLEAN / BLACKOUT (outage);
  8. MEDIUM-ALL / BLACKOUT.

  In each cell: 11 non-gate scenarios × 5 seeds, plus 6 gate scenarios (including gate_stalled_occupant) × 10 seeds,
  i.e. 115 runs per cell and 920 runs in total.

  **Statistical rationale:**
  * per cell, the Wilson 95 % CI half-width is about ±6 % at p = 0.9;
  * 480 gate runs with zero P2 violations bound the per-run rate below 0.63 % (rule of three, 95 %);
  * report Wilson CIs per cell;
  * Holm-corrected comparisons between cells;
  * exact McNemar only for the exploratory before/after pairs;
  * Kaplan-Meier completion curves;
  * E1-E8, stall, idle-gate and chattering counts;
  * every safety event with its zero-event bound;
  * **the acoustic benefit is tested only by B2 against B2 under outage** (cells 5 vs 8, and 1 vs 7), never against
    the archived B0. The pairing is called nominal, because engine noise is unseeded. Results are reported separately
    for development seeds and held-out seeds.

  **Separate, separately reported studies** (not changes to the frozen profiles):
  * a P1 sensitivity study to assumption (D): a softer cone edge or range-dependent detection;
  * a re-run of the acoustic residual-risk analysis with the widened envelope (open issue O-12).

  A smaller alternative, if time is short, is 3 seeds for non-gate and 6 for gate scenarios (about 560 runs). It
  weakens the per-scenario conclusions.
* **H.5 The 624-run campaign** is reported as historical evidence for `ad4fb5e` under criteria v2, and v3 is recomputed
  offline. It is not reused as evidence for the changed implementation.

---

## I. Formal verification implications (item 10)

| Change | Obligations to update or re-verify |
|---|---|
| Remove operational B0 | all checks re-run under the B2 default; M3, M4 and S* re-labelled as outage-mode evidence; GUARANTEES corrected (B2-M4g status, P3 scope) |
| Gate admission and occupancy (stage 2, witness, veto, exclusion) | G1, D1, D2, H1-H3, L1, Q1, O1-O5 / I1-I4 (new `entry_ok`), M3 (re-encoded), M3m, M3f (scope), new M4h, M3s, B2-V1, B2-X; B2-A2 extended; B2-S re-checked |
| Second check before the CR | M4h (observability from the hold point); M3s; liveness: a bounded stage-2 hold when the CR is free and observable (BMC over k captures) |
| Healthy drones not stuck at the gate | G2 restated (abort only before the CR entry plane); D/L checks for the abort edge; a new retry-counter ranking; M4b / M4s for the new path; H1 unchanged (FAILSAFE keeps priority; dwell data do not justify changing it) |
| 1.0 m formation tolerance | referee only: no formal obligation. If the controller thresholds were also changed: F3 balls < e_ok (holds), F4a-d re-run, hysteresis versus on-board noise (new) |
| Recovery timeouts and deadlines | t_rejoin → F6 / F6r; t_occ_max → M3 (V_LO = 2.7 m / t_occ_max); deadlines: referee only |
| Acoustic neighbour tracking (presence, re-synchronisation, re-occupation) | B2-A1 extended to slot regions; C3 tests; new clock-contraction lemma; F1-F3 and F5 with the new v_ref range; B2-S |
| P1 premises made explicit (no parameter change) | new S1 one-sided variant (`a_pair = a_brake`) at the documented per-drone capability, with the provenance of the 0.74 m/s² calibration; otherwise the claim is restricted to mutual detection. S1-S4 premises (mutual detection, assumption D, the echo envelope) written into GUARANTEES |
| Latch model faithful to the logs | M3 with spurious-witness, missed-detection and in-CR-hold nondeterminism (G.3); horizon justified or replaced by induction; an N = 2-3 timed model of queue, latch, stage 2 and abort (UPPAAL-style), with conformance checked against logged traces |
| Evidence labels | the "exhaustive numeric" sweeps (M4a-g, B2-A2, M4g) relabelled as grid sweeps, or given a discretisation-error bound; F6r described as a counter argument, not P3-deg evidence; the README count corrected to 205 |

Passing unit tests is not a substitute for any of these. For P3, unbounded eventual recovery holds under the stated
assumptions. Bounded completion in the experiments is a separate, empirical statement.
