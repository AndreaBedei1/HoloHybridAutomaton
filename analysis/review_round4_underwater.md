# Round-4 review, underwater physics and acoustics: commit `02f6a52`

Baseline `analysis/review_round3_underwater.md`, checked against `git diff 6ce8ba7 02f6a52`, `med6` (66 runs, d2ca747) and `med7` (02f6a52, 4 runs finished when I read it). No simulator; code and results untouched. Of the resolution note I read only the underwater table and the stress-run findings; no formal review.
Tags: **[M]** measured from repository logs or files; **[D]** derived by me; **[A]** assumption of the repository.

## Verdict

**Conditional: freeze only after conditions 1-3 below.** No code change is required for the acceptance campaign, whose matrix excludes the stress set, but the candidate does not hold P2 in a stress case it pre-registered, and the documents give the wrong mechanism for it.

1. Correct `GUARANTEES.md:40,78` and `gate_perception.py:269-271` to state the mechanism of R1, and state before the freeze that the stress metric "no SAFETY_FAILURE" is expected to fail on `lost_drone_mutex_carried`.
2. Let `med7` finish. A new failure class, or a contact of a live drone, reopens this verdict.
3. Reword the conclusion on `lost_drone_mutex_rise_low` s960 (R2).

If the owner wants P2 to hold in the carried case, that is a code change and needs fresh seeds.

## Findings

### R1. MAJOR. A live drone was admitted into a CR holding a dead hull, and the declared limit describes a different path
`med7/lost_drone_mutex_carried/s981`, SAFETY_FAILURE "P2 violated" at t = 48.2 s, at the candidate commit.
- [M] `drone_1`, not engaged (`tx` [0,0]), fails at t = 23.3 s at s = -5.37, l = -0.28 and is carried to s = -0.29, dz = 0.96, against the top bar, by t = 43. `drone_0` had committed at about t = 23.1. Its FRONT echoes are STRUCTURE up to the hold point. It is admitted at t = 45 with its own latch BUSY since t = 37, and enters at t = 48.2. Minimum separation 1.18 m, no contact; `latch_releases` all 0.
- Cause [M, code]: stage 2 is `flows.py:330`, and `entry_ok` (`gate_perception.py:274`) has no latch term. The latch gates the commit only, so it cannot cover a hull that arrives after the commit. `GUARANTEES.md:78` says such a hull "is released by the sonar fallback after t_occ_max"; here nothing was released. `GUARANTEES.md:40` and `gate_perception.py:271` say the latch argument covers it; here it did not. The echo masking I predicted in N1 is what made the hold point "quiet".
- Rate: same timing in 3 of 4 runs of this scenario. [M] `med6` s960 and s961 fail at t_on = 24.9 and 23.1 with `drone_0` committed and 2-3 m from the CR when the run stopped at the hull contact. `med7` s980 passes only because the trigger fired at t = 103, after `drone_0` had passed. Expect recurrence; N is small.
- Plausibility [D]: with Table A1 values the drag at 0.3 m/s is about 17 N against 2 N of buoyancy, so friction holds a hull pressed on the upstream face of the bar while the current lasts. The wedge is not only an engine artefact, though its details are (R5).
- A latch term in `entry_ok` would only delay the entry by `t_occ_max`, 25 s [D]; the hull stays masked. A real fix needs a reach bound on the silent drone or an owner decision that it blocks the gate.

### R2. MAJOR for the claim. The s960 conclusion is not supported: the binding cause is the map fit, not the latch
Resolution note, third bullet. `med6/lost_drone_mutex_rise_low/s960`, INCOMPLETE.
- Supported [M]: safe (P1 2.83 m, no CR entry, E2 and E4 zero); `drone_1` lateral error 0.45-1.13 m, median 0.75; dead hull on the surface about 10-13 m away.
- Not supported [M]: "renews the latch until the deadline". `drone_1` is committed in 3687 of 7323 steps and reaches the hold point repeatedly, with 9 `ENTRY_HOLD_TIMEOUT`. In 1871 of 1879 hold steps `loc_ok` is False, while `entry_ok` is True in 46 % of them. In 78 % of hold steps the fit is good (rms 0.034 m, median lateral offset -0.8 m, the true error) but is refused: `clearance` is below 0 and `identified` is False (`structure_aiding.py:349-354`). The correction that would fix the drone is rejected because the drone is beside the opening.
- So this is not established as "a cost of the rule that silence excludes nobody". Why `identified` stays False after 71 s I could not determine; `others` is not logged. It is a liveness limit of the localization for lateral errors above about 0.5 m, independent of rise speed.
- Consequence: 1 of 7 runs of the `lost_drone_mutex` family at d2ca747 [M], with 6 seeds of it in the acceptance matrix. Reword, and log `others` at the hold.

### R3. MINOR. `gate_pinned_occupant` now tests only the claim path
`scenarios.py:486-501`. With six emissions before the failure, P(a survivor never hears it) is about 0.29^6 = 6e-4 [D], so the pre-registered stratification (`ACCEPTANCE_CRITERIA.md`, stress set) will be empty. The masked, no-claim case now exists only in `lost_drone_mutex_carried`. Say so in the report. Neither `med7` run of this scenario had finished.

### R4. MINOR. Numbers and citations to correct
- `config.py:221-222`, design note and §10: "21 s on expected-neighbour links in the v10 runs". 21.3 s was my `med3` figure. On v10 the same proxy gives a maximum of 29.3 s, `lost_drone_rejoin` s800 [M; upper bound of the heard gap]. Still below 45 s.
- `ACCEPTANCE_CRITERIA.md` v7: "251 runs of batches med3 to med6". I count 54 + 54 + 66 + 66 = 240 directories [M]; 251 matches the med5 status record, which over-counts. The "4 contacts" holds in the 240, but `med5` s941 (INVALID, reason "contact" at t = 11.2 s, hull at dz 0.87) is a fifth dead-hull contact.
- Resolution note N3 says `_axial` is declared in §10; it is not [M]. It is in the docstring and `GUARANTEES.md`, which is sufficient.

### R5. Criteria v7: fair, with one caveat
Excluding the sensor of a dead hull is physically fair: no actuation, about 0.2 J of kinetic energy at most (32 kg, 0.1 m/s) [D], and the failure model makes it rise against whatever is above it.
- Nothing is lost for live drones [M, code]: every pair still counts in P1 and contacts, including live-dead pairs (`referee.py:193-201`); P2 counts the dead hull (`:220-228`); a dead hull striking a live drone fires the live drone's sensor, because `OnActorHit` is bound per agent (`CollisionSensor.cpp`, `HolodeckAgent.cpp:33`). Such a strike counts against the fleet, which is conservative.
- Caveat: runs now continue past contact on unvalidated engine contact mechanics (round 2, F2). What happens to the hull after the contact is a simulation result only.
- v7 is also what exposed R1. "No live drone involved" in the note is true of the four `med6` failures only because they stopped at the contact.

## Status of the round-3 items

| Item | Status at 02f6a52 |
|---|---|
| N1 masked part | `gone` removed, so the A4 path I named is closed [M]. Declared, but with the wrong mechanism (R1) |
| N2 carried scenario | Resolved: on-axis, not engaged, low rise [M]. It now tests what it should |
| N3 `_axial` | Resolved as a declared limit. `axial_total` bound added. In `med6`, 19 of 21 corrections reduced the error, median 0.15 to 0.05 m, max after 0.17 m [M]. The two that did not are s960 with lateral error above 0.5 m, as predicted |
| N4 statistics, `MEDIUM_HT` | Text corrected; RNG key fixed. The 21 s attribution is wrong (R4) |
| N5 rise variants | Resolved. `lost_drone_front_rise_low` 2/2 PASS, min separation 2.69-2.74 m; hull rises at 0.049 m/s in the run [M]; the follower holds about 3 m behind it |
| N6 | Declared in §10 |
| N7 | Off as stated, but s960 does not show it was the cost (R2) |
| Acceptance gaps | Both closed: like-for-like line and pre-registered stress set |

## Not verified

No engine run. `med7` was incomplete: 4 runs, none of `gate_pinned_occupant`. Whether the R1 timing recurs on held-out seeds. Real hull contact behaviour. Why `identified` fails in s960.
