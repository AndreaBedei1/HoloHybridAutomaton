# Round-6 review, underwater physics and acoustics: commit `e034755`

Baseline `analysis/review_round5_underwater.md`, checked against `git diff ef54530 e034755` and `med9` (19 runs finished when I last read it: all 12 stress runs, 7 standard). `med8_mixed_stopped` was read only for the mechanism of one run. No simulator; code and results untouched. My scripts are in my own scratch folder `r6_underwater`. HEAD moved to `3e3db7b` while I worked: it changes the past-centre rule only (`flows.py`, a test, one row of `GUARANTEES.md`). I read that diff; every result here is at `e034755`.
Tags: **[M]** measured from repository logs or files; **[D]** derived by me; **[A]** assumption of the repository.

## Verdict

**Condition 1 is lifted; condition 2 is still open because the standard set is not finished.** On the evidence so far I would freeze once the remaining standard runs of `med9` finish with no new failure class and `ENTRY_HOLD_TIMEOUT` near the `med6` level (0 per run in the standard gate scenarios [M]), and once the carried runs have been repeated at the commit actually frozen (see d). No new safety finding.

- Condition 1, `lost_drone_mutex_carried`: met on both seeds [M]. s987 and s988 are INCOMPLETE, as predicted; P2 holds, maximum occupancy 1, E4 = 0, no contact of a live drone (one dead-hull contact each), minimum separation 1.50 and 2.28 m.
- Condition 2, standard set: 7 of 54 finished, all PASS, none with a gate (`p1_*`). Not evaluable for the gate cost.

## (a) Physical soundness

**Heading integrator (`lowlevel.py`): sound.**
- A PI heading loop is conventional. The integral time of 30 s is well above the closed-loop time of the heading ([D] about 2 s, a rough estimate on the engine's numbers; slower on a real hull but still well under 30 s).
- It removes the fault it was added for [M]. Beyond 6 deg for more than 5 % of queue time: 4 of 63 drone-runs in `med6` and `med7` (my filter; the note counts 5 of 57), 0 of 13 in `med9`. Maximum 16.0 deg to 5.9 deg. `med7` s981 drone_1 was at 8.4 deg rms and 76 % of its queue time beyond 6 deg.
- Long holds [M]: 700 s holds at 1.3-1.4 deg rms, no spectral peak (dominant component at the record length, share below 0.01), reconstructed integral at most 3.5 of the clamp of 8 (at most 0.024 of command against the 0.12 limit). No limit cycle.
- Avoidance modes and currents [M]: at the gate, p95 of the heading error is 2.5 deg in SEPARATION_WARNING, COLLISION_AVOIDANCE and FAILSAFE (4.9, 3.7 and 5.1 before), maximum 4.8 deg. The pooled figures include the 0.3 m/s carried runs. It degrades nothing.
- Windup [D]: the clip needs P and I both near their maxima (0.07 + 0.054 against 0.12), so saturation is not a practical path; the clamp limits any release transient to about 15 deg, not observed.
- MINOR, real vehicle: compass bias that depends on thruster current is unobservable. A heading integrator will turn the vehicle to null the apparent error, up to a few degrees of true heading. Say so if a real deployment is claimed.

**Past-centre rule (`flows.py:338-339` at e034755): sound for the mechanism, and it worked in the one run that used it.**
- The trigger run reads as the note says [M]: `med8_mixed_stopped` s986, `drone_0` at the hold is pushed to s = +1.4 by the avoidance layer ahead of a hull closing on the same axis, then sent back through the CR, P2 at 53.4 s, 1.55 m.
- `med9` s988 [M]: the same push, estimated s > 0 at t = 45.7. The drone goes on to s = 3.5 and leaves, with no reversal. P2 holds.
- At e034755 the test uses the estimated s only, so with an along-axis error of about 1 m (RC7) it could fire before the CR. `3e3db7b` addresses this: the threshold is now `nav_err_m - cr_half_len` = +0.2 m, inside the CR under the declared error, it releases if the estimate falls back before the CR, and it is logged and declared in `GUARANTEES.md`. That reads correctly to me; it has not run in the engine.
- MINOR: the test has no lateral term, at either commit. In s988 the drone was at l = 1.5-2.5 m, beside the frame, not in the opening. The rule sends it on past the gate without having passed it; `gate` is counted as missing, which is right. No safety consequence.

**Sticky adrift (`acoustic_gate.py`):** physically right. A hold should not end because the water estimate fell, and a deaf receiver keeping its holds is the conservative choice.

## (b) Predictions against `med9`

| Prediction (round 5) | Result [M] |
|---|---|
| Carried runs: no P2, INCOMPLETE, gate held for good | Both seeds, as predicted. `adrift` from t = 33 s in s987 |
| Silence needed at 0.3 m/s: 4.5-10 s | Consistent: first hold 5-6 s after the failure; live-sender episodes at most 7.9 s |
| Rise variants unaffected; standard-like runs no measurable cost | 4 runs on time at 169 s; adrift-only cost 0.7-11.5 s per run, longest 6.7 s |
| MEDIUM_HT hold 8-16 % | Not yet run (pre-registered for the frozen code) |
| Standard set, `ENTRY_HOLD_TIMEOUT` near `med6` | Not yet evaluable |

## (c) The adrift log

`gate.adrift` separates the cost as needed [M]. Note for the analysis: a sender that also holds a claim appears in it, so the added cost is the adrift set minus claim holders and dead senders. That is the figure in the table: 62 and 58 s in the 753 s carried run s987, in episodes of about 2 s.

## (d) Freeze

Yes, under the condition above, with one addition. `med9` ran at `e034755`, and the code is now `3e3db7b`. The change is confined to the branch that ran once in `med9`, so I would not ask for the whole standard set again, but the carried scenario is the only one that exercises it (s988 here). Repeat it at the commit you freeze.

What the stress evidence supports: n = 2 per scenario, in a development batch; the pre-registered held-out stress set is what the report should quote. The limits already declared stand: the hold is for ever within reach; transients exceed the bound; the rule is untested under `MEDIUM_HT`.

## Not verified

The other 47 standard runs, including every standard gate scenario. `3e3db7b` in the engine. Any real-vehicle behaviour of the heading loop. Real compass response to thruster current (argued, not measured).
