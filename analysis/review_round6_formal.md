# Review round 6: formal verification

Commit `e034755` on `feature/b2-medium-robustness`, against `ef54530`. Independent reviewer, 2026-11-10. Code and results were not modified; no simulator started; `med9` not read; no underwater review opened.
Tags: [O] observed (real classes on crafted inputs, log data); [I] inferred. Scripts are in my own folder `r6_formal`, and I checked the first line of each before running it.

**Verdict: not ready to tag as it stands, but the distance is small.** N6 to N8 are closed. Rule 3 changes no formal assumption. Rule 2 does what RC11 needs, but its latch has no way out (N9, MAJOR): a one-line change, which I tested in memory. With N9 fixed and `formal/results` regenerated at the tag, I see no blocker on the formal side. This does not replace the med9 result or the final campaign.

## N6 to N8

| Item | Status | Basis |
|---|---|---|
| N6 reach shrinks with the water | **Closed** | `acoustic_gate.py:203-204,219`: the hold rests on the fix time. [O] Real `GatePerception`: after the current stops, `adrift={1}`, BUSY, `entry_ok=False`. S12 with falling water 0/1500 missed; S12m 341/500 |
| N7 deaf receiver | **Closed** | [O] Everyone silent 60 s more: `deaf=True`, hold kept, `entry_ok=False`. A deaf receiver starts no new hold. One new fix or a fresh GP_NONE ends it. Declared in `GUARANTEES.md` |
| N8 passive since the last fix | **Closed by declaration** | Stated in the premises of the row |

## Findings

**N9, MAJOR. The `past_centre` latch has no way out.** `control/flows.py:338-339`.
One control step with `go.s > 0` turns off the stage-2 hold and the give-up for the rest of the episode, wherever the drone then is.
* [O] Real `Flows.gate`, stage 2 refused throughout: hold point, then `s=+0.05` for one step; back at the hold point: no hold, `v_along=+0.5`. Same at `s=-3.0`, `-2.2` and `-0.9`. After a hold timeout, behind the give-up plane: `abort_request=False`, no hold, `v_along=+0.5`.
* Scenario: the estimate is corrected back, or the avoidance layer pushes the drone out of the entry side. It then drives through the whole CR with `entry_ok` false, which is the barrier that `s981` and `s986` were about.
* Reach is narrow. [O] 41 gate runs of med6, med7 and med8 (22 784 committed steps): the condition occurs once, in `s986`, and correctly (estimate 0.01, truth 0.18). The largest one-step jump of the estimate is 0.35 m, and a spurious latch from the hold point needs 1.18 m. The push-out path is not seen in these logs. I rate it MAJOR, not BLOCKER, but I would not tag with it: the fix is one line, and this is the last barrier.
* Resolve: `self.past_centre = go.s > 0.0 or (self.past_centre and go.s > -G.cr_half_len)`. [O] Patched in memory: the RC11 sequence still goes on to the exit, the dip to -0.1 stays decided, and a one-step spike releases the latch at the hold point. Add that case to `test_reloc_hold.py`.

**N10, MINOR. The threshold against the declared error.** `flows.py:338`. With `nav_err_m = 1.0`, `go.s > 0` allows a true `s` down to -1.0, up to 0.2 m short of the CR [I]. `go.s > G.nav_err_m - G.cr_half_len` would be sound against the declared envelope. Observed along-axis error near the gate is below that (0.41-0.55 m in earlier rounds).

**N11, MINOR. Rule 2 is neither declared nor logged.** `docs/GUARANTEES.md:40,41` do not mention an exception to stage 2 or to the give-up. No event or field records it, and `referee/gate_events.py:17-18` counts E4 only on ADMIT, so a crossing by this rule leaves no trace except `entry` empty with `s > 0`. Add an event, so the final campaign can say how often it fired.

**N12, MINOR. The report after such a crossing.** [O] Real `_update_gate_report`: GP_COMMITTED up to the exit, which is right; then (1, GP_NONE), because the verifier never ran. Release then rests on estimated `s > 3.5`, with 1.3 m of margin under the declared error [I]. A drone that crossed this way and comes back into the CR reports (1, GP_NONE) [O], since `near_done` in `passage.gate_report` needs a verified passage, so `GUARANTEES.md:81` does not hold on this branch. Declare it, or let `near_done` use the gate the drone was committed to.

## The four questions

* **(a)** Closed, as above.
* **(b)** Rule 2 is right for a drone truly past the centre: it keeps its claim, `abort_request` stays false, and M3's behaviours are only reduced [I]. The problems are the latch (N9), the threshold (N10), and the unverified release and return (N12). A latch on a transient estimate is possible in the code and absent from the logs.
* **(c)** No formal assumption changes. `heading_tol_deg` is a guard in `at_queue` (`gate_perception.py:152`), so S6 and M1/M2 rest on the guard, not on the controller. [O] S6: 0 violations. The yaw clip is unchanged at 0.12. [O] Real `LowLevelController` on first-order yaw plants (gain 2-8 rad/s per unit, time constant 0.5-3 s, both assumed, no vehicle calibration of yaw): no oscillation; 15 deg step overshoots 0.9-1.7 deg; a bias of 0.02 peaks at 4.6-5.4 deg and is removed. With the integrator at its bound against the step, 28 s to reach 6 deg. Routh: stable for any time constant below 30 s [I]. The RC10 worst case (16 deg, bias 0.056) slightly exceeds the integrator bound (0.054), leaving about 0.7 deg [I].
* **(d)** Not yet. Needed: N9; `formal/results` generated at the tag and committed (the committed files are dated 23:03 from `d2ca747` and have no S12 row; the working tree is regenerated but uncommitted); a clean tree. N10 to N12 can be declared rather than changed. Nothing in the referee or criteria changed.

## Verified and not verified

[O] 73 tests pass (`test_adrift`, `test_lowlevel_heading`, `test_reloc_hold`, `test_status_payload`, `test_b2_isolation`, `test_terminal_and_scope`). [O] S1, S4, S6, S6b, S11, S12, S12m on the real classes at this commit.
Not verified: any simulator run; med9; Rule 2 and Rule 3 in closed loop; the Z3 rows at this commit (no solver model changed in this diff); yaw dynamics of the vehicle.
