# Variant B2: acoustic-aided coordination (one-way range-bearing beacon)

B0 is the communication-free baseline: the same decentralised hybrid automaton on every drone, driven by the shared
mission plan and local sensing (six single-beam sonars, DVL, IMU, compass, depth).  B2 keeps that automaton, its
guards and thresholds unchanged (apart from the explicit give-up edge of a gate entry, ha/spec.py pass_abort) and
adds one input to the PERCEPTION: a one-way acoustic ping carrying a static identity, from which the receiver itself
derives range and bearing, and (H6, section 2b) a 56-bit packet with the sender's onboard position estimate and
error bound.

## 1. What the simulator really provides

Read in the patched engine source (`engine/Source/Holodeck/Sensors/Private/AcousticBeaconSensor.cpp`) and checked
against the probe `results/calibration/beacon_api_probe.json`:

* per (sender, receiver) command the engine writes `[azimuth, elevation, distance, depth]`, where azimuth and
  elevation are those of the SENDER's sensor position in the RECEIVER's sensor frame (`InverseTransformLocation`
  with the receiver component transform: body frame, roll and pitch included, client axes x forward, y left, z
  up) and `depth` is the RECEIVER's own depth (the probe confirms all four values to the millimetre);
* values are exact (no noise apart from an optional noise on `MaxDistance`); `-1` in every field when the distance
  exceeds `MaxDistance` or when the straight segment between the two sensors first hits another actor
  (`CheckVisible`: one ray, any hull or prop blocks it);
* the result appears after `round(d / (dt * 1500))` ticks (0 or 1 tick below 50 m); a receiver holds one incoming
  message per tick (two simultaneous senders overwrite each other).

These values are used as the geometric truth of the direct path and never handed to a controller as they are.

## 2. Hardware assumptions and measurement model (`holo_fleet/sim/acoustic_channel.py`)

| | assumption | what it implies in the model |
|---|---|---|
| H1 | static identity, fixed TDMA schedule: drone k pings in slot k of every period (1 s), slots aligned to simulator ticks | the receiver knows the emission instant without any payload; no ping collisions with n <= 6 |
| H2 | synchronised clocks (OWTT ranging, e.g. a chip-scale atomic clock disciplined before the dive) | range = c_assumed x travel time; per-drone clock offset (a pair bias), one sound-speed scale factor per run, white jitter |
| H3 | small receiving array (inverted USBL) on every drone | direction of arrival in the receiver's body frame, white azimuth / elevation noise; the receiver's own attitude error is applied onboard |
| H4 | first-arrival detection | clear path: direct arrival. Ray blocked by a drone hull or a gate bar (bodies smaller than the first Fresnel zone, 0.3-0.5 m at 20-30 kHz over 3-10 m): SHADOWED, lost with probability `shadow_loss`, else the diffracted direct arrival (<= 0.10 m longer, a little more direction noise). Ray blocked by anything else: lost, or (probability `nlos_rx`) a REFLECTED arrival: longer by 0.3-3 m, direction off by ~25 deg (a contradictory measurement) |
| H5 | processing latency | delivered `latency_s` after the arrival, stamped with the arrival time |
| - | channel | dropout growing with range, burst outages (interference), ping collisions within 10 ms (both lost) |

Channel levels (fixed before any B2 result; the onboard perception never knows the level):

| level | sigma_r [m] | sound speed | clock offset [m] | sigma_az / sigma_el [deg] | dropout near / far | outages | shadow loss | reflections | latency [s] |
|---|---|---|---|---|---|---|---|---|---|
| CLEAN | 0.05 | 0.3 % | 0.01 | 1.0 / 1.0 | 0.02 / 0.10 | none | 0.30 | 0.10 | 0.10 |
| LOW | 0.10 | 0.5 % | 0.03 | 1.5 / 1.5 | 0.08 / 0.25 | none | 0.50 | 0.20 | 0.15 |
| MEDIUM | 0.15 | 1.0 % | 0.05 | 2.5 / 2.5 | 0.20 / 0.50 | 4 s every ~40 s | 0.70 | 0.35 | 0.25 |
| BLACKOUT | nothing is ever received | | | | | | | | |

Anchors: range resolution +-0.1 m of a commercial micro-USBL modem (Blueprint SeaTrac X150 datasheet, "dependant on
provided VOS accuracy"); bearing 1 deg RMS (Tritech MicronNav 200 datasheet) to +-3 deg (older micro-USBL sheets); a
low-cost OWTT + inverted-USBL fleet at 1 Hz reported 68 % of range errors below 0.7 m and of azimuth errors below
7 deg at 30-300 m in a river (Rypkema, Schmidt and Fischell, Field Robotics 2022).  Our ranges are 2-20 m in open
water; CLEAN and LOW sit near the datasheet values, MEDIUM between them and the low-cost field figures.

## 2b. H6: one-way position payload (`holo_fleet/perception/payload.py`, `perception/nav.py`)

Added on 2026-11-04 after an executed counterexample showed that range-bearing measurements alone cannot separate an
occupied critical region from a cleared one: a drone at s = 0.8 m with a +3 m multipath excess (inside the declared
envelope) and a drone at s = 3.8 m without excess produce identical fixes forever
(`results/dev/issue3/indistinguishable_occupancy.json`).  Every ping now carries a 56-bit packet:

| field | bits | encoding |
|---|---|---|
| frame | 2 | TDMA frame counter mod 4, checked by the receiver against its own schedule; no timestamp (the slot is the emission instant, H1, H2) |
| x, y | 12 + 12 | onboard estimate in the shared mission frame, 5 cm, +-102 m (the mission area spans +-45 m) |
| depth | 8 | 5 cm, 0-12.75 m |
| bound | 6 | radius of a ball containing the sender's true position, 0.1 m steps, rounded up after the quantisation error; 63 = invalid |
| CRC-16/CCITT | 16 | a corrupted packet is discarded (CRC-8 would pass ~1/256 of multi-bit errors on a mutual-exclusion premise) |

* **One-way only.** No request, reply, acknowledgement, mode, queue position or control decision is sent.  The data
  come from the sender's own perception (`Perception.payload`), never from the simulator.
* **Bound (NavBound).** Launch survey 0.30 m [REQ] (never restarted: a gate fit's uncertainty is not an error
  bound), plus (2 sin 1.5 deg + 1 %)(1 + 1 %) x distance travelled (compass bias and DVL scale),
  plus 3.44 x 0.0285 m/s x sqrt(sum dt^2) (white MEDIUM DVL noise integrated by the dead reckoning, 99.73 % of a 2-D
  normal), plus min(1.5 m/s^2 tau, |v| + v_bound) dt while the DVL delivers no new sample for tau > 0.15 s (one period + latency) (1.5 m/s^2:
  the calibrated reversal), plus every unvalidated map correction, plus 2 x 0.25 m (DVL lever arm) and twice the
  hold + latency delay (0.1 + 0.05 s) at the speed bound; depth 0.45 m; the ball radius is hypot(horizontal, vertical).  A correction
  larger than the bound marks it invalid for the rest of the run.  It is an envelope assumption like the acoustic
  one (the HIGH profile, 4 deg compass bias, lies outside it), and every run logs it against the true error
  (`nav_bound` in the state records; referee `nav_bound` coverage and the tag NAV_BOUND_PREMISE_VIOLATED).
* **Air time (H6, unverified hardware assumption).** At 1 kbit/s net the packet takes 56 ms; the 40 ms identity chirp
  is its preamble and the propagation spread at 50 m is 33 ms: 129 ms of the 167 ms slot of n = 6 drones at the 1 s
  period, 38 ms left for reverberation tails and jitter.  The transport refuses a configuration that does not fit
  (n >= 8).  Modems with short-range rates far above 1 kbit/s exist (EvoLogics S2CM-HS, up to 62.5 kbit/s at
  120-200 kHz; Subnero M25M, 15 kbit/s peak at short range; Teledyne Benthos DAT, 140-15 360 bit/s), but none
  documents a 7-byte frame time on a synchronised TDMA schedule: this must be measured on the bench.  A BlueROV2-class
  64 bit/s modem (Water Linked M64) cannot carry it at this period; a longer period would require re-deriving every
  1 Hz rule (A_CONFIRM within fresh_s, tie_window_s, REINIT_S, the ball growth v_bound x age).
* **Packet loss.** `payload_loss` per channel level (CLEAN 0.02, LOW 0.05, MEDIUM 0.15): a STRESS knob, not a
  measured rate (the ping is detected but its packet is not decoded; independent of range and path type).  It draws
  from a separate seeded stream, so the ping and measurement draws coincide with those of an earlier run only until
  the first behavioural difference.
* **Use onboard.** A payload is attached to a fix only if its frame counter matches the schedule and some point of
  the fix's error region lies within the sender's bound + its motion since emission + the receiver's own bound + the
  region slack of the reported position; otherwise it is discarded.  The receiver then holds two regions that both
  contain the sender: the gate rules use their intersection (clear if either region is clear of the CR; a possible
  occupant only if both allow it; formal B2-P), the relocalization uses the reported balls to establish that no
  other drone can explain an echo matched to a bar, and the formation clock compares the neighbour's reported
  position with where the own formation time places its slot.
* **Unknown occupant.** A drone whose latest accepted fix placed it possibly in the CR, and that has not been
  cleared since, keeps the latch BUSY: the t_occ_max fallback never releases it, and an acoustic-only episode is
  released only by positive clearance (A2) of every drone heard possibly inside.  Liveness cost: a drone that falls
  silent right after such a fix blocks the gate for good (stated, not hidden).

## 3. Onboard fixes (`holo_fleet/perception/acoustic.py`, `config.AcousticConfig`)

A reception becomes a FIX: the relative position r u(az, el), rotated with the own heading estimate of the arrival
instant (pose history), plus an ERROR REGION from the declared envelope (MEDIUM at 3 sigma, plus the own compass
bias + noise bound 3 deg and the unestimated tilt 3 deg; clock bias 3 sqrt(2) x 0.05 m; sound speed 3 %).  The region
is sampled (7 x 7 x 7) with a slack covering the space between samples (formal B2-A1).  Rejected: receptions
delivered later than 0.4 s after arrival, fixes outside the operating depth band by more than 1.5 m (surface or
bottom bounce), and fixes kinematically inconsistent with the identity's track: 3-D displacement beyond
`v_bound dt + 3 sqrt(2) sigma_white + 0.5 m` (the biases are common to two fixes and cancel) or range change beyond
`v_bound dt + own displacement + 3 sqrt(2) sigma_r + 0.15 m` (a reflection is always longer).  A rejected fix is kept
as contradictory evidence.  Absence of fixes is never evidence.

## 4. Gate (P2): acoustic exit evidence (`holo_fleet/perception/acoustic_gate.py`)

The sonar latch is unchanged (BUSY on an unexplained FRONT corridor echo; EXITING on a sonar exit witness with the
corridor empty; FREE after t_clear; t_occ_max fallback).  B2 adds one more way into EXITING, evaluated only while the
latch is BUSY and the FRONT corridor is empty; it never touches precedence (`has_prio`), separation or collision
avoidance (formal B2-S):

* (A1) accounting at t_lc, the last corridor sighting: every other drone of the plan is EXCLUDED only if an accepted
  fix within 3 s of t_lc, its region grown by v_bound |t_fix - t_lc|, can neither be a hull in the CR (grown by the
  hull radius and by the own gate-frame navigation error, 1.0 m) nor the hull of the corridor echo, and no rejected
  fix in that window could; otherwise it is a POSSIBLE OCCUPANT (a drone without fixes always is);
* (A2) every possible occupant is positively OUTSIDE the CR now: `exit_confirm` = 3 consecutive accepted fixes on
  one track, each region clear of the grown CR, the latest still clear when grown by v_bound up to now, no
  contradictory fix since;
* (A3) at least one possible occupant is positively BEYOND the exit face (the occupant has left).

Soundness argument.  Under the B0 assumption that every entry into the CR is first seen in the FRONT corridor, a
drone in the CR now was in it at t_lc, produced the corridor echo at t_lc, or entered later visibly (a new t_lc).
(A1) leaves each such drone among the possible occupants unless its fixes, inside the envelope, place it elsewhere;
(A2) places every possible occupant outside the CR now.  The argument needs the envelope to hold for the accepted
fixes.  Formal B2-A2/A3 check exhaustively that no measurement inside the envelope of a hull in the CR passes any of
the three tests; what lies outside the envelope (Gaussian tails, reflections that pass the gates) is the residual
risk, measured by `calibration/analyze_b2_channel.py` (`results/calibration/b2_channel/channel_risk.json`).

Residual risk (`calibration/analyze_b2_channel.py`, 20 000 trials per level, an occupant moving inside the CR observed
from a random queue point, half of the trials with the direct path PERSISTENTLY blocked by a non-thin obstacle so
that every received ping is a reflection - a stress beyond the gate scenarios, where only hulls and bars, which
shadow and do not block, lie on the rays).  A wrong "outside the CR" decision needs consecutive accepted
reflections: with `exit_confirm` = 2 it occurred 9 / 58 / 129 times in ~65 000 receptions (CLEAN / LOW / MEDIUM),
with 3 (the value adopted, before any B2 simulation) 0 / 3 / 3; a wrong "beyond the exit face" decision, which (A3)
requires, never occurred (0 in ~200 000 receptions).  A false release would need, in addition, two possible
occupants at once (a drone entering while the previous one is still within reach of the exit face).

Precedence for N = 4-6 is not changed by B2.  The abreast queue keeps its formal M2 guarantees (check_mutex); the
compact layouts of `results/calibration/gate/queue_layouts.json` that would fail precedence for N = 5, 6 are not used, and the
beacon is not a remedy for them: identities never enter the precedence decision.

## 5. Formation (P3)

* association: a sonar echo is associated with a neighbour where its fix places it (pattern and range gate around
  the fix) instead of where the template expects it, and only if it also lies within the B0 range gate of the
  template distance; the residual stays the one against the template.  The fix chooses WHICH echo is the
  neighbour; it never makes a drone far from its slot count as seen there;
* rule A-P (presence): while the latest fix is fresh (`fresh_s` = 2.5 s: one lost ping plus the latency) and lies
  within e_lost of the expected position, the neighbour counts as sighted (no false MISSING for a drone hidden from
  the sonar); it never confirms a MISSING neighbour back and never clears a vacancy;
* rule A-D (displacement): two consecutive accepted fixes whose whole region lies farther than e_lost + 1 m from the
  expected position declare the neighbour MISSING at once and block its sonar re-confirmation while they last;
* relative formation error: the full fix-minus-template residual replaces the range residual in the along-track /
  lateral consensus (same gains, same leak and caps): the along-track offset of an abreast neighbour, unobservable
  from ranges, becomes observable.

Two defects of the first implementation were found by the `b2_formation` phase and corrected before any further
run (commit 27ed79c; pre-fix runs archived in `results/robustness/raw_b2_prefix_8aacabc`, tables in
`results/b2/formation_prefix`):

* the fix-guided association had no slot gate: the echo of a drone with failed thrusters, still heard and drifting
  away, stayed associated with its slot with a residual growing to 8 m, and its neighbours took the guard
  `form_err > e_lost` (FORMATION_RECOVERY, own slot lost) instead of the missing / vacancy path of B0, so the slot was
  never declared vacant (`lost_drone_timeout`, B2 CLEAN: LIVENESS_FAILURE 3/3, B0 PASS 3/3);
* rule A-P stamped the sighting at the arrival of the fix, so that the 1 s ping period against the 1 s neighbour
  memory left a gap at every lost ping or late delivery, and a neighbour heard in its slot but missed by the sonar
  was declared MISSING (more false-missing episodes than B0 in CLEAN: 9 against 3 on `formation_six`).

Neither correction changes a threshold, a guard or the automaton; both make the code match the rules above.

Without receptions (BLACKOUT) every rule is inactive and the perception is the B0 one (tests:
`tests/test_acoustic_b2.py::test_a_silent_channel_is_the_baseline`).

## 6. Guarantees: B0 versus B2

| | B0 | B2 |
|---|---|---|
| P1 separation | formal (check_separation), sonar only | identical: acoustic fixes never reach d_min, escape or CA (B2-S) |
| P2 mutual exclusion | formal M1-M4 under the sonar assumptions; M4g (sonar exit observable) holds n = 2, 3, violated n = 4-6 | B0 rules unchanged; the acoustic exit adds B2-A1..A3 (sound inside the envelope) and B2-M4g (geometric availability, reported per n); residual risk outside the envelope measured, not proved |
| P3 formation | formal ranking functions (check_formation) | unchanged automaton and guards; richer perception inputs |
| channel failure | n/a | BLACKOUT or a silent drone: every acoustic rule is inactive or refuses; behaviour = B0 |

## 7. Experimental matrix (`scripts/run_sensor_robustness_campaign.py --phase b2_final`)

Same scenarios (`FINAL_SCENARIOS`: the five gate scenarios, the formation scenarios and the two P1 scenarios) and
seeds (0, 1, 2) for B0 and B2; protocol label `B2_<channel level>`.  The matrix below was fixed after the CLEAN gate
phase and before any B2 run with degraded sensors or channel (429 runs):

* sensors CLEAN, LOW-ALL, MEDIUM-ONE, MEDIUM-SUBSET, MEDIUM-ALL: B0, and B2 with the channel at the sensors' level;
  LOW-ONE and LOW-SUBSET are left out (LOW-ALL is the most degraded LOW case; the LOW scopes are characterised for
  B0 in `SENSOR_ROBUSTNESS_AND_BEACONING.md`);
* the channel alone: sensors CLEAN with the channel MEDIUM and BLACKOUT (LOW lies between CLEAN and MEDIUM);
* channel BLACKOUT with MEDIUM-ALL sensors (B2 must fall back to B0);
* one deadline for both protocols: from the B0 CLEAN runs (referee/terminal.py).

Phases before it, in the same raw root and reused by it: `b2_gate` (five gate scenarios, CLEAN, B0 and B2_CLEAN)
and `b2_formation` (formation scenarios, CLEAN and MEDIUM-ALL, B0 and B2).
