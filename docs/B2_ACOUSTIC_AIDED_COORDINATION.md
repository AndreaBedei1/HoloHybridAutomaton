# Variant B2: acoustic-aided coordination (one-way range-bearing beacon)

B0 is the communication-free baseline: the same decentralised hybrid automaton on every drone, driven by the shared
mission plan and local sensing (six single-beam sonars, DVL, IMU, compass, depth).  B2 keeps that automaton, its
guards and thresholds unchanged and adds one input to the PERCEPTION: a one-way acoustic ping carrying only a static
identity, from which the receiver itself derives range and bearing.  B1 (presence only, rule R1) is documented in
`SENSOR_ROBUSTNESS_AND_BEACONING.md` and is not part of B2.

## 1. What the simulator really provides

Read in the patched engine source (`engine/Source/Holodeck/Sensors/Private/AcousticBeaconSensor.cpp`) and checked
against the probe `results/robustness/probe/beacon_api_probe.json`:

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
risk, measured by `scripts/analyze_b2_channel.py` (`results/b2/channel_risk.json`).

Residual risk (`scripts/analyze_b2_channel.py`, 20 000 trials per level, an occupant moving inside the CR observed
from a random queue point, half of the trials with the direct path PERSISTENTLY blocked by a non-thin obstacle so
that every received ping is a reflection - a stress beyond the gate scenarios, where only hulls and bars, which
shadow and do not block, lie on the rays).  A wrong "outside the CR" decision needs consecutive accepted
reflections: with `exit_confirm` = 2 it occurred 9 / 58 / 129 times in ~65 000 receptions (CLEAN / LOW / MEDIUM),
with 3 (the value adopted, before any B2 simulation) 0 / 3 / 3; a wrong "beyond the exit face" decision, which (A3)
requires, never occurred (0 in ~200 000 receptions).  A false release would need, in addition, two possible
occupants at once (a drone entering while the previous one is still within reach of the exit face).

Precedence for N = 4-6 is not changed by B2.  The abreast queue keeps its formal M2 guarantees (check_mutex); the
compact layouts of `results/gate/queue_layouts.json` that would fail precedence for N = 5, 6 are not used, and the
beacon is not a remedy for them: identities never enter the precedence decision.

## 5. Formation (P3)

* association: a sonar echo is associated with a neighbour where its fix places it (pattern and range gate around
  the fix) instead of where the template expects it; the residual stays the one against the template;
* rule A-P (presence): a fresh fix within e_lost of the expected position counts as a sighting (no false MISSING
  for a drone hidden from the sonar); it never confirms a MISSING neighbour back and never clears a vacancy;
* rule A-D (displacement): two consecutive accepted fixes whose whole region lies farther than e_lost + 1 m from the
  expected position declare the neighbour MISSING at once and block its sonar re-confirmation while they last;
* relative formation error: the full fix-minus-template residual replaces the range residual in the along-track /
  lateral consensus (same gains, same leak and caps): the along-track offset of an abreast neighbour, unobservable
  from ranges, becomes observable.

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

Same scenarios and seeds (0, 1, 2) for B0 and B2; protocol label `B2_<channel level>`:

* sensors CLEAN, LOW-{ONE, SUBSET, ALL}, MEDIUM-{ONE, SUBSET, ALL}: B0, and B2 with the channel at the sensors' level;
* the channel alone: sensors CLEAN, channel LOW, MEDIUM, BLACKOUT;
* channel BLACKOUT with sensors MEDIUM-ALL (B2 must fall back to B0);
* one deadline for both protocols: from the B0 CLEAN runs (referee/terminal.py).
