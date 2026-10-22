# Sensor robustness and one-way presence beaconing

Status (2026-10-22): **methodology and infrastructure only; the campaign has not been run.**  The patched
HoloOcean engine root (`F:\Andrea\holoocean_patched_root`, docs/HOLOOCEAN_OCTREE_PATCH.md) disappeared from
the machine during this session, and the campaign refuses to run on an unpatched engine (its octree differs
from the baseline: runtime props may be invisible to the sonars).  No robustness or beacon result is reported
here.  See "Next steps".

Terminology.  BASELINE: decentralized, communication-free coordination driven by shared mission constraints
and local sensing.  ENHANCED (variant B1): the same protocol augmented with periodic one-way presence beacons:
no peer-to-peer negotiation, no handshake, no bidirectional exchange, no runtime sharing of state or control
variables; one-way periodic presence broadcast only.  B1 is **not** communication-free.

Literature: verified notes in [SENSOR_ROBUSTNESS_LITERATURE.md](SENSOR_ROBUSTNESS_LITERATURE.md) (36 of 37
entries checked against a real source; unverified items are marked).

## 1. Error model

Every disturbance acts on the sensor, before a controller receives the data; ground truth (PoseSensor,
VelocitySensor, CollisionSensor) stays clean and is read by the referee only.  Code:
`holo_fleet/sim/sensor_noise.py` (profiles), `holo_fleet/sim/holo_env.py` (wrapper),
`holo_fleet/perception/sensor_suite.py` (HoloOcean configuration).

| phenomenon | mechanism | literature support |
|---|---|---|
| sonar intensity noise, false returns | HoloOcean `AddSigma` (Rayleigh scale), `MultSigma` | false-alarm / miss trade-off of thresholded low-cost sonars (A16), spurious altimeter echoes (A11) |
| sonar range uncertainty | wrapper: whole-profile shift per capture, N(0, sigma) quantised to 5 cm bins; HoloOcean `RangeSigma` | acoustic range outliers / heavy tails (A12, A13, S3) |
| missed detections | wrapper: deep fade, profile x 0.2 with probability p per capture | detection gaps in field data (A12) |
| sonar dropout / burst blackout | wrapper: per-capture dropout, whole-drone blackout windows | stress-test choice (no verified rate) |
| DVL velocity noise | HoloOcean `VelSigma` (per beam) | vendor single-ping std ~1 cm/s (A1, A2) |
| DVL dropout / outage | wrapper: per-sample dropout + outage windows | bottom-lock loss (A1-A4); durations not quantified: stress-test choice |
| DVL range noise | HoloOcean `RangeSigma` | altitude feeds the seabed classifier |
| IMU white noise / bias | HoloOcean `AccelSigma`, `AngVelSigma`, `AccelBiasSigma`, `AngVelBiasSigma` (random walk) | MEMS structure (A6, A7); intensities for our units unknown |
| heading noise / bias | HoloOcean magnetometer `Sigma`; `MagneticVector` rotated by a per-drone constant (hard-iron-like bias) | errors of 1-3 deg calibrated, more uncalibrated (A8, A9, S3) |
| depth noise | HoloOcean `DepthSensor Sigma` | least degraded channel (A10) |
| latency / staleness | wrapper: every onboard stream delivered late, stamped with its capture time | acoustic fix delays 0.1 s to seconds (A14, S1) |

The engine draws its own noise with an unseeded generator, so runs are not bit-reproducible (already known
for this project).  Every wrapper effect (dropout, fades, shifts, blackout and outage windows, heading-bias
signs, beacon jitter and dropout) is drawn from generators seeded by the scenario seed, separate from the
existing stress stream.

### Probes before the levels

`probe/probe_sensor_noise.py` (ten pinned BlueROV2 probes, targets at 3 m and 6 m) and
`probe/probe_beacon_api.py` measure what the HoloOcean parameters actually do.  A first pass ran on the
**unpatched** engine (the patched root was already missing), so its numbers are provisional and must be
re-measured; qualitatively:

* `AddSigma` is the Rayleigh scale of the open-water intensity (fitted 0.050 / 0.070 / 0.080 / 0.090 / 0.110
  for the configured values); with the 0.30 threshold the fraction of captures with a false segment grows
  steeply (about 0 at 0.05, 0.3-2 % at 0.07, 2-8 % at 0.08, 10-32 % at 0.09, 60-89 % at 0.11);
* `RangeSigma` (exponential per octree leaf, up to 0.40) and `MultSigma` (up to 0.60) move the leading edge of
  a hull echo by at most one or two 5 cm bins: the first bin is the minimum over many leaves.  HoloOcean alone
  cannot stress `eps_range_far`, hence the wrapper range jitter;
* target peaks are 0.9-1.2 at 3 and 6 m (no range attenuation), so a fade gain of 0.2 removes a return;
* DVL `VelSigma` sigma gives a body-axis std of about 1.9 sigma (x, y) and 0.55 sigma (z); IMU and depth sigmas
  map one to one; magnetometer `Sigma` sigma gives a heading std of about 57 sigma degrees.

### Profiles (simulation stress levels, not real sensor accuracy)

Anchors: sonar 10 Hz and `tau_max` = 0.25 s; `eps_range_far` 0.12 m / `eps_range_near` 0.08 m; formation
`e_ok` 0.5 m / `e_lost` 1.2 m; gate `queue_tol` 0.35 m; warning/CA thresholds 2.4 / 1.7 m.  LOW stays clearly
inside them, MEDIUM puts the high percentiles near them, HIGH exceeds them on purpose.  Values: the code above
(CLEAN = the deployed suite, byte-identical HoloOcean configuration, tested).

| parameter | CLEAN | LOW | MEDIUM | HIGH | anchor |
|---|---|---|---|---|---|
| sonar AddSigma / MultSigma / RangeSigma | 0.05 / 0.10 / 0.05 | 0.06 / 0.15 / 0.05 | 0.075 / 0.25 / 0.10 | 0.09 / 0.40 / 0.20 | threshold 0.30 |
| sonar range jitter sigma [m] | 0 | 0.015 | 0.03 | 0.07 | eps 0.08 / 0.12 |
| sonar fade probability | 0 | 0.01 | 0.03 | 0.10 | 2-of-3 confirmation |
| sonar dropout / blackout | 0 | 2 % / - | 5 % / 0.15 s per 30 s | 12 % / 0.5 s per 20 s | tau_max 0.25 s |
| latency [s] | 0 | 0 | 0.05 | 0.12 | tau_max 0.25 s |
| DVL VelSigma / RangeSigma | 0.01 / 0.02 | 0.02 / 0.05 | 0.04 / 0.15 | 0.08 / 0.40 | DVL freshness 0.35 s |
| DVL dropout / outage | 0 | 1 % | 3 % | 8 % + 1 s per 30 s | |
| IMU accel / gyro sigma | 0.01 / 0.005 | 0.02 / 0.01 | 0.05 / 0.02 | 0.10 / 0.05 | |
| IMU bias step accel / gyro | 0 | 0.0005 / 0.0002 | 0.002 / 0.0005 | 0.005 / 0.002 | |
| magnetometer sigma / heading bias [deg] | 0.005 / 0 | 0.01 / 0.5 | 0.03 / 1.5 | 0.07 / 4 | |
| depth sigma [m] | 0.02 | 0.03 | 0.06 | 0.15 | |
| beacon reception dropout (B1 only) | 0 | 5 % | 20 % | 50 % | |

Isolation: CLEAN plus one phenomenon at its HIGH level (`FACTORS`, 13 factors: sonar_intensity, sonar_range,
sonar_fading, sonar_dropout, sonar_blackout, dvl_velocity, dvl_dropout, dvl_range, imu_white, imu_bias,
heading, depth, latency); the combined case is HIGH.

## 2. Referee metrics and run classes

`holo_fleet/referee/sensor_audit.py` (online, ground truth vs what the sensors delivered and what the
perception made of it): sonar range error vs the exact distance to the target hull, echo recall, false and
missed dynamic associations, stale fraction, longest stale interval, DVL velocity error, heading error, depth
error, gyro error, navigation error.  `holo_fleet/referee/robustness_metrics.py` (offline, per run): P1 (holds,
min separation, warning/CA entries and time, contacts), P2 (holds, occupancy, entry order, queue wait, time
without a drone passing while others wait, stalls >= 20 s, false BUSY / FREE, completion), P3 (episodes,
recovery times, formation error, false missing, missing detection delay, false rejoin, vacancies, false
DEGRADED, re-inclusions, WAIT_REJOIN time), automaton (determinism and observation violations, transitions
per minute, chattering A-B-A within 1 s, FAILSAFE entries by cause and time).

Classes: INVALID (simulator / infrastructure), SAFETY_FAILURE (P1 or P2 violated or a contact),
LIVENESS_FAILURE (safe, a required outcome missing without a conservative declaration), SAFE_DEGRADED (safe,
a slot falsely declared vacant, or a required outcome missing while healthy drones entered FAILSAFE on a
sensor fault), PASS (every outcome of the scenario's committed CLEAN reference run holds again: P3 recovered
or P3-deg, every healthy drone through the gate, end within 3 m of the reference end positions, no healthy
drone in FAILSAFE at the end).  The 16 committed reference runs classify PASS against themselves.

Ground truth stays referee-only: the audit reads the controllers' perception output and never writes to it.

## 3. One-way presence beacon (B1) design

Transmitted: a ping carrying only the sender's static, pre-assigned identity (like a pinger signature), every
1.0 s +- 0.1 s (uniform jitter, seeded; random initial phase) so that pingers do not stay in step.
Not transmitted: position, velocity, heading, automaton mode, slot error, priority, queue state, sensor
readings, map information.  No handshake, ACK, request/response, leader, routing or consensus.

Transport (`holo_fleet/sim/beacon.py`): one HoloOcean `AcousticBeaconSensor` per drone at the hull centre,
`MaxDistance` 50 m, `CheckVisible` on.  HoloOcean 2.3 facts (probe): the engine returns azimuth, elevation,
distance and depth to the receiver, or -1 when out of range or when the line of sight is blocked (also by
another hull); the result appears in the tick of the ping or the next one; the client's message bookkeeping
cannot be used for a broadcast (one failed reception cancels the ping for the receivers read after it; two
concurrent senders void every reception), so the wrapper enqueues the engine command and reads the raw
buffer.  Added in the wrapper: propagation delay distance / 1500 m/s, reception dropout (profile), and a
conservative collision rule (pings whose results can share a receiver's tick are all lost there): HoloOcean
has no acoustic interference model, real receivers may separate distinct pinger codes (B7).  The engine's
azimuth/elevation are dropped (presence only; a bearing variant B2 would be a separate column).
`OpticalModemSensor`: in the probe no reception occurred between modems that do not face each other
(directional laser-modem model, 60 deg cone, default range 50 m, far beyond the < 3-10 m reported for LED
beacons, S4, B8): an optical presence beacon is kept as literature and design only.

Onboard (`holo_fleet/perception/beacon_presence.py`): receptions update "last heard" per slot (identity ->
slot is pre-mission knowledge, like the template); a slot heard within `ttl_s` = 2.5 s has evidence of
presence.  Rules: evidence of presence may only delay a declaration of absence or make a decision more
conservative; absence of receptions means nothing; the beacon never sets `occ_busy = False`, never sets
`has_prio = True`, never clears a collision risk.  The way the evidence enters the missing-neighbour logic is
deliberately **not fixed yet**: it is to be chosen from the failure modes the baseline campaign shows (a
presence ping cannot tell a drone near its slot from a disabled drone still within acoustic range, so a naive
"heard => not missing" rule would block the lost-drone timeout).  With the beacon disabled (default) the
receptions are ignored and the automaton, guards and abstraction are unchanged.

Dev smoke (unpatched engine, numbers not results): in a six-drone line, 64 % of pings were received, 12 %
lost to the collision rule and 24 % to blocked line of sight (drones in line shadow each other), latency
5-12 ms.

## 4. Campaign

`scripts/run_sensor_robustness_campaign.py` (headless, one HoloOcean instance at a time, each run in its own
process, resumable, no frames, raw data in the git-ignored `results/robustness/raw/`):

* isolation: 13 factors x {p1_close_encounter, line_parallel_mutex, lost_drone_rejoin} x seeds {0, 1, 2} = 117 runs;
* main: 16 scenarios x {CLEAN, LOW, MEDIUM} x 3 seeds = 144 runs;
* high: HIGH on the 14 listed scenarios x 3 seeds (all 16 with `--all-high`);
* beacon: B1 on 8 scenarios x {LOW, MEDIUM, HIGH} x 3 seeds = 72 runs (B0 from main/high).

Estimated wall time from a headless smoke: 50-90 s per run of 30-60 s simulated time plus 10-60 s engine start
on the shared machine; about 13-16 h for everything.

## 5. Limits (known now)

* No real hardware calibration: every level is a simulation stress level.
* HoloOcean noise is not seeded; seeds fix only the wrapper effects.
* No multipath model; false returns come only from intensity noise; range errors beyond one or two bins only
  from the wrapper jitter.
* Acoustic beacon: no interference model (conservative wrapper rule), line of sight blocked by any hull, no
  propagation physics beyond distance / sound speed.
* The formal verification covers the automaton and the observation abstraction only; sensor and beacon
  robustness are empirical.

## 6. Next steps

1. Restore the patched engine root (`F:\Andrea\holoocean_patched_root`, see docs/HOLOOCEAN_OCTREE_PATCH.md).
   The built engine is still in `F:\Andrea\holoocean-octree-patch\engine\Binaries\Win64\Holodeck.exe` and its
   SHA-256 (a5e9aa1f...fa028) equals the `exe_sha256` recorded by the committed v2 runs, so the install part of
   `build_and_install.ps1` (everything after the Build.bat call) recreates the root without a rebuild; check
   `PATCH_INFO.json` and that a run's `setup.holoocean.patched` is true.
2. Re-run both probes on the patched engine and update section 1 (levels unchanged unless the probe contradicts
   the anchors; any change recorded before the campaign).
3. Run `--phase isolation`, `--phase main`, `--phase high`; write `scripts/analyze_sensor_robustness.py`
   (summary.csv, failure_cases.json, sensor_metrics.csv, figures) and freeze the baseline results.
4. Decide the B1 rule from the observed failure modes, add its regression test, run `--phase beacon`.
