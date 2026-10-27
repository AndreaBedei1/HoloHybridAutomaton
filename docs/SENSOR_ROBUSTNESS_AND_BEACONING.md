# Sensor robustness and one-way presence beaconing

Status (2026-10-23): the pre-freeze baseline campaign is complete (isolation, CLEAN/LOW/MEDIUM/HIGH; every
invalid run repeated), its failure cases are diagnosed (no real bug, so no protocol fix), the B1 rule is fixed
(rule R1, section 3) and the implementation is frozen; the official campaign (B0 and B1 on one frozen commit)
is in progress.  Results will be in `results/robustness/SUMMARY.md` and `results/beaconing/SUMMARY.md`.

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
`probe/probe_beacon_api.py` measure what the HoloOcean parameters actually do.  Measured on the patched engine
(restored 2026-10-22 from the same build, SHA-256 a5e9aa1f...fa028; results in `results/robustness/probe/`); a
first pass on the unpatched engine gave the same picture:

* `AddSigma` is the Rayleigh scale of the open-water intensity (fitted 0.0496 / 0.0698 / 0.0804 / 0.0895 / 0.110
  for 0.05 / 0.07 / 0.08 / 0.09 / 0.11); with the 0.30 threshold the fraction of captures with a false segment
  (front / rear sonar) grows steeply: 0 / 0 at 0.05, 0.3 / 0.5 % at 0.07, 2.8 / 7.2 % at 0.08, 14 / 35 % at 0.09,
  59 / 91 % at 0.11 (the rear sonar sees a longer open-water span before its target);
* `RangeSigma` (exponential per octree leaf, up to 0.40) and `MultSigma` (up to 0.60) move the leading edge of
  a hull echo by at most one or two 5 cm bins (p95 0.044 m, max 0.094 m): the first bin is the minimum over many
  leaves.  HoloOcean alone cannot stress `eps_range_far`, hence the wrapper range jitter;
* target peaks are 0.9-1.3 at 3 and 6 m (no range attenuation), so a fade gain of 0.2 removes a return;
* DVL `VelSigma` sigma gives a body-axis std of about 1.85 sigma (x, y) and 0.54 sigma (z); IMU and depth sigmas
  map one to one; magnetometer `Sigma` sigma gives a heading std of about 58 sigma degrees.
* The beacon probe confirms the facts of section 3 (range limit, line of sight blocked by a hull, result in the
  tick of the ping or the next, azimuth only with OWAYU).

The probe does not contradict the anchors, so the levels below are unchanged.

### Realistic operating domain: CLEAN, LOW, MEDIUM

The validation uses only CLEAN (the deployed sensors), LOW and MEDIUM.  Each value is tied to a datasheet or a paper
of [SENSOR_ROBUSTNESS_LITERATURE.md](SENSOR_ROBUSTNESS_LITERATURE.md) (ids A1-A16, S1-S3) through the probe mapping
above (HoloOcean parameter -> physical quantity).  A source rarely gives our sigma; it gives a range, an accuracy, a
resolution or a failure mode, and the level stays inside it.  Perturbations without measured statistics for this
hardware (sonar capture dropout and blackout, DVL outage, latency) are **conservative operational perturbations**,
not measured accuracy.

| Sensor | Disturbance | Literature / datasheet evidence | Reported range | LOW | MEDIUM | Justification |
|---|---|---|---|---|---|---|
| Single-beam sonar | range error of the leading edge | Ping sonar range resolution 0.5 % of range (A11) | 1.5-3 cm at 3-6 m | jitter sigma 1.5 cm | jitter sigma 3 cm | 0.5 % at 3 m (LOW) and at 6 m (MEDIUM); HoloOcean RangeSigma 0.05 / 0.10 moves the edge by at most one or two 5 cm bins (probe) |
| Single-beam sonar | spurious echoes (intensity noise) | false alarms coupled to the detection threshold (A16); multipath / outliers in acoustic ranging, 10 % in a published simulation setting (A12), > 25 % in field LBL data (A13) | up to tens of % for ranging | AddSigma 0.06 (~0.1 % of captures with a false segment, probe) | AddSigma 0.075 (~1-3 %) | well below the outlier rates reported for acoustic ranging; isolated false echoes are filtered by the 2-of-3 confirmation |
| Single-beam sonar | missed return (fade) | "no detection, creating gaps" (A12, 10 % as a simulation setting) | 0-10 % | 1 % | 3 % | below the published stress setting |
| Single-beam sonar | capture not delivered / short blackout | none for this hardware | - | dropout 0.5 % | dropout 1 %, 0.15 s blackout every ~60 s | conservative operational perturbation (link / processing hiccup); a blackout exceeds tau_max and exercises the FAILSAFE path |
| DVL | velocity noise | Nortek single-ping std 0.8 cm/s (A2); A50 long-term accuracy +-1.01 % standard, +-0.1 % performance (A1) | ~1 cm/s; 0.1-1 % | VelSigma 0.01 = CLEAN (body std ~1.9 cm/s, probe) | VelSigma 0.015 (~2.8 cm/s) | CLEAN is already twice the single-ping figure; MEDIUM stays within a few cm/s at 0.3-0.5 m/s survey speed |
| DVL | bottom-lock loss | lock is a first-class DVL state (A1); no velocity at all when lock is lost (A3); DVL denial in field operations (A4) | occurs, no rate published | 1 % sample dropout | 3 % dropout + 1 s outage every ~60 s | outage length and rate: conservative operational perturbation |
| DVL | altitude noise | - | - | RangeSigma 0.02 = CLEAN | 0.05 | feeds the seabed classifier only |
| IMU (ICM-20602) | gyro / accelerometer white noise | 0.004 deg/s/sqrt(Hz), 100 ug/sqrt(Hz) (A6) | ~0.016 deg/s, ~4 mm/s^2 at 15 Hz bandwidth | = CLEAN (0.005 rad/s, 0.01 m/s^2) | 0.01 rad/s, 0.02 m/s^2 | CLEAN is already above the datasheet white noise (thruster vibration); MEDIUM doubles it |
| IMU (ICM-20602) | gyro bias | zero-rate output tolerance +-1 deg/s (A6; read from a partly scrambled table extraction, to re-check in the PDF); white noise + random-walk bias structure (A7) | +-1 deg/s | random walk 0.0001 rad/s per sample (~0.3 deg/s spread after 100 s) | 0.0003 (~0.9 deg/s after 100 s) | the spread over a run stays within the datasheet tolerance |
| Compass / AHRS | heading noise and bias | ~1 deg RMS after careful calibration, "several degrees" otherwise (A8); 6.21 -> 0.55 deg and 34 -> < 3.4 deg after calibration (A9) | sub-degree to a few degrees calibrated | sigma 0.01 (~0.6 deg) + constant 0.5 deg bias | sigma 0.02 (~1.2 deg) + 2 deg bias (total ~2.3 deg) | LOW: well calibrated; MEDIUM: an imperfect calibration within the calibrated literature range |
| Depth (Bar30) | depth noise | resolution 2 mm; relative accuracy of the class +-2 cm (A10) | mm - cm | 0.02 m = CLEAN | 0.03 m | CLEAN already 10x the resolution; MEDIUM at the cm level of the relative accuracy |
| All onboard streams | latency | acoustic position fixes 0.1 s to seconds (A14, S1) - a different, slower modality | - | 20 ms | 50 ms | onboard sensors on a wired bus: conservative operational perturbation, kept small |

What the levels mean for the protocol: at LOW every anchor of the verified design holds (sonar data never older than
tau_max = 0.25 s, range errors within eps_range, navigation drift small); MEDIUM adds rare excursions beyond tau_max
(the short blackout, the DVL outage: the drone enters FAILSAFE as designed) and a heading bias whose dead-reckoning
drift reaches ~0.5 m on the approach to a gate (this is what the structure aiding of section 3b bounds).

Scope: the degraded profile is applied to ONE drone, to a SUBSET (half of the fleet) or to ALL drones; the others
keep the deployed sensors.  ONE is chosen deterministically from the committed CLEAN reference run: the healthy drone
that comes closest to another vehicle or to a gate bar (its sensing matters most); SUBSET is ONE and the next healthy
drones by index up to ceil(n/2) (`scope_drones` in `scripts/run_sensor_robustness_campaign.py`; the choice per
scenario is listed in `results/validation/noise_profiles.json`).

Exploratory stress levels.  The first campaign (2026-10-22/25) used stronger levels (`STRESS_V1` in
`sim/sensor_noise.py`: DVL 0.02 / 0.04 per beam, sonar dropout 2-5 %, blackouts every 30 s, HIGH beyond every
anchor).  They are kept only as an out-of-envelope stress test (section 7) and are not used to validate the protocol.

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

Classes: INVALID (simulator / infrastructure: excluded from every scientific denominator and run again),
SAFETY_FAILURE (P1 or P2 violated, a contact with a drone, the gate or the non-cooperative vehicle, or that
vehicle closer than d_safe), LIVENESS_FAILURE (safe, a required outcome missing without a conservative
declaration), SAFE_DEGRADED (safe, a slot falsely declared vacant, or a required outcome missing while healthy
drones entered FAILSAFE on a sensor fault), PASS (every outcome of the scenario's committed CLEAN reference run
holds again: P3 recovered or P3-deg, every healthy drone through the gate, end within 3 m of the reference end
positions, no healthy drone in FAILSAFE at the end).  The 16 committed reference runs classify PASS against
themselves.  P1 in the tables is the property among the drones; the clearance from the non-cooperative vehicle
of `p1_vertical_escape` is reported apart (a contact with it is still a safety failure).

Every run that is not PASS gets a category: A real bug, B assumption exceeded, C conservative false positive,
D liveness issue (e.g. the finite horizon of a run), E simulation artifact.  The categories come from rules on the
run's evidence (`diagnose` in `scripts/analyze_sensor_robustness.py`: FAILSAFE entries on sensor faults and the
longest sonar gap, navigation error, false dynamic associations, the isolated factor); a case the rules do not
explain is marked "?" and analysed by hand (`results/robustness/diagnoses.json` overrides the rules).

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
presence.  General rules: evidence of presence may only delay a declaration of absence or make a decision more
conservative; absence of receptions means nothing; the beacon never sets `occ_busy = False`, never sets
`has_prio = True`, never clears a collision risk.  With the beacon disabled (default, B0) the receptions are
ignored and the automaton, guards and abstraction are unchanged.

**Rule R1, the only onboard use** (`holo_fleet/perception/perception.py`, `track_missing`), chosen after the
baseline campaign: a neighbour whose ping was received within `ttl_s` is declared MISSING after
`NEIGHBOUR_MEMORY_S + missing_grace_s` = 1.0 + 2.0 s without a sonar match, instead of 1.0 s.

* Why: in the baseline most false MISSING declarations follow short sonar gaps (dropouts, fades, a hull or a
  bar shadowing the neighbour for one to three seconds) of a drone that is still in its slot.  A recent ping says
  that the neighbour is alive and within acoustic range: evidence, not proof, that a short gap is not a
  departure.
* What it cannot do: R1 defers a declaration by at most `missing_grace_s`; it never clears a MISSING state
  (only a sonar confirmation near the slot does), never touches the vacancy timer of a neighbour already missing,
  and has no input into `d_min`, `sense_ok`, `occ_busy`, `has_prio` or `passed`.  So it cannot authorize a
  passage, free a critical region, declare a safe separation or override a safety mode.  Tests:
  `tests/test_beacon_presence.py` (deferral bounded by the grace, silence means nothing, expiry after the TTL,
  no clearing, identical safety and gate inputs with every neighbour heard or none, and a structural check that
  no safety, gate or classifier module reads the beacon).
* Bounded cost: a lost drone that keeps pinging within range is declared missing at most 2 s later, and its slot
  vacant at most 2 s later (`t_rejoin` 30 s).  The referee measures it ("stale presence": a deferral while the
  neighbour was really away from its slot, beyond `e_lost`).
* Rejected: R2, holding the vacancy timer while a neighbour is heard (a ping cannot tell a drone near its slot
  from a disabled drone still within acoustic range, so R2 would block the lost-drone timeout for as long as the
  disabled drone pings); any use in occupancy, priority or separation (presence is not position).

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
