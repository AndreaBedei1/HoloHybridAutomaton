# HoloHybridAutomaton: decentralized coordination of BlueROV2 fleets with local hybrid automata

Final technical report.  It describes the system as implemented at the campaign commit, what it assumes, how it is
verified and evaluated, and what the evaluation shows.  Three kinds of statement are kept apart throughout:
**formal results** (SMT proofs, bounded model checking, exhaustive numeric checks, randomized checks; `formal/`,
`docs/GUARANTEES.md`), **simulation results** (HoloOcean 2.3.0 campaign; section 14 onwards) and **assumptions**
supported, or not, by external literature (`docs/PARAMETER_PROVENANCE.md`).  Nothing here is a real-world guarantee:
every experimental number comes from simulation, and every formal result holds for the encoded model under its
stated assumptions.

## 1. Architecture and execution pipeline

Each drone runs its own copy of the same local hybrid automaton.  There is no leader.  A drone uses its own sensors
(six single-beam sonars, DVL, IMU, compass, depth) and a mission plan shared before the dive (survey path, formation
clock, formation template, gate map).  Per control step (0.1 s):

```
sensors -> perception (echo classification, targets, conservative distance, formation and gate observation)
        -> abstract observation -> local hybrid automaton (one mode) -> mode controller -> thrusters
```

The fleet is the parallel composition of identical automata, coupled only through what each drone senses.  The
referee and the visualisation read simulator ground truth for evaluation only; the controllers never do (static
checks in `tests/test_isolation.py`).  A run is one Python process driving one HoloOcean instance; the campaign
orchestrator (`scripts/run_sensor_robustness_campaign.py`) runs isolated instances in parallel at low priority and
classifies every run (`holo_fleet/referee/robustness_metrics.py`).

The simulator is HoloOcean 2.3.0 with a patch that rebuilds the sonar octree when objects move
(`docs/HOLOOCEAN_OCTREE_PATCH.md`, patch commit a8d3070, engine binary sha256 a5e9aa1f...).  Without it, props
placed in the scene after start-up (the gate, a test box) were invisible to the sonar.

## 2. Variants B0 and B2

**B0** is communication-free: no message of any kind is exchanged at run time (`comms_enabled = False`, checked by
tests).

**B2** adds a strictly one-way acoustic identity beacon.  Every drone emits, at pre-assigned TDMA instants (period
1 s), a coded ping that carries its identity and nothing else: no position, velocity, heading, automaton state,
priority or gate occupancy, no handshake, acknowledgement or request-response.  Each receiver measures the one-way
travel time (range, with synchronised clocks) and the direction of arrival with a small array (inverted USBL).
The acoustic measurements enter the perception only: they never reach the separation distance, collision avoidance,
the gate precedence or the sonar exit witness (static check B2-S, `tests/test_b2_isolation.py`).  With a silent
channel (BLACKOUT), B2 is B0.

## 3. The local hybrid automaton

Ten modes: `FORMATION_FOLLOW`, `FORMATION_WAIT_REJOIN`, `DEGRADED_FORMATION`, `FORMATION_RECOVERY`, `MUTEX_APPROACH`,
`MUTEX_YIELD`, `MUTEX_PASS`, `SEPARATION_WARNING`, `COLLISION_AVOIDANCE`, `FAILSAFE_HOLD_OR_RETREAT`
(`holo_fleet/ha/spec.py`).  The priority order FAILSAFE > COLLISION_AVOIDANCE > SEPARATION_WARNING > MUTEX >
FORMATION_RECOVERY > formation keeping is encoded in the guards: a lower-priority edge fires only when every
higher-priority guard is false.  Determinism, completeness of the guards, the priority hierarchy and fault ->
FAILSAFE from every mode are checked by 93 SMT queries; the perception-to-automaton observation invariants by 31
more.  Separation thresholds: warning at 2.4 m (exit 2.7 m), collision avoidance at 1.7 m (exit 2.0 m), safety
distance 1.0 m, nominal spacing 3.5 m.

## 4. Perception and navigation

Navigation is dead reckoning from the DVL velocity rotated by the compass heading, with depth from the pressure
sensor.  Near a mapped gate the estimate is corrected by fitting the bar echoes of several sonar sectors to the gate
map (`perception/structure_aiding.py`, section 9).  Echoes are classified per sector as STRUCTURE (explained by the
gate map from the estimated pose), SEABED, DYNAMIC or UNKNOWN; obstacle echoes of adjacent sectors at similar ranges
form targets.  A guard uses only data younger than tau_max = 0.25 s; the nearest obstacle echo of every healthy
sector is held for tau_max (EchoHold), so that one missed capture becomes a stale reading with a staleness allowance
rather than a gap.

## 5. Sonar geometry and target association

**Conservative distance.**  A range-only sonar with a 120 deg cone does not give the bearing of an echo, and two
neighbours 90 deg apart at the same range merge into one target.  The onboard distance of a target is the
association-free touch-set bound of `perception/distance_bound.py`: for every set of sectors that one hull could
touch, a lower bound on the distance of a hull touching exactly those sectors is computed from precomputed direction
tables (48 000-direction lattice, covering radius 0.0145 rad), and the bound of a target is the minimum over the sets
compatible with its echoes.  No assumption is made on which echo belongs to which object.  The S0 check found no
scene, among about 35 000 single and multi-hull scenes with random detection boundaries in 60 deg +- one octree leaf
and echo errors up to the declared limits, in which the bound exceeds the true distance; the version that assumed the
hull centre inside every member cone is caught overestimating (S0m).  In the formation geometry the bound of a merged
pair of square neighbours rose from 2.43 m to 2.92 m, outside the warning band, which removed the repeated false
warnings diagnosed in the previous campaign.

**Neighbour association.**  The formation check matches expected neighbours (template) to echoes, one-to-one per
(target, sector).  A match is not an identification.  An echo explains a present neighbour before a missing or
vacant one.  A match counts toward bringing a missing neighbour back, resetting its absence timer or re-including a
vacant slot only when every other required, non-vacant neighbour is accounted for at the same step (matched by its own
echo, or, in B2, located by a fresh accepted acoustic fix), so that no other drone can be the echo.  Otherwise the
uncertainty is kept: the match still counts as a sighting, but it identifies nothing.  In B2 an echo where a heard
drone's fix places it belongs to that drone and is never matched to a drone that is not heard.  On the 32
re-occupations of the archived campaign, this rule blocks 8 of the 12 false ones and leaves 19 of the 20 true ones
immediate (offline estimate on the logged matches).

## 6. Acoustic positioning and communication model (B2)

`docs/ACOUSTIC_CHANNEL_MODEL.md` details the model; its hardware assumptions are: TDMA identity pings without payload
(H1); clocks disciplined before the dive (one-way travel-time ranging), with the sound speed computed onboard from
temperature, depth and a known salinity (H2); a calibrated 3-D receiving array whose residual calibration bias depends
on the direction of arrival (H3); first-arrival detection with shadowing by hulls and bars, reflected arrivals on
blocked paths and persistent multipath episodes on clear paths (H4); a processing latency (H5).  The simulator turns
the engine's exact geometry into delivered measurements through four channel levels (CLEAN, LOW, MEDIUM, BLACKOUT).
A strictly one-way range-and-bearing system of this kind requires research-grade hardware (chip-scale atomic clocks,
PPS-triggered pings, a calibrated array); commercial one-way messages give a bearing only.

The onboard perception uses one declared envelope (`config.AcousticConfig`): 3 sigma of the MEDIUM white errors, the
array calibration bias and a one-sided clear-path multipath excess of up to 3 m.  Fixes whose depth is implausible or
whose motion is inconsistent with their track are rejected.  Within this envelope every compatible position lies in
the region the perception builds (randomized check B2-A1).

## 7. Parameter provenance

Every parameter is recorded in `docs/PARAMETER_PROVENANCE.md` and `config/parameter_sources.json` (123 entries)
with its value, source, derivation, validity and citation keys (`references.bib`).  Of the 123 entries, 18 are
supported by a source, 6 partially supported, 34 derived, 42 design choices and 20 stress assumptions; 3 are
contradicted by the literature: the clear-path premise of the acoustic channel, the DVL measurement age (0.05 s
against 0.1-0.2 s in the sources) and the MEDIUM sonar range jitter against the declared echo bound.  The first one
invalidated a safety argument of B2 (an acoustic exit from a too-long range), so the channel (clear-path multipath
episodes, direction-of-arrival bias) and the onboard envelope were corrected before the final campaign.  The other
two, with the remaining open issues (`structure_tol_far_m` against the MEDIUM range noise, no DVL scale factor,
tau_max without margin at MEDIUM, section 11 of that document), were not changed: the sensor profiles stay those
fixed before the earlier results, and the results are read under them.

## 8. Disturbance and degradation models

Sensor degradation (`holo_fleet/sim/sensor_noise.py`, `docs/SENSOR_DEGRADATION_REFERENCES.md`) has two parts:
HoloOcean's own sensor noise (DVL, IMU, magnetometer, depth, sonar additive / multiplicative / range noise), drawn by
the engine with an unseeded generator, and a seeded wrapper for what the engine does not model (sonar capture
dropout, blackouts, whole-profile range jitter, deep fades, DVL dropout and outages, stream latencies, compass bias).
Levels: CLEAN, LOW and MEDIUM, applied to ONE, a SUBSET or ALL of the drones.  Where no source gives a degradation
rate the value is a stress assumption and is labelled as such; no real-world calibration is claimed.  Currents are
spatial fields with gusts and vertical components (`holo_fleet/sim/currents.py`).

## 9. Gate geometry, occupancy and precedence

The gate is the arena gate G06 rebuilt with a 2.0 x 2.0 m opening, 0.25 m bars and 0.30 m depth, without reference
posts.  The critical region (CR) is the box |s| <= 0.8 m, |l| <= 1.0 m, |z| <= 1.0 m around the gate centre.

**Queue and precedence.**  Drones wait at queue points derived from the formation template (abreast columns 3.5 m
apart; slots that share a column stack vertically).  Precedence is local: left first, then top first, decided from
the sector pattern of each neighbour in the queue (`holo_fleet/ha/mutex_rule.py`); a static rank from the shared plan
is used only for ambiguous patterns and never contradicts the geometric order.  M1 (never two drones with priority)
and M2 (exactly the first of the queue order leads, no deadlock) are proved for abreast queues of 2 to 6 drones and
checked numerically for the stacked layouts (M2v).

**Validated domain.**  Every queued drone must be able to see the drone that passes leave the CR (exit witness, M4g).
This holds for queue points within 3.5 m of the axis (abreast queues of up to 3 drones and the verified stacks) and
fails for the outer drones of longer abreast queues.  The planner refuses gate passages outside this domain
(`mission.check_gate_domain`).

**Occupancy latch.**  A queued drone holds the CR BUSY from the first unexplained echo in its FRONT corridor, EXITING
on a positive exit witness with the corridor empty, FREE after a further persistence time; or FREE after t_occ_max =
25 s without corridor echoes.  That fallback assumes a healthy crossing: the BMC M3 shows mutual exclusion for an
occupant moving at least V_LO, and M3f shows that an occupant stopped inside the CR by a fault is released by it.
In B2 the fallback does not release while a fresh fix places a drone inside the CR.  In HoloOcean the sonar exit
witness from a lateral queue point is rarely obtained (the drone that passes is lost behind the frame and then beyond
the detection range), so most releases use the fallback (section 17).

**Map-aided localization.**  A committed drone enters the CR only with a gate localization in which its hull plus the
fit uncertainty lies inside the free opening.  The fit can be acquired from tan(60 deg) x (1.25 m + 1.2 m) = 4.24 m
before the gate plane, the farthest distance at which a drift is observable by two sonar sectors.  Corrections that
would place the hull beside the opening are not applied: replaying the archived MEDIUM runs, such fits were wrong by
0.4 to 1.2 m (another drone's echo taken for a bar).

## 10. Formation maintenance and recovery

Each drone tracks its own slot of the shared plan with its navigation and corrects along-track progress and lateral
spacing with the range residuals of its neighbours.  Recovery returns a displaced drone to its slot from behind along
its own lane.  Ranking-function proofs F1-F3 show the decrease of the along-track and lateral errors in the saturated
and linear bands and the invariance of the final ball, under the declared current envelope (F5).

## 11. Lost-drone and degraded-formation behaviour

A neighbour expected nearby and not seen for 1 s is MISSING.  Its absence timer runs only while the observer keeps
its own slot and the formation clock moves, and while it runs the observer slows its own formation reference (reduced
progress) so that a delayed drone can catch up.  After t_rejoin = 30 s of absence the slot is declared vacant
(BMC F6, ranking F6r): the drone continues in DEGRADED_FORMATION on its original slot, and re-includes the slot when
the drone is identified there again for t_reinclude.

## 12. Formal P1 / P2 / P3 guarantees

`docs/GUARANTEES.md` lists every claim with its evidence.  In summary:

* **P1 (safety).**  The onboard distance never exceeds the true distance (randomized S0, about 35 000 scenes); from any
  state with the onboard distance above the warning threshold the separation stays above d_safe (BMC S1a over 4 s and
  k-induction S1b); the escape opens every threat pattern (exhaustive S2); a sonar blackout of up to 0.8 s is safe
  from 2.34 m (S4).
* **P2 (safety).**  Precedence M1-M2 (proofs), queue geometry M4a-M4f and exit observability M4g inside the
  validated domain (exhaustive numeric), the occupancy latch M3 (BMC, 42 s, every queue view) with its scope M3f.
* **P3 (liveness).**  Recovery F1-F5 (proofs), mode selection F4 (proofs), bounded waiting F6 (BMC).
* **B2.**  Envelope coverage B2-A1 (randomized), decision soundness B2-A2/A3 (exhaustive numeric), isolation B2-S
  (static); the acoustic exit availability B2-M4g is reported, not guaranteed.

The formal suite runs every check, including mutations that must fail; the summary is "all checks as expected"
(`formal/results/SUMMARY.md`).

## 13. Validated operating envelope

* **P1** assumes sonar echo errors in [-0.08, +0.12] m, data younger than 0.25 s, a relative hull tilt below 10 deg
  (measured 0.03 deg), the speed and current bounds of `config.Envelope` and the 120 deg cone (a design assumption).
  CLEAN and LOW are inside these bounds.  MEDIUM exceeds the echo bound in about 1.2 % of the captures and tau_max
  during its outages: under MEDIUM, P1 is tested, not proved.
* **P2** assumes gate passages inside the validated queue domain, a healthy crossing for the t_occ_max fallback and,
  in B0, no fault of the occupant inside the CR.
* **P3** assumes the declared current envelope and sensing.  Documented limits: one displaced neighbour in the overlap
  of two adjacent cones looks like two neighbours (B0); along-track drift between abreast neighbours is unobservable
  in B0; a drone whose drift places it in front of a bar on the merge diagonal, or whose drift at the gate exceeds the
  1.2 m fit bound, cannot relocalize and holds before the gate (safe).
* **B2** assumes the hardware of section 6 and the declared envelope; under it the acoustic exit evidence is not
  available in the validated gate geometry.

<!-- RESULTS: sections 14-22 are filled from results/robustness/final (scripts/analyze_campaign.py) -->

## 14. Results under CLEAN, LOW and MEDIUM

## 15. B0 versus B2

## 16. Acoustic blackout

## 17. Gate completion, false releases and timeouts

## 18. Formation error and recovery time

## 19. Safety failures, conservative degradation and liveness failures

## 20. Statistical uncertainty and reproducibility

## 21. Fundamental observability limits

## 22. Remaining risks and unsupported operating conditions
