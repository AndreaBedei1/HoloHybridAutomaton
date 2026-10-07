# Phase 1 - single directional sonar probe (HoloOcean 2.3.0 `SinglebeamSonar`)

**Verdict: the realistic sensor works.** One HoloOcean `SinglebeamSonar` mounted on the bow of a
BlueROV2 detects:

- another BlueROV2;
- the gates of the Marine Race Arena;
- runtime-spawned props.

Its range error is at most one 5 cm bin. It has one HoloOcean-specific pitfall, the octree cache, which
explains the v1 symptom ("the imaging sonar does not see the gates"). The pitfall has a clean fix,
measured below.

Scripts:
- probe: `probe/probe_singlebeam.py` (sessions `k1`, `k2`, `k3`, `k2_short`);
- analysis: `probe/analyze_sonar_probe.py`.

Outputs:
- figures: `figures/v2/sonar_probe/`;
- numbers: `results/v2/sonar_probe/summary.json`;
- raw sensor outputs: `results/v2/sonar_probe/raw_examples.json`.

Ground truth is used freely here: this is a calibration bench, not a controller.

## Sensor under test

| parameter | value |
|---|---|
| type | `SinglebeamSonar` (echosounder: one conical beam, echo intensity per range bin) |
| mounting | `location` [0.24, 0, 0] m from the BlueROV2 origin (bow), boresight = body x |
| beam | `OpeningAngle` 120 deg (cone half-angle 60 deg); a 30 deg run checks that the parameter is honoured |
| range | 0.3 - 12 m, 234 bins of 5 cm |
| rate | 10 Hz (one capture every 3 ticks at 30 ticks/s) |
| noise (noise runs) | `AddSigma` 0.05 (Rayleigh, on intensity), `MultSigma` 0.1 (normal, on intensity), `RangeSigma` 0.05 m (exponential, per octree leaf) |
| octree | `octree_min` 0.06 m, `octree_max` 5 m: a cache folder used only by this project (`.../Octrees/OpenWater/min6_max768`) |

## Answers to the probe questions

**A. Another BlueROV2: yes.**
- Both aspects tested (head-on and broadside) are detected in 100 % of captures from 0.6 m to 11 m
  centre distance.
- Nothing is detected beyond `RangeMax`, at 12.5 m.
- Agents are always visible because each sonar builds a moving octree for every other agent.

**B. An arena gate: yes, if the gates exist before the sonar is created.**
- Gate G06 of Horseshoe Bay, aimed at the left pillar, the top bar and the opening centre from 1.5 m to
  8 m: 100 % detection (session k2).
- With the sonar created first (session k1, the v1 order): 0 % at every pose.

**C. Runtime-spawned objects: same rule as B.**
- A 0.5 m steel box is detected at 1-10 m (k2) and never in k1.

**D. Output.**
- A float array of 234 range bins with echo intensity in [0, ~0.95].
- There is **no bearing information inside the cone**. The first bin above threshold is the nearest
  obstacle anywhere in the 120 deg cone. Objects at different ranges appear as separate peaks.
- Echogram: `echogram_drone_sweep.png`.

**E. Range: 12 m.**
- Every target is detected up to the configured `RangeMax`.
- Range accuracy with noise off, error = estimate - true distance to the nearest surface
  (`range_true_vs_sonar.png`):

| target | error [m] |
|---|---|
| BlueROV2 head-on | +0.004 (>= 1.75 m) ... +0.054 (<= 1.3 m) |
| BlueROV2 broadside | +0.013 |
| steel box | +0.025 |
| gate pillar | -0.015 |
| gate top bar | +0.035 |
| gate, aimed at the opening | -0.037 ... -0.001 |

All errors are within about one bin plus one octree leaf. The probe's own echoes also give the hull
extents: 0.225 m ahead and 0.275 m abeam of the agent origin.

**F. Beam width: as configured.**
- With `OpeningAngle` 120 deg, a BlueROV2 is detected up to 60 deg off-axis, both horizontally and
  vertically. At 3 m the limit is 62.5 deg, because of the hull's angular size.
- With 30 deg, the limit is 17.5 deg (15 deg half-angle plus the hull). See `fov.png`.

**G. Near the surface: the water surface is invisible to the sonar.**
- Looking up from 2, 4 and 8 m depth: no echo.
- A horizontal beam 0.54 m below the surface: no echo.
- A drone 4 m ahead near the surface is detected normally.
- Cause: the surface is not collision geometry, so it is not in the octree.
- Physically this is optimistic, because a real surface is a strong reflector and causes multipath.
  For our safety logic it is benign: no false alarms.

**H. Near the seabed: clutter in a wide horizontal beam.**
- Around the arena the OpenWater seabed lies at z ≈ -295 m (vegetated terrain). Missions fly at
  4-5 m depth, so it is never in range.
- When forced next to it:
  - a down-looking beam measures the altitude;
  - a horizontal 120 deg beam at 0.5, 1 and 2 m altitude returns the seabed at 0.63, 1.33 and 2.68 m;
    the cone grazes the bottom at about h / sin 60 deg;
  - that seabed echo masks, as first return, a drone 4 m ahead.

**I. Noise and dropout.**
- With the noise above, the intensity floor in empty water has p99 = 0.15.
- False alarms per capture: 98 % at threshold 0.15, 6.7 % at 0.20, and **0 % at >= 0.25**
  (60 empty captures).
- At threshold 0.25, a BlueROV2 at 1, 2, 4 and 8 m is detected in **100 %** of 40 captures each.
  The range error is a stable +0.054 m: the first bin above 0.25 is one bin behind the first echo.
  This is on the unsafe side (the target looks farther away) and is accounted for in the P1 envelope.
- HoloOcean's native noise produced no dropout. The 6-sonar suite will add a simulator-side beam-dropout
  model, identical for every sonar, for the stress demos (as v1 did for its rings).
- See `noise_dropout.png`.

**J. Rate and cost.**
- At `Hz` 10 the sonar appears in the state every 3rd tick, so the data age seen by a 10 Hz controller
  is at most 0.1 s.
- Cost of one warm 120 deg sonar at 30 ticks/s:

| setting | ms per tick | real-time factor |
|---|---|---|
| no sonar | 33.8 | 0.99 |
| one sonar, open water | 35.0 | 0.95 |
| one sonar, facing the dense arena gates | 40.0 | 0.83 |

- That is about **4 ms per capture in open water and about 19 ms facing the gates**. The gates are
  built from 1488 micro-blocks.
- The first attach with an empty cache causes a single pause of about 10 s, to build the octree root of
  the whole map (13 MB).
- Extrapolation to be measured in Phase 2: 6 drones x 6 sonars at 10 Hz could cost 50-230 ms per tick.
  Possible levers: a lower sonar rate, a coarser octree, fewer spawned gates.

**K. Octree pitfalls, and why v1 could not see the gates.**
The HoloOcean engine source (`Octree.cpp`, `HolodeckSonar.cpp`) shows:

- The static octree is built by overlap tests on `ECC_WorldStatic` the first time a sonar initialises.
  That happens inside `env.reset()`, during the pre-start ticks.
- The tree is cached on disk under `Octrees/<map>/min<OctreeMin>_max<OctreeMax>/`: a `roots.json` for the
  whole map plus one file per 7.68 m cell. The cache key contains **only the map name and the octree
  sizes**.
- Agents are excluded from the static tree and get moving per-agent octrees, which is why vehicles were
  always visible.

Consequences, all measured:
- **k1:** props spawned after `reset()` are absent from `roots.json`, so they are invisible.
- **k2:** props spawned before the first sonar exists are visible.
- **k3:** a cache written with props is replayed in a later run *without* those props. The sonar then
  returns **ghost echoes** at the old gate and box positions, at the same ranges as in k2.
  See `octree_props_k1_k2_k3.png`.

## The fix (principled, no sensor substitution)

1. Spawn the static scene (arena gates, props) **before** any sonar exists. Attach the sonars afterwards
   with `agent.add_sensors(...)`. This restores the assumption the HoloOcean sonar model is built on: a
   static octree of the actual static scene.
2. Use a project-private octree size (0.06 / 5 m, so `min6_max768`). Other projects on this machine use
   other folders, which are never touched.
3. Key the cache by a **signature of the static scene** (gate layout and props). Delete the private folder
   when the signature changes. Without this step, k3 shows that ghosts appear.
4. While a sonar is attached at runtime, disable the client's 60 s engine-handshake timeout
   (`env._has_sonar = True`). Octree building can take longer than one tick.

The RangeFinder rings of v1 are therefore **not needed**: the realistic sonar can be used as intended.

## Consequences for the next phases (to confirm before building)

- **Six identical sonars, 120 deg each, on the body axes.**
  - Six cones cover the sphere without gaps only if the half-angle is at least 54.7 deg, the angle of the
    eight cube-corner directions.
  - At 120 deg the margin is 5.3 deg at those corners, and neighbouring sectors overlap by up to 30 deg.
  - Phase 2 will check this in 3D, including near-field pockets caused by mounting the sensors on the
    hull faces, against the P1 safety volume.
- **Sector-level perception.** Each sonar reports "nearest echo at r in my cone", with no bearing inside
  the cone. Consequences:
  - P1: escapes are planned away from the sector axes. A single sector guarantees an opening rate of at
    least v cos 60 deg; overlapping sectors narrow the direction.
  - The v1 formation law needed metric relative positions of the neighbours, which this sensing cannot
    give. Formation keeping moves to each drone tracking its slot of the shared formation template with
    its own navigation (DVL, compass, depth). The sonars are used for safety and for the gate protocol.
  - P2: the gate priority rule is re-expressed on sector relations (front, left, up) instead of metric
    gate-frame offsets.
- **Structure echoes.** The sonars see the gates, so perception must separate structure echoes from
  vehicles. It does this with the known gate map: echoes explained by the map are structures, and
  unexplained echoes are vehicle candidates.
