# Configuration

## Fleet configuration

Every threshold, gain and envelope is a field of the frozen dataclass `FleetConfig` in `holo_fleet/config.py`.
The formal suites import the same values, so the verified models and the controller use one source.

| Group | Class | Contents |
|---|---|---|
| `sep` | `SeparationThresholds` | d_nominal 3.5, d_warning 2.4, d_ca 1.7, d_safe 1.0 m and their exit thresholds |
| `env` | `Envelope` | speed, current, drift, braking and staleness bounds assumed by the proofs |
| `gate` | `GateRule` | CR extents, queue geometry and tolerances, pass path, timers, `avoidance_guard` |
| `form` | `FormationRule` | formation tolerances, recovery, rejoin and vacancy timers, `neighbour_adaptation` |
| `perc` | `PerceptionConfig` | echo classification and distance-bound parameters |
| `plant` | `PlantCalibration` | thruster limit and plant constants measured in the simulator |
| `ref` | `RefereeConfig` | evaluation thresholds (referee only) |
| `beacon` | `BeaconConfig` | B2 beacon mode, channel level, period, payload, range |
| `acoustic` | `AcousticConfig` | onboard acoustic envelope: declared errors, `latency_max_s`, freshness and silence limits |
| `failure`, `scope` | `FailureModel`, `ScopeConfig` | permanent failure model and exclusion zone of the gates |

Change a value with `dataclasses.replace`, never by mutation:

```python
import dataclasses
from holo_fleet.config import DEFAULT, BeaconConfig

cfg = dataclasses.replace(DEFAULT, beacon=BeaconConfig(mode="ACOUSTIC_RB", channel="MEDIUM"))
cfg_v11 = dataclasses.replace(DEFAULT, gate=dataclasses.replace(DEFAULT.gate, avoidance_guard=False))
```

The source of every parameter (literature, derivation, design choice or stress setting) is in
[`PARAMETER_PROVENANCE.md`](PARAMETER_PROVENANCE.md); the bibliography is `references.bib`.

## Sensor profiles

`holo_fleet/sim/sensor_noise.py`: `CLEAN`, `LOW`, `MEDIUM` (the validation levels), `HIGH` (outside the navigation
envelope), `MEDIUM_HB3`, `MEDIUM_HB4` (compass bias 3 and 4 deg). A campaign label adds the scope of the degradation:
`MEDIUM-ALL` (every vehicle), `MEDIUM-SUBSET`, `MEDIUM-ONE`.

## Acoustic channel levels

`holo_fleet/sim/acoustic_channel.py`: `CLEAN`, `LOW`, `MEDIUM`, `MEDIUM_HT` (long-tail losses), `BLACKOUT`. One-factor
variations of `LOW` are named `SW-<key><value>`, with keys `per` (period s), `lat` (latency s), `jit` (jitter s),
`loss` (random loss), `burst` (outage length s, every 90 s) and `asym` (asymmetric loss); for example `SW-lat0.35`.

## Protocols of the campaign runner

`scripts/run_sensor_robustness_campaign.py --protocols`:

| Protocol | Configuration |
|---|---|
| `B0` | no beacon (sonar and onboard navigation only) |
| `B2_<channel>` | beacon with payload on that channel level, e.g. `B2_MEDIUM` |
| `B2_SW-<spec>` | beacon on a sweep channel, e.g. `B2_SW-lat0.45` |
| `B2NP_<channel>` | beacon without payload (ablation) |
| `B2NA_<channel>` | beacon without the neighbours' speed adaptation (`FormationRule.neighbour_adaptation = False`) |
| `B0NG`, `B2NG_<channel>`, ... | the same without the gate guard (`GateRule.avoidance_guard = False`, the `b2-freeze-v11` controller) |

## Environment variables

| Variable | Default | Effect |
|---|---|---|
| `HOLO_FLEET_HOLOOCEAN_ROOT` | `F:\Andrea\holoocean_patched_root` | root of the patched HoloOcean engine |
| `MARINE_RACE_ARENA_ROOT` | `~/Desktop/HoloDroneCompetition` | Marine Race Arena used by the gate scenarios |
| `HOLO_FLEET_ENGINE_SLOTS` | 1 | concurrent HoloOcean instances allowed (1 to 4; each slot needs its private engine root, `scripts/prepare_engine_roots.ps1`) |
| `HOLO_FLEET_SLOT_WAIT_S` | 0 | seconds a run waits for a free engine slot |
| `HOLO_FLEET_HORIZON_FACTOR` | 1 | scales the deadline of the terminal criteria |
| `HOLO_FLEET_CACHE` | `~/.cache/holo_fleet` | cache of derived data |
