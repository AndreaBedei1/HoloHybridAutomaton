# Run `gate_arena_comms_s0`

Scenario B: triangle survey through arena gates G06/G07, drift 0.1 m/s + intermittent acoustic heartbeat channel (50% loss, 0.3 s latency; formation hint only)

- status: completed_early_stop, simulated 220.2 s in 985.8 s wall time
- drones: 3, inter-agent communication: ON
- determinism monitor violations: {'drone_0': 0, 'drone_1': 0, 'drone_2': 0}

| property | verdict (referee, ground truth) | key numbers |
|---|---|---|
| P1 separation | HOLDS | min distance 2.4534 m (d_safe 1.0 m), violations 0 |
| P2 mutual exclusion | HOLDS | max occupancy {'G06': 1, 'G07': 1}, entry order {'G06': ['drone_0', 'drone_2', 'drone_1'], 'G07': ['drone_0', 'drone_2', 'drone_1']} |
| P3 formation recovery | HOLDS | 1 episode(s), max recovery after perturbation 18.699813000000006 s (deadline 75.0 s), final error 0.0417 m |
| operational envelope | inside | max effective drift 0.1 m/s vs claimed 0.6 m/s |

Gate/formation decisions logged by the automata: `{"FORMATION_LOST": 2, "FORMATION_RECOVERED": 5, "APPROACH": 6, "PASS": 2, "YIELD": 5, "EXITED": 6, "RETRY_PASS": 4}`

Perception vs ground truth: 11933 neighbour estimates, coverage 98.1% within 8 m, |error| p99 per axis [0.47, 0.465, 0.112] m, max [1.18, 1.451, 0.235] m.

![trajectories_topdown](figures/trajectories_topdown.png)

![pairwise_distances](figures/pairwise_distances.png)

![critical_region_occupancy](figures/critical_region_occupancy.png)

![formation_error](figures/formation_error.png)

![automaton_modes](figures/automaton_modes.png)

![depth_profile](figures/depth_profile.png)

![perception_error](figures/perception_error.png)

![sensor_samples](figures/sensor_samples.png)

![scene_snapshots](figures/scene_snapshots.png)

![chasecamera](chasecamera.gif)

![sidecamera](sidecamera.gif)
