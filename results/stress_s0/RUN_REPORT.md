# Run `stress_s0`

Scenario C: gate arena, variable current (sinusoid+gusts+jet), launch delays 0/7/14 s, 25% beam dropout, 0.8 s sonar blackouts, FLS off on drone_2, 0.15 m nav init error

- status: completed_early_stop, simulated 308.3 s in 1363.5 s wall time
- drones: 3, inter-agent communication: OFF
- determinism monitor violations: {'drone_0': 0, 'drone_1': 0, 'drone_2': 0}

| property | verdict (referee, ground truth) | key numbers |
|---|---|---|
| P1 separation | HOLDS | min distance 2.4084 m (d_safe 1.0 m), violations 0 |
| P2 mutual exclusion | HOLDS | max occupancy {'G06': 1, 'G07': 1}, entry order {'G06': ['drone_0', 'drone_2', 'drone_1'], 'G07': ['drone_0', 'drone_2', 'drone_1']} |
| P3 formation recovery | HOLDS | 1 episode(s), max recovery after perturbation None s (deadline 75.0 s), final error 0.1295 m |
| operational envelope | inside | max effective drift 0.488 m/s vs claimed 0.6 m/s |

Gate/formation decisions logged by the automata: `{"FORMATION_LOST": 12, "FORMATION_RECOVERED": 16, "RESUME_FOLLOW": 14, "APPROACH": 10, "PASS": 2, "YIELD": 5, "RETRY_PASS": 4, "EXITED": 6, "RESUME_PASS": 5}`

Perception vs ground truth: 16382 neighbour estimates, coverage 98.0% within 8 m, |error| p99 per axis [0.489, 0.45, 0.26] m, max [1.236, 1.013, 0.54] m.

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
