# Run `formation_medium_s0`

Scenario A: triangle survey, uniform lateral drift 0.25 m/s

- status: completed_early_stop, simulated 99.9 s in 469.8 s wall time
- drones: 3, inter-agent communication: OFF
- determinism monitor violations: {'drone_0': 0, 'drone_1': 0, 'drone_2': 0}

| property | verdict (referee, ground truth) | key numbers |
|---|---|---|
| P1 separation | HOLDS | min distance 3.5131 m (d_safe 1.0 m), violations 0 |
| P2 mutual exclusion | n/a (no gate) | max occupancy {}, entry order {} |
| P3 formation recovery | HOLDS | 0 episode(s), max recovery after perturbation None s (deadline 75.0 s), final error 0.0501 m |
| operational envelope | inside | max effective drift 0.25 m/s vs claimed 0.6 m/s |

Gate/formation decisions logged by the automata: `{"FORMATION_LOST": 2, "FORMATION_RECOVERED": 2}`

Perception vs ground truth: 5994 neighbour estimates, coverage 100.0% within 8 m, |error| p99 per axis [0.33, 0.217, 0.012] m, max [0.391, 0.275, 0.014] m.

![trajectories_topdown](figures/trajectories_topdown.png)

![pairwise_distances](figures/pairwise_distances.png)

![formation_error](figures/formation_error.png)

![automaton_modes](figures/automaton_modes.png)

![depth_profile](figures/depth_profile.png)

![perception_error](figures/perception_error.png)

![sensor_samples](figures/sensor_samples.png)

![scene_snapshots](figures/scene_snapshots.png)

![chasecamera](chasecamera.gif)

![sidecamera](sidecamera.gif)
