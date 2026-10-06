# Run `pair_crossing_s0`

2 drones head-on on the same survey line (P1 sanity check)

- status: completed_early_stop, simulated 75.8 s in 243.2 s wall time
- drones: 2, inter-agent communication: OFF
- determinism monitor violations: {'drone_0': 0, 'drone_1': 0}

| property | verdict (referee, ground truth) | key numbers |
|---|---|---|
| P1 separation | HOLDS | min distance 2.1111 m (d_safe 1.0 m), violations 0 |
| P2 mutual exclusion | n/a (no gate) | max occupancy {}, entry order {} |
| P3 formation recovery | n/a | 0 episode(s), max recovery after perturbation None s (deadline 75.0 s), final error None m |
| operational envelope | inside | max effective drift 0.0 m/s vs claimed 0.6 m/s |

Gate/formation decisions logged by the automata: `{"RESUME_FOLLOW": 6}`

Perception vs ground truth: 768 neighbour estimates, coverage 100.0% within 8 m, |error| p99 per axis [0.446, 0.473, 0.076] m, max [0.593, 0.478, 0.106] m.

![trajectories_topdown](figures/trajectories_topdown.png)

![pairwise_distances](figures/pairwise_distances.png)

![automaton_modes](figures/automaton_modes.png)

![depth_profile](figures/depth_profile.png)

![perception_error](figures/perception_error.png)

![sensor_samples](figures/sensor_samples.png)

![scene_snapshots](figures/scene_snapshots.png)

![chasecamera](chasecamera.gif)

![sidecamera](sidecamera.gif)
