# Run `formation_medium_gust_s0`

Scenario A: triangle survey, uniform lateral drift 0.25 m/s, plus a 0.80 m/s lateral gust on the right lane for t in [35, 50] s

- status: completed_early_stop, simulated 106.5 s in 494.6 s wall time
- drones: 3, inter-agent communication: OFF
- determinism monitor violations: {'drone_0': 0, 'drone_1': 0, 'drone_2': 0}

| property | verdict (referee, ground truth) | key numbers |
|---|---|---|
| P1 separation | HOLDS | min distance 2.4479 m (d_safe 1.0 m), violations 0 |
| P2 mutual exclusion | n/a (no gate) | max occupancy {}, entry order {} |
| P3 formation recovery | HOLDS | 1 episode(s), max recovery after perturbation 4.899950999999994 s (deadline 75.0 s), final error 0.0952 m |
| operational envelope | EXCEEDED (flagged) | max effective drift 1.026 m/s vs claimed 0.6 m/s |

Gate/formation decisions logged by the automata: `{"FORMATION_LOST": 5, "FORMATION_RECOVERED": 5}`

Envelope declarations by the drones themselves: drone_1 ENVELOPE_VIOLATION t=41.3s, drone_1 ENVELOPE_RESTORED t=45.3s

Perception vs ground truth: 6390 neighbour estimates, coverage 100.0% within 8 m, |error| p99 per axis [0.336, 0.213, 0.015] m, max [0.377, 0.27, 0.069] m.

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
