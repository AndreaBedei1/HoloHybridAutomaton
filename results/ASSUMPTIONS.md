# Empirical validation of the formal assumptions

Measured offline on the HoloOcean runs (ground truth vs onboard logs). 'VIOLATED' means the run left the assumption set, so the formal guarantee does not cover that run (its referee verdict is still empirical evidence).

| run | A_eps: max abs err xyz, d <= 2.8 m (P1) | p99 abs err xyz, 2.8-8 m | A_cov: detection d <= 2.8 m / 2.8-8 m | A_sym | A_tau max [s] | A_hold: max lateral / vertical change of along-tied queued drones in transit windows, max s past queue line [m] | A_mono worst margin [m] | A_cmax [m/s] |
|---|---|---|---|---|---|---|---|---|
| pair_crossing_s0 | [0.249, 0.212, 0.091] | [0.223, 0.21, 0.011] | 100.0% / 100.0% | 100.0% | 0.07 | - | - | 0.794 |
| formation_medium_s0 | - | [0.33, 0.217, 0.012] | - / 100.0% | 100.0% | 0.07 | - | - | 0.147 |
| formation_high_s0 | - | [0.343, 0.222, 0.013] | - / 100.0% | 100.0% | 0.07 | - | - | 0.208 |
| formation_medium_gust_s0 | [0.276, 0.17, 0.069] | [0.336, 0.213, 0.015] | 100.0% / 100.0% | 100.0% | 0.07 | - | - | 0.351 |
| gate_arena_s0 | [0.309, 0.254, 0.085] | [0.373, 0.356, 0.109] | 100.0% / 98.1% | 100.0% | 0.07 | not invoked (no along-tied queued drone at any commit) | 0.01 | 0.544 |
| stress_s0 | [0.385, 0.359, 0.151] **(!)** | [0.429, 0.403, 0.255] | 100.0% / 97.9% | 100.0% | 0.17 | not invoked (no along-tied queued drone at any commit) | 0.01 | 0.863 |
| gate_arena_comms_s0 | [0.309, 0.299, 0.045] | [0.375, 0.341, 0.11] | 100.0% / 98.0% | 100.0% | 0.07 | not invoked (no along-tied queued drone at any commit) | 0.00 | 0.575 |

Formal parameters: eps_rel=0.35 m, tau_max=0.2 s, hold bounds lat/z = 0.60/0.30 m (2 x hold_tol), hold_tol_s = 0.3 m, mono_tol=0.15 m, c_max=1.05 m/s.
Errors are measured against the truth at the acquisition time of the sonar data (data age is checked separately as A_tau, as in the P1 model).  Detection coverage is a hard requirement inside the warning band; at 2.8-8 m it is reported for the gate corridor.
