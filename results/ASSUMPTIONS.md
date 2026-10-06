# Empirical validation of the formal assumptions

Measured offline on the HoloOcean runs (ground truth vs onboard logs). 'VIOLATED' means the run left the assumption set, so the formal guarantee does not cover that run (its referee verdict is still empirical evidence).

| run | A_eps (max abs err xyz, m) | A_cov | A_sym | A_tau max (s) | A_hold (lat/z/s max, m) | A_mono worst (m) | A_cmax (m/s) |
|---|---|---|---|---|---|---|---|
| dev_form_02 | [0.28, 0.178, 0.101] | 100.0% | 100.0% | 0.07 | - / - / - | - | 0.505 |

Formal parameters: eps_rel=0.3 m, tau_max=0.2 s, hold_tol lat/z/s = 0.3/0.15/0.3 m, mono_tol=0.15 m, c_max=1.05 m/s.
