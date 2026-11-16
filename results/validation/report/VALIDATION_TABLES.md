# Validation experiments

## Communication sweep

| factor | level | scenario | runs | complete | P3 failures | P1/P2 ok | mean completion s | mean last gate s | mean gate wait s | max live silence s | fairness violated | false vacancies | premise C max s | blind commits | reception rate |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline | 0 | formation_square | 5 | 5 | 0 | 5 | 79.5 | None | 0.0 | 6.1 | 0 | 0 | 0 | 0 | 0.9 |
| baseline | 0 | gate_single | 5 | 5 | 0 | 5 | 150.3 | 73.7 | 29.9 | 7.1 | 0 | 0 | 3.6 | 0 | 0.9 |
| baseline | 0 | lost_drone_mutex | 5 | 5 | 0 | 5 | 159.9 | 76.5 | 19.8 | 169.0 | 0 | 0 | 1.8 | 0 | 0.5 |
| per | 8 | gate_single | 5 | 5 | 0 | 5 | 151.5 | 85.8 | 36.8 | 40.1 | 5 | 0 | 8.7 | 0 | 0.9 |
| lat | 2 | gate_single | 5 | 5 | 0 | 5 | 156.7 | 101.5 | 44.3 | 9.0 | 5 | 0 | 12.1 | 0 | 0.9 |
| jit | 1 | gate_single | 5 | 5 | 0 | 5 | 152.9 | 91.7 | 38.6 | 6.5 | 0 | 0 | 6.8 | 0 | 0.9 |
| loss | 0.7 | gate_single | 5 | 5 | 0 | 5 | 150.5 | 78.1 | 32.6 | 21.1 | 4 | 0 | 6.6 | 0 | 0.3 |
| loss | 0.7 | lost_drone_mutex | 5 | 5 | 0 | 5 | 159.7 | 77.9 | 20.0 | 169.0 | 0 | 0 | 5.1 | 0 | 0.2 |
| loss | 0.9 | gate_single | 5 | 5 | 0 | 5 | 154.8 | 94.4 | 41.2 | 73.4 | 5 | 3 | 10.8 | 0 | 0.1 |
| loss | 0.9 | lost_drone_mutex | 5 | 5 | 0 | 5 | 159.3 | 78.2 | 20.5 | 169.0 | 5 | 0 | 5.1 | 0 | 0.1 |
| burst | 30 | gate_single | 5 | 5 | 0 | 5 | 152.4 | 88.2 | 37.6 | 33.1 | 0 | 0 | 2.6 | 0 | 0.7 |
| burst | 60 | gate_single | 5 | 5 | 0 | 5 | 161.5 | 107.1 | 47.9 | 63.1 | 5 | 15 | 4.4 | 0 | 0.5 |
| asym | 0.9 | gate_single | 5 | 5 | 0 | 5 | 150.6 | 79.1 | 31.7 | 76.1 | 5 | 1 | 4.6 | 0 | 0.5 |
| blackout | 1 | formation_square | 5 | 5 | 0 | 5 | 79.1 | None | 0.0 | 88.7 | 5 | 0 | 0 | 0 | 0.0 |
| blackout | 1 | gate_single | 5 | 5 | 0 | 5 | 154.0 | 93.2 | 37.9 | 160.0 | 5 | 0 | 11.0 | 0 | 0.0 |
| blackout | 1 | lost_drone_mutex | 5 | 0 | 0 | 5 | 169.2 | 75.4 | 19.3 | 169.3 | 5 | 5 | 5.0 | 0 | 0.0 |

## Emergence

| protocol | scenario | seed | class | reaction s | max form err m | err at jet end m | recovery after jet s | mean err after m | P1 min m | duration s |
|---|---|---|---|---|---|---|---|---|---|---|
| B2NA_LOW | em_delay_head | 6100 | PASS | None | 2.78 | 1.19 | 17.6 | 0.66 | 3.436 | 102.1 |
| B2NA_LOW | em_delay_head | 6101 | PASS | None | 2.74 | 1.19 | 16.8 | 0.57 | 3.449 | 102.1 |
| B2NA_LOW | em_delay_head | 6102 | PASS | None | 2.68 | 1.23 | 16.1 | 0.52 | 3.465 | 102.1 |
| B2NA_LOW | em_delay_head | 6103 | PASS | None | 2.79 | 1.14 | 17.8 | 0.6 | 3.145 | 102.1 |
| B2NA_LOW | em_delay_head | 6104 | PASS | None | 2.73 | 1.23 | 16.6 | 0.5 | 3.41 | 102.1 |
| B2_LOW | em_delay_head | 6100 | PASS | 5.7 | 2.38 | 1.13 | 11.5 | 0.35 | 3.18 | 102.1 |
| B2_LOW | em_delay_head | 6101 | PASS | 4.7 | 2.3 | 1.18 | 10.3 | 0.39 | 3.124 | 102.1 |
| B2_LOW | em_delay_head | 6102 | PASS | 5.7 | 2.32 | 1.22 | 10.2 | 0.34 | 3.161 | 102.1 |
| B2_LOW | em_delay_head | 6103 | PASS | 1.7 | 2.34 | 0.97 | 12.1 | 0.39 | 2.855 | 102.1 |
| B2_LOW | em_delay_head | 6104 | PASS | 5.7 | 2.32 | 1.18 | 10.8 | 0.34 | 3.185 | 102.1 |
| B2NA_LOW | lost_drone_rejoin | 6100 | PASS | None | 3.57 | 1.95 | 24.2 | 0.85 | 3.4 | 102.1 |
| B2NA_LOW | lost_drone_rejoin | 6101 | PASS | None | 3.55 | 2.01 | 24.2 | 0.88 | 3.345 | 102.1 |
| B2NA_LOW | lost_drone_rejoin | 6102 | PASS | None | 3.51 | 1.99 | 23.8 | 0.84 | 3.438 | 102.1 |
| B2NA_LOW | lost_drone_rejoin | 6103 | PASS | None | 3.59 | 1.95 | 24.4 | 0.87 | 3.019 | 102.1 |
| B2NA_LOW | lost_drone_rejoin | 6104 | PASS | None | 3.55 | 1.96 | 24.2 | 0.85 | 3.353 | 102.1 |
| B2_LOW | lost_drone_rejoin | 6100 | PASS | 4.7 | 2.76 | 1.88 | 14.2 | 0.45 | 3.198 | 102.1 |
| B2_LOW | lost_drone_rejoin | 6101 | PASS | 4.7 | 2.94 | 1.94 | 15.7 | 0.59 | 2.983 | 102.1 |
| B2_LOW | lost_drone_rejoin | 6102 | PASS | 4.7 | 2.73 | 1.87 | 13.4 | 0.47 | 2.886 | 102.1 |
| B2_LOW | lost_drone_rejoin | 6103 | PASS | 4.7 | 2.9 | 1.83 | 15.9 | 0.54 | 2.869 | 102.1 |
| B2_LOW | lost_drone_rejoin | 6104 | PASS | 4.7 | 2.82 | 1.88 | 14.5 | 0.56 | 3.007 | 102.1 |

### Emergence by configuration (mean, min-max over seeds)

| protocol | scenario | runs | complete | reaction s | adaptation time s | progress yielded m | min v_ref m/s | max form err m | recovery after jet s | mean err after m | P1 min m |
|---|---|---|---|---|---|---|---|---|---|---|---|
| B2NA_LOW | em_delay_head | 5 | 5 | none | 0.00 (0.00-0.00) n=5 | 0.00 (0.00-0.00) n=5 | 0.30 (0.30-0.30) n=5 | 2.74 (2.68-2.79) n=5 | 16.98 (16.10-17.80) n=5 | 0.57 (0.50-0.66) n=5 | 3.145 |
| B2_LOW | em_delay_head | 5 | 5 | 4.70 (1.70-5.70) n=5 | 48.26 (39.00-58.90) n=5 | 2.48 (1.95-2.94) n=5 | 0.20 (0.20-0.20) n=5 | 2.33 (2.30-2.38) n=5 | 10.98 (10.20-12.10) n=5 | 0.36 (0.34-0.39) n=5 | 2.855 |
| B2NA_LOW | lost_drone_rejoin | 5 | 5 | none | 0.00 (0.00-0.00) n=5 | 0.00 (0.00-0.00) n=5 | 0.30 (0.30-0.30) n=5 | 3.55 (3.51-3.59) n=5 | 24.16 (23.80-24.40) n=5 | 0.86 (0.84-0.88) n=5 | 3.019 |
| B2_LOW | lost_drone_rejoin | 5 | 5 | 4.70 (4.70-4.70) n=5 | 67.70 (63.30-70.40) n=5 | 4.58 (4.10-5.23) n=5 | 0.20 (0.20-0.20) n=5 | 2.83 (2.73-2.94) n=5 | 14.74 (13.40-15.90) n=5 | 0.52 (0.45-0.59) n=5 | 2.869 |
