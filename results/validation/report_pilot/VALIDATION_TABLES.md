# Validation experiments

## Communication sweep

| factor | level | scenario | runs | complete | P3 failures | P1/P2 ok | mean completion s | mean last gate s | mean gate wait s | max live silence s | fairness violated | false vacancies | premise C max s | blind commits | reception rate |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline | 0 | formation_square | 1 | 1 | 0 | 1 | 78.9 | None | 0.0 | 3.1 | 0 | 0 | 0 | 0 | 0.9 |
| baseline | 0 | gate_single | 1 | 1 | 0 | 1 | 150.2 | 74.5 | 30.4 | 4.1 | 0 | 0 | 1.7 | 0 | 0.9 |
| baseline | 0 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.7 | 81.0 | 21.2 | 168.0 | 0 | 0 | 0.9 | 0 | 0.5 |
| per | 2 | formation_square | 1 | 1 | 0 | 1 | 79.1 | None | 0.0 | 6.1 | 0 | 0 | 0 | 0 | 0.9 |
| per | 2 | gate_single | 1 | 1 | 0 | 1 | 149.8 | 72.2 | 29.0 | 8.1 | 0 | 0 | 2.6 | 0 | 0.9 |
| per | 2 | lost_drone_mutex | 1 | 1 | 0 | 1 | 160.1 | 82.5 | 21.0 | 169.0 | 0 | 0 | 2.4 | 0 | 0.5 |
| per | 4 | formation_square | 1 | 1 | 0 | 1 | 79.0 | None | 0.0 | 12.1 | 1 | 0 | 0 | 0 | 0.9 |
| per | 4 | gate_single | 1 | 1 | 0 | 1 | 150.3 | 77.5 | 31.8 | 8.1 | 1 | 0 | 4.7 | 0 | 0.9 |
| per | 4 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.4 | 68.1 | 17.1 | 169.0 | 0 | 0 | 2.9 | 0 | 0.5 |
| per | 8 | formation_square | 1 | 1 | 0 | 1 | 79.1 | None | 0.0 | 24.1 | 1 | 0 | 0 | 0 | 0.9 |
| per | 8 | gate_single | 1 | 1 | 0 | 1 | 153.0 | 92.0 | 40.0 | 24.1 | 1 | 0 | 8.6 | 0 | 0.8 |
| per | 8 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.5 | 68.6 | 16.8 | 169.0 | 1 | 0 | 4.3 | 0 | 0.5 |
| lat | 0.5 | formation_square | 1 | 1 | 0 | 1 | 79.1 | None | 0.0 | 3.5 | 0 | 0 | 0 | 0 | 0.9 |
| lat | 0.5 | gate_single | 1 | 1 | 0 | 1 | 158.2 | 104.5 | 45.4 | 5.5 | 0 | 0 | 11.4 | 0 | 0.9 |
| lat | 0.5 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.1 | 65.1 | 16.2 | 168.0 | 0 | 0 | 4.8 | 0 | 0.5 |
| lat | 1 | formation_square | 1 | 1 | 0 | 1 | 79.5 | None | 0.0 | 4.0 | 0 | 0 | 0 | 0 | 0.9 |
| lat | 1 | gate_single | 1 | 1 | 0 | 1 | 150.2 | 84.0 | 31.6 | 4.9 | 0 | 0 | 10.3 | 0 | 0.9 |
| lat | 1 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.4 | 64.1 | 15.8 | 168.0 | 0 | 0 | 0 | 0 | 0.5 |
| lat | 2 | formation_square | 1 | 1 | 0 | 1 | 79.1 | None | 0.0 | 5.0 | 1 | 0 | 0 | 0 | 0.9 |
| lat | 2 | gate_single | 1 | 1 | 0 | 1 | 158.7 | 105.2 | 45.7 | 5.9 | 1 | 0 | 11.9 | 0 | 0.9 |
| lat | 2 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.1 | 66.2 | 15.7 | 168.0 | 0 | 0 | 5.6 | 0 | 0.5 |
| jit | 0.25 | formation_square | 1 | 1 | 0 | 1 | 79.4 | None | 0.0 | 3.4 | 0 | 0 | 0 | 0 | 0.9 |
| jit | 0.25 | gate_single | 1 | 1 | 0 | 1 | 150.3 | 76.5 | 30.6 | 5.3 | 0 | 0 | 2.7 | 0 | 0.9 |
| jit | 0.25 | lost_drone_mutex | 1 | 1 | 0 | 1 | 160.1 | 70.2 | 17.7 | 169.0 | 0 | 0 | 2.0 | 0 | 0.5 |
| jit | 0.5 | formation_square | 1 | 1 | 0 | 1 | 79.3 | None | 0.0 | 3.7 | 0 | 0 | 0 | 0 | 0.9 |
| jit | 0.5 | gate_single | 1 | 1 | 0 | 1 | 150.2 | 78.6 | 32.6 | 4.6 | 0 | 0 | 3.8 | 0 | 0.9 |
| jit | 0.5 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.5 | 68.1 | 17.0 | 169.0 | 0 | 0 | 2.1 | 0 | 0.5 |
| jit | 1 | formation_square | 1 | 1 | 0 | 1 | 79.1 | None | 0.0 | 4.5 | 0 | 0 | 0 | 0 | 0.9 |
| jit | 1 | gate_single | 1 | 1 | 0 | 1 | 150.4 | 85.6 | 36.9 | 4.7 | 0 | 0 | 11.1 | 0 | 0.9 |
| jit | 1 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.2 | 78.5 | 20.5 | 169.0 | 0 | 0 | 5.2 | 0 | 0.5 |
| loss | 0.3 | formation_square | 1 | 1 | 0 | 1 | 79.0 | None | 0.0 | 5.1 | 0 | 0 | 0 | 0 | 0.7 |
| loss | 0.3 | gate_single | 1 | 1 | 0 | 1 | 149.7 | 72.7 | 28.8 | 7.1 | 0 | 0 | 3.7 | 0 | 0.7 |
| loss | 0.3 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.4 | 81.5 | 21.7 | 168.0 | 0 | 0 | 0.9 | 0 | 0.4 |
| loss | 0.5 | formation_square | 1 | 1 | 0 | 1 | 79.3 | None | 0.0 | 9.1 | 0 | 0 | 0 | 0 | 0.5 |
| loss | 0.5 | gate_single | 1 | 1 | 0 | 1 | 150.4 | 77.6 | 32.2 | 10.1 | 0 | 0 | 3.6 | 0 | 0.5 |
| loss | 0.5 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.4 | 66.9 | 16.7 | 169.0 | 0 | 0 | 0.9 | 0 | 0.2 |
| loss | 0.7 | formation_square | 1 | 1 | 0 | 1 | 78.9 | None | 0.0 | 15.1 | 1 | 0 | 0 | 0 | 0.3 |
| loss | 0.7 | gate_single | 1 | 1 | 0 | 1 | 150.0 | 79.5 | 32.1 | 15.1 | 0 | 0 | 3.6 | 0 | 0.3 |
| loss | 0.7 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.5 | 66.9 | 16.6 | 169.0 | 0 | 0 | 2.9 | 0 | 0.2 |
| loss | 0.9 | formation_square | 1 | 1 | 0 | 1 | 79.1 | None | 0.0 | 60.1 | 1 | 1 | 0 | 0 | 0.1 |
| loss | 0.9 | gate_single | 1 | 1 | 0 | 1 | 152.1 | 88.5 | 39.9 | 34.1 | 1 | 0 | 9.2 | 0 | 0.1 |
| loss | 0.9 | lost_drone_mutex | 1 | 0 | 1 | 1 | 159.2 | 80.5 | 21.2 | 168.0 | 1 | 1 | 4.7 | 0 | 0.0 |
| burst | 10 | formation_square | 1 | 1 | 0 | 1 | 79.8 | None | 0.0 | 12.1 | 0 | 0 | 0 | 0 | 0.8 |
| burst | 10 | gate_single | 1 | 1 | 0 | 1 | 150.6 | 78.7 | 33.1 | 11.1 | 0 | 0 | 0.7 | 0 | 0.8 |
| burst | 10 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.7 | 69.4 | 16.8 | 168.0 | 0 | 0 | 0.9 | 0 | 0.4 |
| burst | 30 | formation_square | 1 | 1 | 0 | 1 | 79.5 | None | 0.0 | 32.1 | 0 | 0 | 0 | 0 | 0.6 |
| burst | 30 | gate_single | 1 | 1 | 0 | 1 | 155.2 | 97.6 | 45.6 | 31.1 | 0 | 0 | 0.7 | 0 | 0.7 |
| burst | 30 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.9 | 80.9 | 21.2 | 169.0 | 0 | 0 | 0.9 | 0 | 0.4 |
| burst | 60 | formation_square | 1 | 1 | 0 | 1 | 80.1 | None | 0.0 | 35.7 | 0 | 0 | 0 | 0 | 0.6 |
| burst | 60 | gate_single | 1 | 1 | 0 | 1 | 170.9 | 127.7 | 65.5 | 61.1 | 1 | 2 | 0.7 | 0 | 0.6 |
| burst | 60 | lost_drone_mutex | 1 | 1 | 0 | 1 | 163.6 | 111.0 | 31.6 | 169.0 | 1 | 0 | 0.9 | 0 | 0.3 |
| asym | 0.5 | formation_square | 1 | 1 | 0 | 1 | 79.4 | None | 0.0 | 8.1 | 0 | 0 | 0 | 0 | 0.7 |
| asym | 0.5 | gate_single | 1 | 1 | 0 | 1 | 150.1 | 74.6 | 30.1 | 9.1 | 0 | 0 | 0.7 | 0 | 0.7 |
| asym | 0.5 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.9 | 66.1 | 16.3 | 169.0 | 0 | 0 | 0 | 0 | 0.4 |
| asym | 0.9 | formation_square | 1 | 1 | 0 | 1 | 79.5 | None | 0.0 | 31.1 | 1 | 0 | 0 | 0 | 0.5 |
| asym | 0.9 | gate_single | 1 | 1 | 0 | 1 | 150.3 | 84.5 | 33.5 | 25.7 | 1 | 0 | 0.6 | 0 | 0.5 |
| asym | 0.9 | lost_drone_mutex | 1 | 1 | 0 | 1 | 159.4 | 65.8 | 16.4 | 168.0 | 0 | 0 | 0.9 | 0 | 0.3 |
| blackout | 1 | formation_square | 1 | 1 | 0 | 1 | 79.0 | None | 0.0 | 88.7 | 1 | 0 | 0 | 0 | 0.0 |
| blackout | 1 | gate_single | 1 | 1 | 0 | 1 | 150.3 | 84.0 | 31.4 | 160.0 | 1 | 0 | 10.6 | 0 | 0.0 |
| blackout | 1 | lost_drone_mutex | 1 | 0 | 0 | 1 | 169.4 | 80.3 | 21.1 | 169.3 | 1 | 1 | 4.9 | 0 | 0.0 |

## Emergence

| protocol | scenario | seed | class | reaction s | max form err m | err at jet end m | recovery after jet s | mean err after m | P1 min m | duration s |
|---|---|---|---|---|---|---|---|---|---|---|
| B2NA_LOW | em_delay_head | 6000 | PASS | None | 2.72 | 1.26 | 16.2 | 0.56 | 3.058 | 102.1 |
| B2_LOW | em_delay_head | 6000 | PASS | 5.7 | 2.3 | 1.21 | 10.4 | 0.34 | 3.074 | 102.1 |
| B2NA_LOW | lost_drone_rejoin | 6000 | PASS | None | 3.54 | 1.99 | 24.0 | 0.81 | 3.134 | 102.1 |
| B2_LOW | lost_drone_rejoin | 6000 | PASS | 3.7 | 2.67 | 1.87 | 13.7 | 0.45 | 2.956 | 102.1 |

### Emergence by configuration (mean, min-max over seeds)

| protocol | scenario | runs | complete | reaction s | adaptation time s | progress yielded m | min v_ref m/s | max form err m | recovery after jet s | mean err after m | P1 min m |
|---|---|---|---|---|---|---|---|---|---|---|---|
| B2NA_LOW | em_delay_head | 1 | 1 | none | 0.00 (0.00-0.00) n=1 | 0.00 (0.00-0.00) n=1 | 0.30 (0.30-0.30) n=1 | 2.72 (2.72-2.72) n=1 | 16.20 (16.20-16.20) n=1 | 0.56 (0.56-0.56) n=1 | 3.058 |
| B2_LOW | em_delay_head | 1 | 1 | 5.70 (5.70-5.70) n=1 | 47.60 (47.60-47.60) n=1 | 2.45 (2.45-2.45) n=1 | 0.20 (0.20-0.20) n=1 | 2.30 (2.30-2.30) n=1 | 10.40 (10.40-10.40) n=1 | 0.34 (0.34-0.34) n=1 | 3.074 |
| B2NA_LOW | lost_drone_rejoin | 1 | 1 | none | 0.00 (0.00-0.00) n=1 | 0.00 (0.00-0.00) n=1 | 0.30 (0.30-0.30) n=1 | 3.54 (3.54-3.54) n=1 | 24.00 (24.00-24.00) n=1 | 0.81 (0.81-0.81) n=1 | 3.134 |
| B2_LOW | lost_drone_rejoin | 1 | 1 | 3.70 (3.70-3.70) n=1 | 68.60 (68.60-68.60) n=1 | 4.98 (4.98-4.98) n=1 | 0.20 (0.20-0.20) n=1 | 2.67 (2.67-2.67) n=1 | 13.70 (13.70-13.70) n=1 | 0.45 (0.45-0.45) n=1 | 2.956 |
