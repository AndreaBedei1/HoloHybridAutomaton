# Validation experiments

## Communication sweep

| factor | level | scenario | runs | complete | P3 failures | P1/P2 ok | mean completion s | mean last gate s | mean gate wait s | max live silence s | fairness violated | false vacancies | premise C sat / conc / viol / inc | CR entries not admitted | packets received | accepted | rejected | stale |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| baseline | 0 | gate_single | 5 | 5 | 0 | 5 | 150.6 | 75.1 | 30.8 | 5.1 | 0 | 0 | 15 / 0 / 0 / 0 | 0 | 4448 | 4385 | 63 | 0 |
| baseline | 0 | lost_drone_mutex | 5 | 5 | 0 | 5 | 159.5 | 75.7 | 19.6 | 169.0 | 0 | 0 | 5 / 0 / 0 / 0 | 0 | 1668 | 1663 | 5 | 0 |
| lat | 0.35 | gate_single | 5 | 5 | 0 | 5 | 152.9 | 89.6 | 35.5 | 5.3 | 0 | 0 | 15 / 0 / 0 / 0 | 0 | 4445 | 2977 | 47 | 1421 |
| lat | 0.35 | lost_drone_mutex | 5 | 5 | 0 | 5 | 159.1 | 75.3 | 19.5 | 169.0 | 0 | 0 | 5 / 0 / 0 / 0 | 0 | 1661 | 909 | 3 | 749 |
| lat | 0.45 | gate_single | 5 | 5 | 0 | 5 | 156.7 | 101.2 | 44.3 | 5.4 | 0 | 0 | 15 / 0 / 0 / 0 | 0 | 4423 | 0 | 0 | 4423 |
| lat | 0.45 | lost_drone_mutex | 5 | 5 | 0 | 5 | 159.5 | 70.3 | 17.9 | 169.0 | 0 | 0 | 5 / 0 / 0 / 0 | 0 | 1658 | 0 | 0 | 1658 |
