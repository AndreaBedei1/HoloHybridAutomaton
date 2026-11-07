# B2-only validation campaign: results (criteria v5)

Runs planned 216; complete 216; missing 0; INVALID 0; SUT_ERROR 0.  Commits: {'9866242d7abcd9d0d528f2b4f235fe8e6452e422': 216}.

## Item 1: safety (ground truth, every complete run)

* judged runs: 216 (gate runs 108); BLACKOUT x gate_stalled_occupant reported separately.
* runs violating P1, P2, with a contact or an unsafe action (E2.to_free, E3, E4): **0**; 95 % upper bound of the per-run rate 0.0139 (rule of three)
* REPLAY_MISMATCH: 0


BLACKOUT x gate_stalled_occupant (predicted outside the guarantee, M3f): 0 runs, P2 violated in 0, unsafe actions in 0.

## Item 2: H6 premise (sent navigation bound contains the true error)

* runs with the bound logged: 216; runs with an exceedance: **0**; smallest margin 0.956 m


## Item 3: CLEAN liveness (B2_CLEAN / CLEAN)

* runs 72 (66 outside gate_stalled_occupant); non-PASS outside it: **0**


## Item 4: cells (16 scenarios, gate_stalled_occupant excluded; Kaplan-Meier censored at the v5 horizon, 3 x the a-priori deadline)

| cell | runs | PASS | on time | SAFE_DEGRADED | INCOMPLETE | other | PASS+SD [95 % Wilson] | KM median t_end [s] | PASS late at the gate | PASS with transient false vacancy |
|---|---|---|---|---|---|---|---|---|---|---|
| B2_CLEAN / CLEAN | 66 | 66 | 60 | 0 | 0 | 0 | 66/66 [0.94, 1.00] | 103.5 | 0 | 1 |
| B2_LOW / LOW-ALL | 66 | 66 | 60 | 0 | 0 | 0 | 66/66 [0.94, 1.00] | 113.9 | 0 | 4 |
| B2_MEDIUM / MEDIUM-ALL | 66 | 56 | 51 | 0 | 10 | 0 | 56/66 [0.74, 0.92] | 155.0 | 8 | 2 |

## Item 5: gate_stalled_occupant (v5: safety only over the whole horizon)

* non-BLACKOUT runs 18; accepted (P1, P2, no contact, E2.to_free = E3 = E4 = E5 = 0): **18**; class PASS 18; INVALID (trigger never met or infrastructure): 0.  Not included in the PASS rates or the completion curves.

## Item 6: acoustic benefit (B2 vs B2 under outage, endpoint PASS, exact McNemar, Holm)

| comparison | pairs | B2 only | outage only | p | p (Holm) |
|---|---|---|---|---|---|
| B2_MEDIUM/MEDIUM-ALL vs B2_BLACKOUT/MEDIUM-ALL | 0 | 0 | 0 | 1.0000 | 1.0000 |
| B2_CLEAN/CLEAN vs B2_BLACKOUT/CLEAN | 0 | 0 | 0 | 1.0000 | 1.0000 |

Pairing is nominal (HoloOcean's own sensor noise is unseeded).

## Classes by scenario and cell

| scenario | B2_CLEAN/CLEAN | B2_LOW/LOW-ALL | B2_MEDIUM/MEDIUM-ALL |
|---|---|---|---|
| p1_head_on | PASS3 | PASS3 | PASS3 |
| p1_vertical_escape | PASS3 | PASS3 | PASS3 |
| p1_two_lines | PASS3 | PASS3 | PASS3 |
| p1_close_encounter | PASS3 | PASS3 | PASS3 |
| formation_triangle | PASS3 | PASS3 | PASS3 |
| formation_square | PASS3 | PASS3 | PASS3 |
| formation_six | PASS3 | PASS3 | PASS3 |
| formation_recovery_head_current | PASS3 | PASS3 | INCO1 PASS2 |
| formation_gust | PASS3 | PASS3 | INCO1 PASS2 |
| lost_drone_rejoin | PASS3 | PASS3 | INCO1 PASS2 |
| lost_drone_rejoin_front | PASS3 | PASS3 | PASS3 |
| lost_drone_timeout | PASS3 | PASS3 | PASS3 |
| gate_single | PASS6 | PASS6 | PASS6 |
| integrated_short | PASS6 | PASS6 | INCO3 PASS3 |
| mutex_deadlock_resolution | PASS6 | PASS6 | PASS6 |
| line_parallel_mutex | PASS6 | PASS6 | PASS6 |
| lost_drone_mutex | PASS6 | PASS6 | INCO4 PASS2 |
| gate_stalled_occupant | PASS6 | PASS6 | PASS6 |
