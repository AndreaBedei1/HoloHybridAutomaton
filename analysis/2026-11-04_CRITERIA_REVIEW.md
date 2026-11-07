# Independent criteria review before the B2 freeze

Read-only review of criteria v3, replay, gate events and permanent-fault semantics, requested during infrastructure
validation. The reviewer received the property definitions and constraints, not prior review verdicts or the author's
conclusions. No tests, formal checks, simulations, process changes or edits were performed by this reviewer.

**Status:** review findings below await author verification and disposition. No acceptance value or formula was changed.
The operational authority remains `CODEX_HANDOFF_B2_CONSOLIDATION.md`, with the user's subsequent instructions.

## Findings to verify before freeze

1. **Unsafe-action disjunction omitted.** `robustness_metrics.py` uses E4 alone; handoff Section 6.4 requires
   E3 OR E4 OR E2.to_free. A commit cancelled by stage 2 can have E3=1/E4=0/E7=1; a premature FREE without a
   commit can have E2.to_free>0/E4=0. Implementing the specified disjunction preserves the requirements.
2. **Replay safety inputs missing.** `replay_terminal` supplies zero collision flags and no intruders, whereas the
   online terminal fails on recorded contacts and intruder proximity. Restore time-indexed evidence rather than
   changing class priority to hide a replay mismatch.
3. **Horizon leakage.** Contacts/intruder summaries, automaton totals, vacancy events and some P2 aggregates still
   come from the full log after positions/states have been cut. Later events must not be clipped onto the final
   retained row or change the class of an earlier completed mission. Historical aggregates without time localization
   need an explicit limitation.
4. **SAFE_DEGRADED drops the hold condition.** `_degradation_explains` tests final geometry after excluding sensor-fault
   FAILSAFE drones. Non-FAILSAFE drones that recovered only 1 s before deadline still independently miss the 8.9 s
   hold. Apply the same episode/hold semantics to their retained history.
5. **Abort cycles and bounded response.** `gate_events` times uninterrupted committed intervals. The stated
   commit-to-passage obligation is not reset merely by an abort. Preserve the original response obligation across
   retries. Cancelling it on abort would be a property decision requiring the user, not an automatic review fix.
6. **Bypass and idle stall.** A bypass remains unpassed but no longer queued, preventing `all_waiting` for the remaining
   queue. Separate liveness participation from the failed passage requirement/GATE_BYPASS tag. Check later genuine
   passages after an initial bypass for consistency with TerminalMonitor.
7. **Tolerance boundary.** Recovery in `referee.py` uses strict `< e_ok`; the property includes `<= 1 m`. Preserve the
   value and implement the inclusive boundary for ordinary and degraded recovery.
8. **Superseded documents.** ACCEPTANCE_CRITERIA still describes the historical v2 rules/reference-derived deadline
   and 624 B0/B2 matrix. Retain that history, record its supersession and ad4fb5e/discarded-run deviation, then freeze
   v3 only after the prerequisites pass. PARAMETER_PROVENANCE incorrectly equates controller/referee thresholds.

## Derivation and claim limits (keep all fixed values)

- E_TOL=1 m is an agreed mission requirement, not a literature-derived physical bound.
- T_HOLD=4/0.45=8.888... s, rounded to 8.9 s, is motivated by an ideal first-order loop. Velocity caps and sonar
  corrections mean it is not a proved 2% settling bound for the whole closed loop.
- Progress tolerance 30*(0.30-0.20)=3 m corresponds to one nominal reduced-progress episode. Retained tau, independent
  synchronization slowdown and the same-lane latch can accumulate more schedule lag. Do not increase the tolerance.
- Deadline terms are a-priori allowances, not established physical worst-case progress bounds under all disturbances.
- The FAILSAFE count uses a Poisson-based quantile per process/drone on the pre-FAILSAFE base horizon. The actual
  generator has a uniform first start plus exponential gaps and outage durations; this does not establish 99% coverage
  for the final fleet mission. It covers scheduled sonar/DVL outages, not every FAILSAFE cause. Changing its formula
  or coverage target requires the user's methodological decision.
- `_update_absence` is limited to original fleets of at least three; fewer than two survivors have no nontrivial
  relative formation geometry. Do not claim those cases as tested degraded-formation recovery.
- An open final response interval is inconclusive only while shorter than its bound; exceeding the bound is already
  a violation. The code counts this, but the gate-events docstring overstates inconclusiveness.
- Finite holds cannot exclude later oscillation. Sampled ground-truth positions alone do not prove continuous-time
  P1/P2. Keep formal, sampled empirical, and assumption-dependent claims separate.

The reviewer found the remaining class order consistent with handoff Section 6.4, including CENSORED before degraded
or incomplete conclusions for historical traces which end too early.
