# Round-2 independent review: underwater physics and acoustic communications

Commit `c6d8c98` (branch `feature/b2-medium-robustness`), against `120b400`. Read-only review; no simulator started.
Tags: **[M]** measured from repository logs or benches with my own scripts; **[D]** derived by me; **[A]** an assumption of the repository; **[L]** read in the source listed at the end.

## Verdict

No BLOCKER against reporting these results as *simulation* results under the declared hardware assumptions H1-H6: the internal logic matches the owner's failure model and "silence never clears a committed or admitted drone" holds in the code. Six MAJOR findings limit what the results can support. F1, F3 and F4 would become blockers for any statement about real vehicles or real channels, and F4 and F5 weaken the physical basis of the two silence thresholds and of the acoustic envelope.

## Findings

### F1. MAJOR. The rise-speed range is mis-derived; the campaign value sits at its top
`holo_fleet/config.py:387-389`, design note lines 54-57. The docstring says published heave damping gives "about 0.06 m/s". That is the linear term alone.
- [L] von Benzon et al. Table A1 (BlueROV2 Heavy, identified in a 1.15 m pool): Z_w = 33 N s/m, Z_w|w| = 190 N s^2/m^2, added mass 18.68 kg.
- [D] For F = 1.96 N (+0.2 kg): both terms 190 w^2 + 33 w = F give **0.047 m/s**; linear only 0.059; quadratic only 0.102. The config's plan-area formula gives 0.116 (0.132 with the paper's CAD area 0.2049 m^2).
- [D] 0.18 m/s needs F = 12.1 N (1.2 kg) with the published damping, six times the declared buoyancy. A vehicle trimmed +0.2 kg in fresh water and used at sea gains about 3.3 N and gives about 0.10 m/s.
- Assessment: 0.10 m/s is possible but is the upper part of 0.05-0.13 m/s for the declared +0.2 kg. The paper's own heave model error is largest (15 %) at its lowest input, so the linear term at 0.05 m/s is an extrapolation; neither end of the range is established. The +0.2 kg is itself [A]; the paper's measured configuration is 1 N heavy.
- Consequence: all 9 TOTAL_FAILURE runs of med3 use 0.10 m/s (`run_config.json`). At 0.047 m/s a dead hull is 0.8 m higher, not 1.75 m, when a follower at 3.5 m spacing and 0.2 m/s reaches it [D]: the rising hull stays in the lane during the wait.
- Resolve: correct the docstring and design note; run the three TOTAL_FAILURE scenarios at 0.05 and 0.13 m/s (after F7); state the range as 0.05-0.13 m/s for +0.2 kg, with higher values tied to larger buoyancy.

### F2. MAJOR. No campaign run contains a pinned occupant; the bench starts from rest
`calibration/probe_buoyant_ascent.py:41-43,98`, design note lines 58-61.
- [M] In all three `gate_stalled_occupant` runs (s910-912) the drone fails at s = 0.06-0.10 m moving at 0.42-0.46 m/s, coasts to s = 0.50-0.55 m, clears the bar and reaches the surface (dz end 4.14 m). The state "pinned at dz 0.87 m" was never present in a run. The gate stays BUSY by the GP_ADMITTED status alone, so the outcome does not depend on where the hull is.
- [D] The coast is an engine artefact. Engine drag is written in N but applied in cN (`HolodeckBuoyantAgent.cpp:91-94`, gravity and buoyancy are converted at 75, 82), so it is 100 times too small and linear damping dominates (`BlueROV2.cpp:73`). The four points of `current_cmd_to_drift` fit D = 11.9-12.4 N s/m against m c = 11.5. Stopping distance from 0.45 m/s: engine 0.44 m (observed 0.43-0.45), real vehicle with Table A1 values 0.24 m. A real drone failing at s = 0.08 m stops near s = 0.32 m, under the edge of the bar (half-depth 0.15 m + hull 0.229 m).
- The bench shows only: at zero initial velocity, zero current and zero tilt, UE collision holds the box under G06 for the 40 s it ran (290 contact steps). It says nothing about sliding, tilt from the off-centre contact, or current across the bar. Pinned force [D] is 1.41 N, not 1.96 N.
- Resolve: add a run where the failure occurs at the hold (low speed) with the hull under the bar, so survivors face a real persistent echo in the CR; add bench cases with initial velocity and with along-axis current.

### F3. MAJOR. Dead drones move with the water; the exclusion premise ignores it and the combination is untested
`config.py:357-359` ("has risen several metres above the gate"), `perception/acoustic_gate.py:150,158-164`, `perception/perception.py:310`.
- [M] All three TOTAL_FAILURE scenarios in med3 have `current: []`. A failed drone in a current has been tested only on the open-water bench.
- [D] Queue line s = -4.18 m (n = 2) to -5.41 m (n = 3), CR face s = -0.8 m, validated current 0.6 m/s (`config.Envelope.current_validated_max`). Along-axis 0.3 m/s: the hull reaches the CR face in 11-15 s, having risen 1.1-1.5 m at 0.10 m/s, 0.5-0.7 m at 0.047 m/s. Along-axis 0.6 m/s: 6-8 s, 0.6-0.8 m even at 0.10 m/s. The CR half-height is 1.0 m: at 0.3 m/s the hull clears the top only at the nominal rise; at the low rise, or at 0.6 m/s, it arrives inside the CR height.
- Failure: a GP_NONE drone that dies at the queue and is carried against the frame is excluded as an occupant after 70 s while physically in or against the CR. `GP_PASSED` is worse: the exclusion has no timeout and no silence condition, so a drone that dies just past the exit in a head current is excluded for good while drifting back. `no_fleet_drone` then lets the corridor echo stop holding the latch; contact is left to the sonar separation layer alone.
- Resolve: TOTAL_FAILURE at queue and at exit under along-axis currents 0.15, 0.3, 0.6 m/s; condition silence or GP_PASSED exclusion on a drift-reach bound (last accepted position plus the current bound times the elapsed time cannot reach the CR), or drop the exclusion for silent drones.

### F4. MAJOR. The loss process that justifies 45 s and 70 s is not the process seen in shallow-water trials
`sim/acoustic_channel.py:130-134,161-162,170,180-183`, `config.py:220-224,357-360`, design note lines 66-67.
- Model: per-ping iid dropout plus 4 s outages common to every link. [L] Morozs et al. (WUWNet 2024, Werbellin lake data): link ON/OFF durations are long-tailed (Pareto fits better than exponential), link quality barely correlates with distance in shallow short-range water, and OFF patterns of different links are nearly independent. From their Fig. 6c, about 10 % of OFF periods on "poor" links last 40 s or more; Fig. 5 shows a few percent at 60 s over all links (read from plots). Caveat: S2C modems, 43-720 m, 1 s logging; not the 3-7 m links here, where the transfer is unknown. The model has neither per-link long OFF periods nor independent bursts, so `silent()` (protected only against a whole-receiver outage, `perception/acoustic.py:308-310`) is never stressed by the event it would meet.
- The stated basis is the wrong statistic: [M] the 31 s of the 72 v10 runs is `formation_six` s801, over all pairs including the 17.5 m ones, for which vacancy by silence is not used. The maximum in any other scenario is 26 s.
- [M] Expected-neighbour links in med3 (432 pair-runs; gap in `form.acoustic_fixed` + 2.5 s, an upper bound of the heard gap): median 6.4 s, p99 17.4 s, maximum 21.3 s. The worst are 7 m pairs with 30-57 % of the time shadowed by the middle hull. Zero false vacancies; 21 of 21 silence vacancies were true. The binary 70 % shadow loss is a stress assumption: at 7 m the first Fresnel radius is 0.30 m at 30 kHz [D], comparable to the hull, so a real shadow is partial and not calibrated by any source I read.
- [D] Model extrapolation for a 7 m link shadowed all the time (q = 0.227): P(silence >= 45 s per 705 s mission) = 6e-3, 1.2e-3 for a 169 s mission; P(>= 70 s) about 8e-5 (8 events in 1e5). So the "< 1e-3" in `config.py:358` holds in the model, but the document it cites contains no such calculation.
- Failure: a false 45 s vacancy is recoverable (`perception/perception.py:285-288`). At 70 s the failure is not: a live GP_NONE drone unheard on one link for 70 s can commit during that OFF period and is excluded by that receiver anyway.
- Resolve: per-link ON/OFF channel (Gilbert-Elliott or an empirical heavy-tailed OFF law) as a sensitivity level; re-run the silence statistics on expected-neighbour links; put the calculation in the document or remove the citation; state the thresholds as simulation-calibrated.

### F5. MAJOR. Reflection geometry is not derived for this fleet, and the envelope depends on it
`sim/acoustic_channel.py:82,223-224`, `docs/ACOUSTIC_CHANNEL_MODEL.md:30,70`, `config.py:343`.
- The doc calls the excess U(0.3, 3.0) m "DERIVED" from sqrt(d^2 + 4h^2) - d. [D] At the mission depth h = 4.3 m: 5.8 m at d = 3.5, 4.1 m at d = 7, 2.0 m at d = 17.5. It reaches 0.3-3 m only near a boundary or at long range. Expected neighbours lie outside the bound `los_excess_max_m` = 3 m.
- A real boundary bounce at these ranges arrives at 51-68 deg elevation [D], and the depth gate would reject it if the array resolves elevation. The residual risk is in-plane reflectors (gate frame, other hulls), whose azimuth error can be tens of degrees, while the model limits it to 5 deg (`:223`). So the case the envelope must cover (a persistent, plausible, wrong arrival) is restricted by construction, and the B2-A1 coverage result cannot detect that.
- The clear-path episode (`:184-192`) takes the time of the bounce and the direction of the direct path, which is a stress construct, not propagation. 13.8 % of all receptions are in such an episode [M]; the sum of receptions is 51 111.
- Resolve: an image-source model (surface, bottom, bars, hulls) coupling excess and direction; re-run `calibration/analyze_b2_channel.py` and `formal/check_acoustic.py`. MEDIUM azimuth is 2.9 deg RMS [D] (2.5 white, 1.5 from the bias) against the field 7 deg (68 %) that the repository quotes (I did not check that source); a 7 deg case should be run.

### F6. MAJOR, correctly disclosed. H6 is not available as a small commercial product
`perception/payload.py:25-31`, `sim/beacon.py:66-74`, design note lines 74-76.
- [L] Water Linked M64 (92 g, the BlueROV2 class): 64 bit/s net, about 500 ms typical latency. One 64-bit packet takes 1 s, so a cycle of 3-6 s for 3-6 drones, over `status_fresh_s` = 2.5 s. [L] EvoLogics 120/180 series: up to 62.5 kbit/s, 120-180 kHz, 300 m. So the raw rate is not a physical limit, and the design note is right on that. But no source I read gives an 8-byte frame in 64 ms behind a 40 ms chirp, or any modem combining it with a calibrated 3-D array and a one-way timed range. "1 kbit/s net" also leaves no room for coding overhead.
- Self-noise is absent. [L] RoboVaaS (ahoi modems, T200 thrusters at 1.1 m from the hydrophone): PDR 90-98 % both ways at 0-75 m, and at 75-100 m 74 % vs 53 %, the deficit at the vehicle receiver attributed to thruster noise. MEDIUM's 24 % short-range loss is therefore harsher in magnitude; but loss does not depend on the thrust state of the receiver or of drones nearby.
- Resolve: keep H6 as a stated assumption in every claim; add a bench measurement before any field statement; consider tying loss to own thrust level.

### F7. MINOR. The failing bench check is a table artefact; it matters only for the low-end sweep
`sim/currents.py:20-24`, `config.py:279-282`, `probe_buoyant_ascent.py:113`. [D] Force 1.795 (c - v)^2 against 11.5 v reproduces every bench row: target 0.05 gives table command 0.553 and model 0.041 m/s (measured 0.040), target 0.10 gives 0.887 and 0.097 (0.095). The convex curve is interpolated linearly between 0.3 and 1.0. Harmless at the nominal value, but 0.05 m/s is where F1 points. Resolve: invert c = v + sqrt(6.41 v) (0.616 for 0.05 m/s) or add table points at 0.5 and 0.7; keep the check.

### F8. MINOR. A diffracted arrival keeps the clean packet-loss rate
`sim/acoustic_channel.py:212` returns `nlos` False, so `sim/beacon.py:163` applies 15 %, while its own comment and design note line 75 say a shadowed arrival suffers more. [D] A 40 ms chirp at 5 kHz has about 23 dB processing gain, a 1 kbit/s packet in the same band about 7 dB; a chirp that barely gets through will rarely carry a decodable packet. 1394 such receptions in med3 [M]. The bias is optimistic for liveness only (a lost status never excludes). Resolve: flag shadowed arrivals as NLOS for the payload.

### F9. MINOR. `lateral_far`: one-sided range excess pulls a drone towards its neighbour; no time alignment
`perception/perception.py:440`, `control/flows.py:157`. A multipath excess on a lateral neighbour makes it look farther and moves the reference towards it: [D] p99 +0.8 m with one lateral neighbour (multipath episodes at the model's rate), with lanes already at 2.6-3.5 m. The fix is 1.5 s old at most while `slot_err` is current: up to 0.75 m of lateral error during a lane change at 0.5 m/s [D]. The cap (2 m) and the untouched separation layer keep this a liveness matter. Resolve: require two neighbours or take a lower envelope of the range over a window; propagate the fix to t as `_rep_offset` does.

### F10. MINOR. CLEAN latency is physically inconsistent with the packet
`sim/acoustic_channel.py:87`: 0.10 s, while chirp plus packet last 0.104 s before the last bit arrives; LOW leaves 46 ms for decoding. It matters for the CLEAN and LOW regressions only. Resolve: latency of at least 0.104 s plus decoding when a payload is carried.

### F11. MINOR. CRC-16 is sound but never exercised in simulation
`sim/beacon.py:163-165` drops packets, it never corrupts them. CRC-16/CCITT over 48 bits is an adequate choice [D: undetected rate 2^-16 per corrupted packet, about 0.3 per hour for six drones at 20 % corruption]. The status is stored before any consistency check (`perception/acoustic.py:228-230`) and GP_PASSED has no freshness test; a wrong packet is overwritten at the next decode. Resolve: unit-level fault injection; note the residual rate in the report.

### F12. MINOR. RC2 is postponed, not solved; the payload consistency check goes vacuous
[M] NavBound at run end: median 4.8 m, maximum 14.4 m at 461 s, against the 15.4 m code limit (about 500 s of mission at that rate), median 5.3 and up to 35 times the true maximum error (nav error max 1.66 m). `perception/acoustic.py:241-242`: the reach is `rep.bound + b_own + ...`, which exceeds the 3-7 m link after a few minutes, so an inconsistent position is no longer rejected. Resolve: state the mission length for which the payload bound is representable.

### F13. MINOR. Silence bookkeeping corner cases
`perception/acoustic.py:297-300,308-310`. A drone never heard is never silent (`heard.get(slot, t)`); a sole survivor (every other drone dead) is "deaf" for ever and falls back to the B0 timer. Both are conservative and worth a sentence in the report. `docs/ACCEPTANCE_CRITERIA.md:29-31` (v8, "considered gone after 50 s") still reads as current.

### F14. MINOR. Passage branches beyond the crossing are not exercised
[M] 42 of 42 verified passages in med3 are "crossing"; the REAR and odometry branches of `perception/passage.py:127-136` were never used, so they have no MEDIUM evidence. The 120 deg sonar is a design choice with no product behind it (`docs/PARAMETER_PROVENANCE.md:94`); [L] the real Ping is 25 deg, 0.3 m minimum range, 115 kHz.

## Sound, and how I checked

- **Vertical current as the ascent.** Defensible as an engine device: with |c| much larger than v the force is nearly constant, 1.41 N at rest and 1.12 N at terminal [D]. Measured rise in campaign runs 0.095 m/s [M], independent of horizontal current within 0.094-0.099 [M, bench]. It emulates a speed, not the force or a real plant, and the drone is exactly neutral before failing (`BlueROV2.cpp:38`), so depth hold never carries the +2 N a ballasted vehicle needs.
- **Slot arithmetic.** 40 + 64 + 33 = 137 ms of 167 ms; `sim/beacon.py:66-74` enforces it. First-order bounces at fleet ranges arrive within 4 ms [D]. The reverberation tail in a closed basin was not estimated.
- **`lateral_far` within the model.** Signal: |e_i - e_k| at run end in 126 expected-neighbour pairs of formation and rejoin runs: median 0.61 m, p95 2.18, maximum 2.53 [M]. Reference error of the loop after 30 s with MEDIUM fix errors and a 2 deg compass bias: sd 0.13 m (one neighbour ahead), 0.23 m (one beside), 0.11 m (three) [D, Monte Carlo on `ChannelModel`]. Useful where the error exceeds about 0.5 m, as it is for the RC6 tail; steady-state gain 0.8 [D].
- **Crossing signature.** Physically credible for posts at 0.7 m in a 120 deg cone: the sum grows by 0.25 m (the tolerance) at |s| about 0.43 m [D], and detection occurs at true s = -0.43 to -0.16 m [M]. Measured lateral error rms 0.058 m, vertical 0.044 m [M]; verified at true s = 3.08-4.23 m, always far beyond the CR. With a 25 deg beam the window is still about +-0.3 m [D] and three 10 Hz captures fit. Cross-talk among six co-frequency sonars is not modelled.
- **Silence never clears a claim.** `acoustic_gate.py:166-171`: `status_claim` has no time term, and `no_fleet_drone` returns False on it. Read in the code.

## What the simulation does not represent; claims to avoid

Not represented: wave acoustics, temporal correlation of the channel (per link), thruster self-noise, Doppler, sound-speed structure, array calibration on a real hull and own-hull shadowing of the array, real sonar beam patterns and mutual interference of six same-frequency sonars per drone, real BlueROV2 hydrodynamics (engine drag is 100 times smaller than its own formula; surge force 14.1 r + 6.9 r^2 N in the repository's fit vs 13.7 r + 141 r^2 in von Benzon), wake and thruster-jet interaction at 3.5 m spacing, contact mechanics at the bar, partial failures, and a live but silent drone.

Claims a report should avoid:
- that the channel is "realistic" or "validated": MEDIUM is a stress model with assumed statistics;
- that 45 s or 70 s are physically justified: they are simulation-calibrated;
- that a real BlueROV2 rises at 0.10 m/s; that a failed drone stays pinned or leaves the CR "in 12.5 s";
- that a dead drone "cannot enter" the CR once silent;
- that the fleet needs only modems commercially available for small ROVs;
- that a 1 kbit/s, 64-bit TDMA frame with one-way range and bearing has been demonstrated; that the MEDIUM results bound real-water performance.

## Verified / not verified

Verified: the listed code at `c6d8c98` and the engine source (`holoocean-octree-patch`); all 54 runs of `results/robustness/raw_dev/med3` by script (reception, payload, vacancy, passage, navigation, dead-drone ascent, per-link fresh-fix gaps, hull shadowing from truth); 72 v10 runs (the 31 s claim); the arithmetic above.
Not verified: no simulation or engine run; the Monte Carlo uses the model, not the engine; `med4` was only sampled (12 runs complete at c6d8c98, none of the RC6 scenarios; `p1_two_lines` s921 is INCOMPLETE with slot error 15-16 m, not investigated); Rypkema 2022, SeaTrac, Subnero and the `+0.2 kg` deployment value; multipath tails in enclosed water; values read from plots are approximate.

## Sources read
- M. von Benzon et al., "An Open-Source Benchmark Simulator: Control of a BlueROV2 Underwater Robot", J. Mar. Sci. Eng. 10 (2022) 1898, Table A1, Table 5, Sec. 4.2.
- N. Morozs et al., "Statistical ON-OFF Link Modeling Based on Sea Trial Data", WUWNet 2024, doi:10.1145/3699432.3699468 (Figs. 2, 5, 6; Secs. 3.1-3.2).
- "System Architecture and Communication Infrastructure for the RoboVaaS project", arXiv:2206.11082 (HTML rendering, first 100 000 characters).
- Water Linked Modem M64 datasheet W-MK-19009-1 (2019). EvoLogics 120/180 product page. Blue Robotics Ping product page.
