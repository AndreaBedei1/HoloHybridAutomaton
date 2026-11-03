# One-way acoustic channel model (variant B2)

This document describes the acoustic beacon of variant B2 as it is modelled in the simulator (`holo_fleet/sim/acoustic_channel.py`, `holo_fleet/sim/beacon.py`) and as it is used onboard (`holo_fleet/perception/acoustic.py`, `holo_fleet/perception/acoustic_gate.py`, declared envelope `config.AcousticConfig`). Status: 2026-10-29, working tree of branch `b2-acoustic-aided`. Parameter provenance is in `docs/PARAMETER_PROVENANCE.md` (sections 7 and 8) and `config/parameter_sources.json`; citation keys refer to `references.bib`; R3 is the 2026-10-29 audit of this channel (`HoloHybridAutomaton_archive/provenance_work/R3_acoustic_b2.md`). Where this document differs from sections 2 to 4 of `docs/B2_ACOUSTIC_AIDED_COORDINATION.md`, this document is current.

## 1. What B2 transmits

- The link is **strictly one-way**. A drone emits a coded ping on a fixed TDMA schedule; nobody replies.
- The ping carries **identity only**: the static code of the pinger, assigned by the shared mission plan. There is **no payload**.
- There is **no handshake, no acknowledgement and no request-response**. A sender never learns whether it was heard.
- **No position, state, priority, mode, queue, occupancy or sensor data is transmitted.** In the HoloOcean `AcousticBeaconSensor` the fourth result field is the receiver's own depth, not the sender's; nothing of the sender's state is carried (`beacon.py`).
- The receiver measures range (one-way travel time against a disciplined clock) and direction (receiving array) itself. Inter-agent communication is otherwise off (`FleetConfig.comms_enabled = False`).
- **Hardware consequence (R3).** COTS one-way messages give a bearing only: the one-way message of the SeaTrac USBL variant delivers an incoming bearing but no range (field comment reproduced in a third-party driver header, `moosivp_seatrac_types`), and range needs a request-response exchange. Every verified commercial micro-USBL or modem is two-way or a topside short-baseline system, and none documents synchronised-clock one-way ranging. A strictly one-way range-bearing system therefore needs research-grade hardware (section 8).

## 2. What the HoloOcean engine provides

Read in the patched engine source (`AcousticBeaconSensor.cpp`) and confirmed by `results/calibration/beacon_api_probe.json` (all four values to the millimetre); documentation: `HoloOceanDocs230`.

- For each (sender, receiver) command the engine writes `[azimuth, elevation, distance, depth]`: azimuth and elevation of the sender's sensor in the receiver's sensor frame (body frame, roll and pitch included; x forward, y left, z up), the distance, and the receiver's own depth. The values are exact; an optional noise exists on `MaxDistance` only.
- All four fields are -1 when the distance exceeds `MaxDistance` or when the straight segment between the two sensors first hits another actor (`CheckVisible`: one ray, any hull or prop blocks it).
- The result appears after round(d / (dt x 1500)) ticks (0 or 1 tick below 50 m). A receiver holds one incoming message per tick: two simultaneous senders overwrite each other, and the client's own message bookkeeping cancels a broadcast after one failed reception. `beacon.py` therefore enqueues the engine command itself and reads the raw buffer.
- These values are the geometric truth of the direct path. They are never handed to a controller; `acoustic_channel.py` turns them into what a receiver would deliver.

## 3. Hardware assumptions H1 to H5

| | Assumption | In the model | Evidence and verdict |
|---|---|---|---|
| H1 | Static identity on a fixed TDMA schedule: drone k pings in slot k of every period (1.0 s) at t = m x period + k x period / n | slots are aligned to the 30 Hz simulator ticks: slot = period_ticks // n, i.e. 500, 333, 233, 200, 167 ms for n = 2 to 6; a coded waveform per drone (an LFM chirp or spread-spectrum code), duration 20 to 40 ms, bandwidth of 5 kHz or more for about 0.15 m resolution | SUPPORTED (period): one-way beacon at 1 Hz, PPS-triggered, four distinguishable chirps (`rypkema2022synchronous`); 4 Hz sweeps (`jakuba2021performance`). DERIVED (collision freedom): the slot exceeds the 33 ms propagation spread at 50 m plus a 20 to 40 ms ping; TDMA needs good synchronisation (`chitre2008underwater`); collisions grow with the number of transmitters (`binder2016spatial`) |
| H2 | Synchronised clocks: one-way travel time (OWTT) ranging with a CSAC or OCXO disciplined to GNSS PPS at the surface and never power-cycled during the dive. The onboard sound speed c_assumed is computed from measured temperature and depth and a known salinity (Mackenzie or UNESCO); the engine's 1500 m/s is not the onboard value | range = c_assumed x (t_arrival - t_scheduled). Errors: a constant clock offset per drone (a range bias between two drones), one sound-speed scale factor per run (the water mass is shared), white detection jitter that includes the sender's trigger jitter | SUPPORTED (`microsemi_sa45s`, `eustice2011synchronous`, `rypkema2022synchronous`); clock figures in section 4; sound speed: `mackenzie1981nine`, `mbari_sndspd` |
| H3 | A small calibrated 3-D inverted-USBL receiving array on every drone (4 to 6 hydrophones, synchronous multichannel sampling clocked from the same time base, matched filtering, direction of arrival by phase or beamforming) and an AHRS fixed to the array | direction of arrival in the receiver's body frame: white azimuth and elevation noise plus a smooth direction-dependent calibration bias fixed per receiver, b(az) = A (sin(az + p1) + sin(2 az + p2)) / 2 per axis (`doa_bias_deg` = A). The receiver's own attitude error is applied onboard | SUPPORTED as feasible research hardware, not a COTS feature: a four-element 38 mm array (`jakuba2021performance`), 5-element pyramid (`rypkema2022synchronous`), 6 elements on a 0.13 m circle (`huang2025raspi2usbl`, preprint). The array must be 3-D: both angles are much noisier below about 30 deg array-frame elevation, and errors are bias-dominated |
| H4 | First-arrival detection of the coded ping | clear path: the direct arrival, except during a multipath episode. Ray blocked by a drone hull or a gate bar (bodies comparable to the first Fresnel zone, section 6): SHADOWED, lost with probability `shadow_loss`, else the diffracted direct arrival, at most `shadow_extra_m` longer and with `shadow_dir_deg` more direction noise. Ray blocked by anything else (a large prop, terrain): usually lost; with probability `nlos_rx` a reflected arrival, longer by U(0.3, 3.0) m, azimuth error N(0, 0.2 x `reflect_dir_deg`) and elevation error N(0, `reflect_dir_deg`): a contradictory measurement. Clear-path multipath: with probability `los_outlier` per reception an episode starts on the link and lasts `los_outlier_pings` receptions on average; the arrival time is that of a boundary bounce, longer by a fixed excess drawn from `reflect_extra_m`, while the array still resolves the direct path | Shadowing: the Fresnel premise is DERIVED (section 6), the loss probability a STRESS ASSUMPTION. Reflections: the extra path is DERIVED from boundary geometry (surface or bottom bounce: sqrt(d^2 + 4 h^2) - d), the probability and the direction error are STRESS ASSUMPTIONS. Multipath episodes: phenomenon reported in the field (`webster2012advances`, `olson2006robust`, `rypkema2022synchronous`, `renner2020ahoi`), probability a STRESS ASSUMPTION; the earlier premise "clear path, no range outlier" was CONTRADICTED (R3 P7) |
| H5 | Processing latency (detection, beamforming) | delivered `latency_s` after the arrival and stamped with the arrival time; t_arrival = t_tx + d / 1500 | STRESS ASSUMPTION inside the published 8 ms to 0.5 s bracket (`waterlinked_m64`, `huang2026precise`, `rypkema2022synchronous`) |
| - | Channel | per-reception dropout growing linearly with range to `dropout_far` at 50 m; burst outages (interference). Two arrivals never collide at a receiver: the slots are far wider than the propagation spread. `beacon.py` still loses all pings whose engine results can share one receiver buffer (a 2-tick window, 67 ms); with slots of 167 ms or more this never triggers | dropout magnitudes PARTIALLY SUPPORTED by the ahoi marina data (`renner2020ahoi`); outages STRESS ASSUMPTION; the unused `guard_s` field of earlier versions was removed |

## 4. Clock drift against range error

A constant fractional frequency offset y between two clocks gives a time error y t and a range error c y t (c = 1500 m/s); per part per billion over 1 h this is 5.4 mm. Values for t = 3600 s (derived from the figures in R3 P3; sources in the last column):

| Clock case | y | Time error after 1 h | Range error after 1 h | Source |
|---|---|---|---|---|
| CSAC disciplined to PPS (datasheet, 1.0E-12) | 1e-12 | 0.0036 us | 0.0054 mm | `microsemi_sa45s` |
| CSAC, temperature step deck to water (datasheet bound, 5E-10) | 5e-10 | 1.8 us | 2.7 mm | `microsemi_sa45s` |
| CSAC typical holdover (below 100 us per 24 h) | 1.16e-09 | 4.17 us | 6.25 mm | `rypkema2022synchronous` |
| CSAC (about 1 ppb) | 1e-09 | 3.6 us | 5.4 mm | `jakuba2021performance` |
| OCXO Trimble Mini-T (2.8 ns/s) | 2.8e-09 | 10.1 us | 15.1 mm | `eustice2011synchronous` |
| OCXO of the open-source receiver (below 180 us per 12 h, preprint) | 4.17e-09 | 15 us | 22.5 mm | `huang2025raspi2usbl` |
| TCXO (Seascan), bench maximum (below 300 us per 24 h) | 3.47e-09 | 12.5 us | 18.8 mm | `eustice2011synchronous` |
| TCXO (Seascan), typical specification (20 ns/s) | 2e-08 | 72 us | 108 mm | `eustice2011synchronous`, `jakuba2021performance` |

The initial synchronisation (+-100 ns) adds 0.15 mm; the Allan deviation of the CSAC (2.5e-10, 8e-11, 2.5e-11, 8e-12 at 1, 10, 100, 1000 s) is negligible at 1000 s; aging (below 9e-10 per month) changes the frequency by about 1e-12 in one hour, so the drift is linear over a one-hour mission. The channel levels model this as a constant offset per drone and run (1 sigma): CLEAN 0.01 m (6.67 us), LOW 0.03 m (20 us), MEDIUM 0.05 m (33.3 us). The constant-per-run structure overbounds a disciplined CSAC; for a TCXO at its 20 ns/s typical specification the end-of-mission offset (0.11 m per drone, up to 0.22 m between two drones drifting in opposite directions) exceeds the MEDIUM 1-sigma and reaches the 0.21 m envelope bound. Because the bias changes by about 1.5 um between two consecutive fixes for y = 1e-9, the onboard assumption that biases cancel between fixes (`AcousticTracker.consistent`) is sound. For missions beyond one hour or TCXO-class clocks the model needs offset plus drift.

Sound speed: dc/dT = 4.08, 3.60, 3.16, 2.75 m/s per deg C at 5, 10, 15, 20 deg C and dc/dS = 1.14 to 1.29 m/s per psu (Mackenzie 1981, salinity 35, 5 m depth; coefficients from a secondary transcription). A residual of 0.3 % (4.5 m/s) corresponds to about 1.4 deg C at 15 deg C or 3.8 psu; 1 % to about 4.7 deg C or 12.5 psu. Measured sound speeds in the opened field papers span 1426 to 1481 m/s, so an uncompensated 1500 m/s would be off by 1.3 % (river) to 5.2 % (marina): beyond MEDIUM and beyond the onboard 3 % bound. This is why H2 requires the onboard estimate.

## 5. Channel levels

Values are read from `acoustic_channel.py`. CLEAN, LOW and MEDIUM are the levels of the validation campaign; BLACKOUT delivers nothing and tests the fall-back to B0. The onboard perception never knows the level of a run. The last column is the verdict of `docs/PARAMETER_PROVENANCE.md` section 7.

| Field | Unit | CLEAN | LOW | MEDIUM | BLACKOUT | Verdict |
|---|---|---|---|---|---|---|
| `sigma_range_m` | m | 0.05 | 0.1 | 0.15 | - | PARTIALLY SUPPORTED |
| `sound_speed_err` | relative | 0.003 | 0.005 | 0.01 | - | DERIVED |
| `clock_offset_m` | m | 0.01 | 0.03 | 0.05 | - | SUPPORTED |
| `sigma_az_deg` | deg | 1 | 1.5 | 2.5 | - | PARTIALLY SUPPORTED |
| `sigma_el_deg` | deg | 1 | 1.5 | 2.5 | - | PARTIALLY SUPPORTED |
| `doa_bias_deg` | deg, amplitude | 0.5 | 1.5 | 3 | - | STRESS ASSUMPTION |
| `dropout` | probability | 0.02 | 0.08 | 0.2 | - | PARTIALLY SUPPORTED |
| `dropout_far` | probability at 50 m | 0.1 | 0.25 | 0.5 | - | PARTIALLY SUPPORTED |
| `outage_every_s` | s, mean period | 0 | 0 | 40 | - | STRESS ASSUMPTION |
| `outage_len_s` | s | 0 | 0 | 4 | - | STRESS ASSUMPTION |
| `nlos_rx` | probability | 0.1 | 0.2 | 0.35 | - | STRESS ASSUMPTION |
| `reflect_extra_m` | m, uniform | (0.3, 3) | (0.3, 3) | (0.3, 3) | - | DERIVED |
| `reflect_dir_deg` | deg, elevation std (azimuth 0.2 x) | 25 | 25 | 25 | - | STRESS ASSUMPTION |
| `shadow_loss` | probability | 0.3 | 0.5 | 0.7 | - | STRESS ASSUMPTION |
| `shadow_extra_m` | m, uniform up to | 0.1 | 0.1 | 0.1 | - | DERIVED |
| `shadow_dir_deg` | deg | 0.5 | 1 | 1.5 | - | STRESS ASSUMPTION |
| `los_outlier` | probability per reception | 0 | 0.01 | 0.03 | - | CLEAN 0: CONTRADICTED; LOW, MEDIUM: STRESS ASSUMPTION |
| `los_outlier_pings` | receptions, mean | 5 | 5 | 5 | - | STRESS ASSUMPTION |
| `latency_s` | s | 0.1 | 0.15 | 0.25 | - | STRESS ASSUMPTION |
| `blackout` | - | False | False | False | True | DESIGN CHOICE |

Anchors (R3): range resolution +-0.1 m of a commercial micro-USBL modem, which is a two-way specification (about +-0.14 m one-way for the same detector); bearing 1 deg RMS (MicronNav 200, limited by its heading sensor) to +-3 deg (older sheets); a low-cost one-way inverted-USBL fleet at 1 Hz reported, in a static test at about 30 and 57 m in a river, 68 % of range errors below 0.7 m and 68 % of azimuth errors below 7.0 deg with a direction-dependent bias (`rypkema2022synchronous`). Our ranges are 2 to 20 m in open water. CLEAN and LOW sit near the datasheet values; MEDIUM is at the datasheet end and still below the field figures (open issue O-11 in the provenance document). For MEDIUM (A = 3.0 deg, white 2.5 deg) the 68 % quantile of the azimuth error is at most 4.2 deg (normal distribution, computed here).

## 6. Fresnel zone and the corrected Rypkema citation

**First Fresnel radius at mid-path**, r1 = 0.5 sqrt(lambda d) with c = 1500 m/s:

| Path length d [m] | 20 kHz (lambda 7.5 cm) | 30 kHz (lambda 5.0 cm) |
|---|---|---|
| 1 | 0.14 m | 0.11 m |
| 3 | 0.24 m | 0.19 m |
| 5 | 0.31 m | 0.25 m |
| 10 | 0.43 m | 0.35 m |
| 20 | 0.61 m | 0.50 m |

The earlier statement of 0.3 to 0.5 m for 3 to 10 m overstated the radius at 3 m. The BlueROV2 hull box (0.458 x 0.576 x 0.254 m, `HULL_HALF`) is comparable to r1 at 3 to 5 m and smaller beyond: an obstruction is partial and diffraction can fill the shadow, so a hull or bar shadows the direct path rather than cutting it. The loss in dB is not calibrated by any opened source; an air-filled electronics tube reflects strongly, so the shadow need not be weak (physical reasoning only). For an edge offset a = 0.23 m at mid-path, the diffracted arrival is longer by 2 (sqrt(d^2 / 4 + a^2) - d / 2): 0.10 m at d = 1 m and 0.035 m at d = 3 m, so `shadow_extra_m` = 0.10 m is conservative from 1 m.

**Rypkema, Schmidt and Fischell (2022)** (`rypkema2022synchronous`): the statistics 68 % of range errors below 0.7 m and 68 % of azimuth errors below 7.0 deg come from the *static calibration on a rotating rig at about 30 m and about 57 m in a river*, with a CSAC, a five-hydrophone array and four 20 ms LFM chirps between 6 and 11 kHz (about 2 kHz bandwidth). They are not figures for 30 to 300 m, which is how earlier text of this repository quoted them; the operating area was about 300 x 200 m. Navigation against LBL was 68 % below 7.89 m, and the beacon-to-AUV range generally stayed below 100 m. The values were read in the arXiv HTML rendering, which duplicates digits; check them in the journal PDF before quoting.

## 7. Onboard declared envelope and how each phenomenon is handled

The perception uses the declared envelope `config.AcousticConfig`, fixed before any B2 result. It is the MEDIUM channel at k_sigma = 3, plus the own attitude error and a one-sided range excess:

| Field | Value | Basis |
|---|---|---|
| `k_sigma` | 3 | Gaussian: P(abs(z) > 3) = 0.27 % per axis; heavy tails make the real exceedance higher |
| `sigma_range_m` | 0.15 | MEDIUM channel |
| `sound_speed_err` | 0.03 | 3 x the MEDIUM 1 % (one scale factor per run); requires the onboard c estimate |
| `clock_offset_m` | 0.21 | 3 sqrt(2) x the MEDIUM 0.05 m: the bias between two drones' clocks |
| `sigma_az_deg` | 2.5 | MEDIUM channel |
| `sigma_el_deg` | 2.5 | MEDIUM channel |
| `doa_bias_deg` | 3 | MEDIUM amplitude of the array calibration bias (new since R3) |
| `los_excess_max_m` | 3 | upper bound of reflect_extra_m; one-sided: the region reaches this much nearer (new since R3) |
| `heading_err_deg` | 3 | own compass bias (MEDIUM 2.0 deg) plus a 1.0 deg allowance for noise (open issue O-8) |
| `tilt_err_deg` | 3 | own unestimated roll and pitch of a BlueROV2 holding depth (assumption) |
| `latency_max_s` | 0.4 | MEDIUM latency 0.25 s plus margin; inside the 8 ms to 0.5 s bracket |
| `nav_err_gate_m` | 1 | own position error in the gate frame, bounded by aiding (open issue O-6) |
| `track_gate_m` | 0.5 | extra slack of the kinematic consistency gate of a track |
| `exit_confirm` | 3 | 3 not 2: persistent reflections (section 7) |
| `tie_window_s` | 3 | fixes this close to the last corridor sighting account for a drone |
| `fresh_s` | 2.5 | a fix this recent counts for the neighbours (formation): one lost ping plus the latency |
| `depth_tol_m` | 1.5 | a fix this far outside the operating depth band is a reflection |

A reception becomes a **fix**: the relative position r u(az, el) in the own body frame, rotated with the own heading estimate of the arrival instant (pose history) into the map frame, plus an **error region** sampled 7 x 7 x 7 over the box (range r = 3 m: range +-0.75 m, r = 10 m: range +-0.96 m, r = 20 m: range +-1.26 m, plus 3 m nearer; azimuth +-13.5 deg; elevation +-13.5 deg), with a slack of half the diagonal of a grid cell (formal B2-A1). The range interval is one-sided: it reaches `los_excess_max_m` nearer than the measured range, because a multipath episode can only make the range too long. Rejected before use: a reception delivered more than `latency_max_s` after its arrival; a fix whose nominal depth lies more than `depth_tol_m` outside the operating depth band (a surface or bottom bounce); a fix that is kinematically inconsistent with the previous accepted fix of the same identity, i.e. a 3-D displacement beyond v_bound dt + 3 sqrt(2) sigma_white + `track_gate_m`, or a range change beyond v_bound dt + own displacement + 3 sqrt(2) sigma_range + 0.15 m (a reflection is always longer). The biases of the envelope are common to two fixes and cancel in the gate, which budgets the white part only. A rejected fix is kept as contradictory evidence; two mutually consistent rejected fixes, with no accepted fix for 3 s, start a new track. **Absence of fixes is never evidence.**

| Phenomenon | Model | Onboard handling | Residual |
|---|---|---|---|
| White range and direction noise | sigma_range_m, sigma_az_deg, sigma_el_deg | inside the box at 3 sigma | Gaussian tails beyond 3 sigma (0.81 % per reception for one of three axes) |
| Clock offset | constant per drone and run | box term 0.21 m = 3 sqrt(2) x 0.05 m; cancels between two fixes | TCXO-class clocks over long missions |
| Sound-speed scale | one factor per run | box term 3 % of the range; requires the onboard c estimate (H2) | an uncompensated 1500 m/s violates it |
| Array calibration bias | smooth in direction, fixed per receiver, abs(b) <= `doa_bias_deg` | box term `doa_bias_deg` in azimuth and elevation; treated as common to consecutive fixes (db/d(az) is at most 1.5 A per radian, so about 0.08 m at 10 m for a 0.1 rad change at MEDIUM, computed here) | the change between fixes is not budgeted separately beyond `track_gate_m` |
| Own heading and tilt error | compass bias and noise, unestimated roll and pitch | box terms `heading_err_deg`, `tilt_err_deg` | the noise allowance is below 1 sigma at MEDIUM (open issue O-8) |
| Shadowed ray (hull or bar) | lost or diffracted, up to +0.10 m | inside the box; `RANGE_SLACK_M` = 0.15 m in the range test | none beyond the box |
| Reflected arrival on a blocked path | +0.3 to +3.0 m, elevation error 25 deg | rejected by the depth check (a bounce arrives from above or below) and by the kinematic gate, in particular range continuity; contradictory fixes are remembered | reflections that pass both tests repeatedly on one track: `exit_confirm` = 3 consecutive fixes for an acoustic exit |
| Clear-path multipath episode | fixed excess of +0.3 to +3.0 m for a few receptions, direct direction | one-sided range extension `los_excess_max_m` = 3.0 m. The kinematic gate cannot reject an established episode (a constant excess cancels between fixes); a range jump at its onset beyond the continuity budget (about 1.5 m plus the own displacement for dt = 1 s, computed here) is rejected | the region is large: see section 9 |
| Dropout, outage, blackout | independent losses, bursts, none at all | nothing is inferred from silence; B0 behaviour | none |
| Latency | `latency_s` after the arrival, arrival-time stamp | fix placed with the pose of the arrival instant; stale after `latency_max_s` = 0.40 s | latency above 0.40 s |
| Collision | none by the TDMA design | none | none |

Residual risk outside the envelope (reflections that pass the gates twice in a row on one track) was measured with `calibration/analyze_b2_channel.py`, 20 000 trials per level, an occupant moving inside the CR, half of the trials with the direct path persistently blocked so that every received ping is a reflection (a stress beyond the gate scenarios). A wrong "outside the CR" decision occurred 9, 58 and 129 times in about 65 000 receptions (CLEAN, LOW, MEDIUM) with `exit_confirm` = 2 and 0, 3 and 3 times with 3, the value adopted; a wrong "beyond the exit face" decision never occurred (0 in about 200 000). These figures were measured before the envelope was widened (`doa_bias_deg`, `los_excess_max_m`) and were not re-run for this document.

## 8. Hardware realisation of a strictly one-way system (R3)

Verified commercial products (all two-way or topside short baseline; accuracy figures are two-way specifications and are used here only as orders of magnitude):

| Product | Ranging | Relevant figures | Key |
|---|---|---|---|
| Blueprint Subsea SeaTrac X150, X110, X010 | two-way (request-response, 16 kHz timer); one-way messages give bearing only | 24 to 32 kHz, range resolution +-0.1 m (depends on the velocity of sound supplied), X150 angular resolution typically 2 % of range (about +-1 deg), 1 km, 15 identifiers | `blueprint_seatrac`, `moosivp_seatrac_types` |
| Tritech MicronNav 200 | two-way transponder or wired-trigger responder | 20 to 28 kHz, +-0.2 m, bearing 1 deg RMS limited by the heading sensor, topside head | `tritech_micronnav200`, `videoray_micronnav2017` |
| Sonardyne Micro-Ranger 2 Integrator | surface transceiver interrogates transponders | 20 to 34 kHz, below 995 m, up to 3 Hz, array below 3.5 % and system below 5 % of slant range (1DRMS) | `sonardyne_microranger2` |
| Water Linked Underwater GPS G2 | topside short baseline with a 4-receiver antenna | range below 0.2 %, angle below 1 deg, 2 to 4 Hz (figures from a product-page extraction, not a datasheet) | `waterlinked_ugps_g2`, `waterlinked_ugps_g2_antenna` |
| Water Linked Modem M64 | communication modem, no ranging function stated | 64 bit/s, about 500 ms typical latency, 200 m | `waterlinked_m64` |

EvoLogics S2C USBL could not be verified and is not used. Research systems with a PPS-synchronous one-way mode: WHOI Micro-Modem (time of arrival to within 125 us, `eustice2011synchronous`), Micromodem-2 with a 4-element array (22.5 to 27.5 kHz, post-processing only, `jakuba2021performance`), the SandShark system of `rypkema2022synchronous` (and its single-transmitter precursor `rypkema2017oneway`, abstract verified only), and an open-source Raspberry Pi receiver (`huang2025raspi2usbl`, preprint; tank 1 to 23 m).

Required per drone, every drone being both pinger and receiver: (1) a chip-scale atomic clock (SA.45s class: below 120 mW, below 17 cc, below 35 g, warm-up below 180 s) or an OCXO, disciplined to GNSS PPS at the surface; (2) an omnidirectional transducer and power amplifier whose emission is hardware-triggered by the disciplined PPS at the slot instant, with trigger jitter much smaller than 30 us (5 cm) and a calibrated constant delay, and a coded waveform that is the static identity; (3) a 3-D array of 4 to 6 hydrophones with synchronous sampling, matched filtering, first-arrival detection and array calibration; (4) an AHRS rigidly fixed to the array; (5) onboard sound speed from temperature, depth and a known salinity; (6) a channel plan with a slot of at least the ping duration plus 33 ms plus a guard, and transmit-receive blanking (the ahoi turnaround is 12 ms). B2 therefore assumes beyond COTS hardware: a custom OWTT receiver; clocks and transmit chains calibrated so that the per-drone offset stays within the figures of section 4; a calibrated 3-D array reaching the error levels with neighbours near the horizontal plane, a geometry where published planar arrays are worst; and onboard sound-speed compensation. Feasibility evidence is class (b), prototypes on small AUVs and gliders; the CLEAN and LOW magnitudes at 2 to 20 m between moving BlueROV2-class vehicles are not demonstrated in the field.

## 9. Consequence of the formal check (B2-M4g)

`formal/check_acoustic.py` (results saved in `formal/results/acoustic.json` and the aggregated `SUMMARY.md`, 2026-10-29 22:59) checks the envelope and the decision rules of `perception/acoustic_gate.py`:

- **B2-A1** envelope coverage holds: 4000 random measurements at 0.5 to 30 m, worst margin -0.017 m (must be 0 or below).
- **B2-A2/A3** decision soundness holds: of 121 500 cases (CR grid x envelope corners, every abreast queue point n = 2 to 6, headings within the tolerance), none passes the exit, outside or exclusion test while a hull is in the CR; the closest case still lies 4.61 m short of the exit face.
- **B2-S** structural check holds: no module that decides separation, collision avoidance or precedence reads an acoustic fix, and the latch calls the acoustic exit only with the FRONT corridor empty.
- **B2-M4g** (geometric availability of the acoustic exit evidence: `exit_confirm` = 3 consecutive fixes beyond the exit face before the end of the passer's path, worst envelope error, worst ping phase, occlusion by bars and queued hulls) is **violated inside the validated gate domain**. With the envelope that covers the clear-path multipath excess (`los_excess_max_m` = 3.0 m) and the array bias, the error regions are too large to certify an exit: for the abreast queues n = 2 and n = 3 every observer-passer pair reports "no evidence", and for n = 3 several observers also cannot account for the other queued drones. **B2 therefore falls back to the sonar latch in the validated gate geometry; no acoustic exit is claimed there.** For n = 4 to 6 the planner refuses the passage (queue points beyond `GateRule.queue_lateral_max` = 3.5 m) and the result is reported as the domain boundary.
- B2 still **blocks the `t_occ_max` fallback while an occupant is heard inside the CR** (`AcousticExit.heard_inside`: a fresh accepted fix places some drone possibly inside the CR, its region grown by the motion since the fix). This only adds caution; it never moves the latch while a corridor echo is seen, never touches precedence, separation or collision avoidance, and its absence means nothing.

The check reports availability, not safety: the B2-M4g rows are recorded as reported violations (expected and observed "violated"), not as failures of the suite, and where the acoustic evidence does not exist the latch is the sonar one, that of B0.
