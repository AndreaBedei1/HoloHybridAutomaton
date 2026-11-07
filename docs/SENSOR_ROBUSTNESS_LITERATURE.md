# Literature notes: sensor degradation and one-way beaconing for a BlueROV2-class UUV fleet

Compiled 2026-10-22. Every entry below was checked against at least one real web source.
"Verified via" lists what was actually opened. Bibliographic data (authors, venue, volume,
pages) were cross-checked against the Crossref API (api.crossref.org/works/<DOI>) unless
stated otherwise. Where only the abstract was available, the summary says so and no
full-text numbers are given. Quotations are kept under 15 words.

Legend
- **Q** = quantitative value actually stated in the source (with its context).
- **S** = qualitative support only.
- **UNVERIFIED** = could not be checked; do not cite as fact.
- Datasheet values describe the vendor's product under the vendor's test conditions. They are
  NOT calibration data for our vehicles (we have none); use them only as orders of magnitude
  or as justification for parameter ranges in simulation.

---------------------------------------------------------------------------------------------

## 1. Seed works

### S1. Li, Chitre, Stojanovic (2025) - Underwater acoustic communications
- Citation: Z. Li, M. Chitre, M. Stojanovic, "Underwater acoustic communications,"
  *Nature Reviews Electrical Engineering*, vol. 2, no. 2, pp. 83-95, 2025 (published online
  4 Dec 2024).
- DOI: 10.1038/s44287-024-00122-w
- Verified via: Crossref metadata; full-text PDF hosted by NUS ARL
  (https://arl.nus.edu.sg/wp-content/uploads/2025/03/Underwater-acoustic-communications.pdf),
  pages 1-4 and extracted text read.
- Summary: Review of the acoustic channel (propagation loss, frequency-dependent absorption,
  ambient noise, multipath delay spread, Doppler), signal processing for communication, and
  network protocols. Stresses that long propagation delay makes terrestrial MAC/ARQ
  designs (carrier sensing, acknowledgements) unsuitable underwater. Explicitly notes that no
  standardized statistical channel model exists and that the channel is non-stationary.
- Q:
  - Sound speed "about 1,500 m s-1"; propagation delay "a full second over 1.5km, or 2s for a round trip".
  - Delay spreading "over tens of milliseconds" (abstract); coherence time "around 100ms" (cited from ref. 10).
  - RF waves "can travel a few metres at about 10kHz"; blue-green optical links
    "cannot travel further than about 100m, depending on the clarity of water".
  - Bandwidth vs. distance example: 100 km single hop "about 1kHz of bandwidth"; 10 x 10 km relays "about 10kHz".
  - Ambient noise PSD decreases "at approximately 17dB per decade" above 1 kHz.
- S: Multipath (surface/bottom reflections), random fading from surface motion, Doppler
  from platform motion; channel statistics hold only over limited time windows.
- Can justify: modelling acoustic messages with travel-time latency (~0.67 ms/m); treating
  the acoustic channel as non-stationary with fading (random packet/ping loss); arguing
  against handshake/ACK-based protocols in favour of one-way broadcast; bounding optical
  links to short range (<~100 m even in clear water).

### S2. Ferreira, Graca, Alves, Cruz (2023) - Single-receiver localization of an unsynchronized periodic beacon
- Citation: B. M. Ferreira, P. A. Graca, J. C. Alves, N. A. Cruz, "Single Receiver Underwater
  Localization of an Unsynchronized Periodic Acoustic Beacon Using Synthetic Baseline,"
  *IEEE Journal of Oceanic Engineering*, vol. 48, no. 4, pp. 1112-1126, Oct. 2023.
- DOI: 10.1109/JOE.2023.3275611 (resolves to IEEE Xplore document 10186022)
- Verified via: Crossref metadata; abstract via Semantic Scholar API. Full text not accessible
  (paywalled); IEEE page returned no content to the fetcher.
- Summary (abstract only): 3-D localization of a stand-alone acoustic beacon using a single
  receiver on a surface vehicle and the synthetic-baseline principle. Purely passive: "no
  explicit synchronization, interaction, or communication with the acoustic beacon." The beacon's
  periodicity (not a shared clock) is exploited; times of arrival are referenced to GNSS time.
  Validated in a real search for an underwater glider lost in September 2021 off Portugal.
- Q: none available from the abstract (ping period, ranges, accuracy: UNVERIFIED).
- Can justify: that a periodic, unsynchronized "presence" ping (no handshake, no payload)
  still carries usable timing information if the receiver knows/estimates the period and
  integrates arrivals over multiple positions; supports a passive-listener design.

### S3. Rypkema, Schmidt, Fischell (2022) - Synchronous-clock range-angle relative acoustic navigation
- Citation: N. R. Rypkema, H. Schmidt, E. M. Fischell, "Synchronous-Clock Range-Angle Relative
  Acoustic Navigation: A Unified Approach to Multi-AUV Localization, Command, Control, and
  Coordination," *Field Robotics*, vol. 2, pp. 774-806, 2022.
- DOI: 10.55417/fr.2022026 (open access; arXiv:2110.13825)
- Verified via: Crossref metadata; full arXiv text downloaded and searched.
- Summary: A single beacon broadcasts periodically (GPS PPS-triggered); each AUV carries a
  chip-scale atomic clock and a 5-hydrophone USBL array, giving one-way travel-time range plus
  azimuth/inclination to the beacon. Vehicles never transmit; the operator selects behaviours
  by choosing one of four broadcast waveforms, so command is "one-way" and scales to any number
  of receivers. Coordination arises from behaviours defined in a beacon-centric frame.
  Validated with three SandShark AUVs (no DVL) in a river, against an LBL system.
- Q:
  - Beacon "repeat rate of the beacon at 1 Hz"; particle filter with 500 particles.
  - Broadcast signals: four LFM "20 ms chirps at 7-9 kHz, 10-8 kHz, 8-6 kHz and 9-11 kHz".
  - Calibration at ~30 m and ~57 m: "68% of acoustic range MLEs have an absolute error of less than 0.7 m"; 68% of azimuth errors below 7.0 deg.
  - Navigation vs LBL over six missions: "68% of the difference in solutions fell below 7.9 m".
  - Without DVL, dead-reckoning error grows "approximately 2% - 10% of distance traveled".
  - Heading error std "approximately 3" deg after magnetometer/IMU calibration (vs dual-antenna GPS).
  - Beacon-AUV range "generally remains under 100 m" in their experiments.
- S: Reflections/reverberation produce multiple peaks in range pseudo-distributions, needing
  temporal filtering "to discard false maxima"; partial loss of signal on some array elements
  produces range outliers.
- Can justify: presence/command via a small set of distinguishable waveforms; 1 Hz ping rate
  as a realistic order for short-range relative navigation; heavy-tailed range/bearing
  errors (false maxima) rather than pure Gaussian; DR drift of a few % of distance for
  DVL-less small AUVs; ~3 deg heading std as an order of magnitude after calibration.

### S4. Sharma, Van Passen, Prasad, Chowdhury (2025) - Pisces: low-power LED 3D localization
- Citation: S. Sharma, D. Van Passen, R. V. Prasad, K. Chowdhury, "Low power, non-intrusive 3D
  localization for underwater mobile robots," *Communications Engineering*, vol. 4, art. 93, 2025.
- DOI: 10.1038/s44172-025-00422-5 (open access)
- Verified via: Crossref metadata; full-text PDF (genesys-lab.org mirror, identical DOI/footer
  "Communications Engineering | (2025)4:93") extracted and read.
- Summary: Each robot carries two blue 455 nm LEDs blinking at robot-specific frequencies
  (ID by frequency); the receiver is a tri-photodiode (OPT101) array, and 3D position comes
  from angle of arrival and received-signal-strength ratios, decoded with a Goertzel filter.
  Training-free; tested on a controlled rig in a 3 m diameter indoor pool with turbid water.
- Q:
  - MAE "0.031 m at 0.32 m separation and 0.16 m MAE at 1 m separation".
  - "11 mA of transmitter LED current"; localization "within 10 ms for distances up to 3 m".
  - Receiver FOV "around 60 deg at 0.15 m and 95 deg horizontally at >1 m".
  - Photodiode bandwidth "capped at 1.8 kHz"; LED IDs must differ by at least 50 Hz;
    max localization rate 40000/400 = 100 Hz.
  - Tested distances 0.32-3.0 m; receiver at 0.25 m depth.
  - Background: 450 nm least attenuated in open ocean, shifting to 540 nm in turbid water (cited).
- S: Requires line of sight; SNR and accuracy degrade with distance; multipath and
  reflective noise visible in measurements; limited elevation coverage at larger distances.
- Can justify: optical relative-localization range of only a few metres for small,
  low-power beacons; limited FOV (tens of degrees) and LOS requirement; frequency-coded IDs as
  a collision-avoidance mechanism for simultaneous optical beacons.

### S5. Vallicrosa et al. (2016) - Homing and docking with range-only localization and light beacons
- Citation: G. Vallicrosa, J. Bosch, N. Palomeras, P. Ridao, M. Carreras, N. Gracias,
  "Autonomous homing and docking for AUVs using Range-Only Localization and Light Beacons,"
  *IFAC-PapersOnLine*, vol. 49, no. 23, pp. 54-60, 2016 (IFAC CAMS 2016).
- DOI: 10.1016/j.ifacol.2016.10.321
- Verified via: Crossref metadata; abstract via Semantic Scholar API. ScienceDirect full text
  returned HTTP 403 to the fetcher.
- Summary (abstract only): Two-stage docking. Stage 1: Sum-of-Gaussians filter estimates the
  docking-station position from acoustic ranges while the AUV follows an observable
  trajectory, then homes into visual range. Stage 2: light-beacon navigation estimates the
  station pose; visual updates feed a SLAM filter. Evaluated by hardware-in-the-loop simulation,
  not at sea.
- Q: none from the abstract (detection range of lights, accuracy: UNVERIFIED).
- Can justify: the acoustic-far / optical-near hand-over architecture; range-only acoustic
  localization requiring vehicle manoeuvre for observability.

---------------------------------------------------------------------------------------------

## 2. Topic A - Sensor degradation underwater

### A1. Water Linked DVL A50 technical specifications (datasheet)
- Source: Blueye Robotics spec sheet for WL-DVL-A50 ("Supplier: Water Linked"),
  https://www.blueyerobotics.com/specsheet/WL-DVL-A50/pdf (generated 22 Oct 2026). The
  manufacturer's own docs pages (waterlinked.github.io/dvl/...) returned 404 at time of check.
- Verified via: PDF downloaded and text extracted.
- Q (vendor spec): 1 MHz, 4-beam convex Janus, 22.5 deg beam angle; ping rate "4 - 15 Hz
  (adaptive to altitude)"; min altitude 5 cm; max altitude 50 m (">35 m is dependent on seabed
  conditions, salinity"); max velocity 3.75 m/s; velocity resolution 0.1 mm/s; long-term accuracy
  "+-1.01 % (Standard version), +-0.1 % (Performance version)". Status LED shows "Lock".
- S: Bottom lock is altitude- and seabed-dependent (explicit caveat beyond 35 m).
- Can justify: DVL velocity error at the ~1 % level for this class; update rate of a few Hz to
  ~15 Hz; existence of a lock/no-lock state (i.e. dropout is a first-class sensor state).
  Note: distributor pages (DeepTrekker) list 2.6 m/s and 4-26 Hz; numbers differ between
  resellers, so cite the specific sheet used.

### A2. Nortek DVL1000 (300 m, Generation 3) datasheet
- Source: https://www.nortekgroup.com/products/dvl-1000-300m/pdf (dated 22 Oct 2026).
- Verified via: PDF downloaded, raw text extracted.
- Q (vendor spec): bottom velocity "Single ping std @ 1.5 m/s 0.8 cm/s at 1/2 max altitude";
  long-term accuracy "+-0.1% / +-0.1 cm/s (export-controlled), >1% (license-free)"; altitude
  0.1-75 m; max ping rate 8 Hz; 1 MHz, 4-beam Janus, 25 deg beam angle; "Quality estimates - per
  beam and ping".
- Can justify: separating single-ping white noise (~cm/s) from long-term scale/bias error
  (~0.1-1 %); per-beam validity (partial-beam dropouts are reported by real DVLs).

### A3. Tal, Klein, Katz (2017) - INS/DVL fusion with partial DVL measurements
- Citation: A. Tal, I. Klein, R. Katz, "Inertial Navigation System/Doppler Velocity Log (INS/DVL)
  Fusion with Partial DVL Measurements," *Sensors*, vol. 17, no. 2, art. 415, 2017.
- DOI: 10.3390/s17020415 (PMC5335996)
- Verified via: Crossref metadata + abstract (Crossref abstract field). PMC page blocked by captcha.
- Summary: On the Technion AUV, when bottom lock is not maintained on all beams the DVL cannot
  output a velocity vector, so a loosely coupled INS receives no aiding and drifts. Proposes an
  "extended loosely coupled" approach using raw partial-beam data; evaluated in a 6-DOF
  simulation only.
- Q: none in abstract.
- S: "failure to maintain bottom lock" -> "no velocity data can be integrated" -> drift.
- Can justify: modelling DVL loss as total dropout of the velocity vector (not just extra noise),
  and that partial-beam situations occur in practice.

### A4. Topini et al. (2023) - Deep learning for AUV navigation in DVL-denied environments
- Citation: E. Topini, F. Fanelli, A. Topini, M. Pebody, A. Ridolfi, A. B. Phillips, B. Allotta,
  "An experimental comparison of Deep Learning strategies for AUV navigation in DVL-denied
  environments," *Ocean Engineering*, vol. 274, art. 114034, 2023.
- DOI: 10.1016/j.oceaneng.2023.114034 (open copy: https://nora.nerc.ac.uk/id/eprint/534179/)
- Verified via: Crossref metadata; NORA landing page abstract.
- Summary (abstract): DVL "failures or fallacies" or DVL-denied areas can disrupt dead
  reckoning; the authors estimate body-frame velocity without the DVL using deep networks
  trained on National Oceanography Centre AUV field data, compared offline.
- Q: none extracted (no dropout statistics on the landing page).
- Can justify: DVL denial is a recognised operational failure mode on real AUVs (field data).

### A5. Kantor, Fairfield, Jonak, Wettergreen - Experiments in navigation and mapping with a hovering AUV
- Citation: G. Kantor, N. Fairfield, D. Jonak, D. Wettergreen, in *Field and Service Robotics*
  (Springer Tracts in Advanced Robotics), pp. 115-124 (FSR 2007; book 2008).
- DOI: 10.1007/978-3-540-75404-6_11
- Verified via: Crossref metadata only. Content claim "navigation filter designed to handle
  frequent DVL dropouts" comes from a search-engine snippet of the CMU RI page, which could not be
  opened (TLS error). -> content UNVERIFIED; cite only after reading.

### A6. TDK InvenSense ICM-20602 datasheet (IMU on the Blue Robotics Navigator board)
- Sources: Blue Robotics Navigator product page lists on-board "ICM-20602" IMU, "MMC5983" and
  "AK09915" magnetometers, BMP390 barometer
  (https://bluerobotics.com/store/comm-control-power/control/navigator/). Datasheet DS-000176
  Rev 1.0 (10/03/2016) via https://www.farnell.com/datasheets/2632660.pdf.
- Verified via: product page fetched; datasheet PDF downloaded and text extracted (table layout
  partly scrambled by extraction; values below read from feature list and Table 1).
- Q (vendor typ. values): gyro noise "4 mdps/sqrt(Hz)" (feature list; Table 1: 0.004 dps/sqrt(Hz)
  @10 Hz, total RMS 0.04 dps at 100 Hz bandwidth); accelerometer noise "100 ug/sqrt(Hz)";
  gyro initial zero-rate-output tolerance +-1 dps at 25 C, ZRO variation +-0.01 dps/C over
  -40..85 C (read from scrambled table; re-check in the PDF before quoting).
- Can justify: an order of magnitude for consumer MEMS gyro white noise and turn-on bias
  (~1 deg/s uncalibrated, temperature-dependent). Not a calibration of our units.

### A7. El-Sheimy, Hou, Niu (2008) - Allan variance analysis of inertial sensors
- Citation: N. El-Sheimy, H. Hou, X. Niu, "Analysis and Modeling of Inertial Sensors Using Allan
  Variance," *IEEE Trans. Instrumentation and Measurement*, vol. 57, no. 1, pp. 140-149, 2008.
- DOI: 10.1109/TIM.2007.908635
- Verified via: Crossref metadata (abstract not retrieved).
- S: Standard reference for using Allan variance to identify quantization noise, angle random
  walk, bias instability, rate random walk and rate ramp in inertial sensors.
- Can justify: the structure "white noise + random-walk bias" for the IMU model (as used by
  HoloOcean's IMU sensor, see H4). Do not attribute specific numbers without reading it.

### A8. Spielvogel, Shah, Whitcomb (2022) - Online magnetometer hard/soft-iron estimation
- Citation: A. R. Spielvogel, A. S. Shah, L. L. Whitcomb, "Online Three-Axis Magnetometer
  Hard-Iron and Soft-Iron Bias and Angular Velocity Sensor Bias Estimation using Angular
  Velocity Sensors for Improved Dynamic Heading Accuracy," *Field Robotics*, vol. 2,
  pp. 1001-1027, 2022.
- DOI: 10.55417/fr.2022033 (arXiv:2201.02449)
- Verified via: Crossref metadata; arXiv full text searched.
- Summary: MAVBE estimator for hard-/soft-iron magnetometer biases and gyro bias using only
  magnetometer and rate-gyro data; evaluated on a JHU ROV in a 7.5 m test tank (six 1.5 kW
  thrusters) and on an Iver3 AUV in the field.
- Q: MEMS AHRS heading "typically to within several degrees of accuracy" but needs careful
  hard/soft-iron calibration; in lab tests the MAVBE-calibrated heading had "a RMSE of roughly 1"
  deg vs ground truth.
- S: Classical calibrations need large rotations in all three axes, impractical for pitch/roll-
  stable vehicles (relevant to BlueROV2-type ROVs).
- Can justify: heading error of several degrees if uncalibrated, ~1 deg RMS achievable with
  good online calibration; heading bias as a slowly varying vehicle-specific error.

### A9. Rodriguez-Martinez, Troni (2025) - MAGYC: magnetometer and gyro bias via factor graphs
- Citation: S. Rodriguez-Martinez, G. Troni, "Full Magnetometer and Gyroscope Bias Estimation
  Using Angular Rates: Theory and Experimental Evaluation of a Factor Graph-Based Approach,"
  *IEEE J. Oceanic Engineering*, vol. 50, no. 3, pp. 1606-1615, 2025.
- DOI: 10.1109/JOE.2024.3523701 (arXiv:2412.09690)
- Verified via: Crossref metadata; arXiv abstract and full text (v1) searched.
- Q: In a seafloor-mapping dive, dead-reckoning error dropped "from 10% to 0.5% of the distance
  traveled" (abstract). In one experiment set the initial heading error "is reduced from 6.21 to
  0.55" (EXP1-14) and "from 34.19 to below 3.41" (EXP1-24) (arXiv v1 text; units are degrees per
  the figure context; check the published version).
- S: Soft-iron errors arise for "systems surrounded by ferromagnetic materials".
- Can justify: uncalibrated heading errors from a few to tens of degrees on real underwater
  platforms; heading error as the dominant DR drift driver.

### A10. Blue Robotics Bar30 pressure/depth sensor (MS5837-30BA) product specification
- Source: https://bluerobotics.com/store/sensors-cameras/sensors/bar30-sensor-r1/
- Verified via: product page fetched.
- Q (vendor): depth resolution "2 mm in fresh water (0.2 mbar)"; absolute accuracy +-200 mbar
  (~2 m fresh water) at 0-45 C; relative accuracy not specified on the page for Bar30 (Bar02:
  +-2 mbar ~2 cm). Requires drying "at least 2 hours daily" to maintain accuracy.
- Can justify: depth noise at mm-cm level with a possibly large constant offset (handled by
  zeroing at surface); depth is the most reliable navigation channel.

### A11. Blue Robotics Ping sonar altimeter/echosounder product specification
- Source: https://bluerobotics.com/store/sonars/echosounders/ping-sonar-r2-rp/
- Verified via: product page fetched.
- Q (vendor): 115 kHz; beamwidth 25 deg (-3 dB); usable range typ. 0.3-100 m; range resolution
  "0.5% of range"; accuracy and update rate not specified.
- S: Range "depends on operating conditions"; degraded by vegetation, fish groups, etc.; onboard
  algorithm tracks bottom "even in complicated situations with multiple echoes".
- Can justify: range-proportional noise (~0.5 % of range) and the existence of spurious echoes
  (fish, vegetation, multiple echoes) for a single-beam altimeter of the BlueROV2 class.

### A12. Kaeli, Packard, Fiester, Tebo, Purcell (2019) - Range-only acoustic localization with RANSAC
- Citation: J. Kaeli et al., "A Range-Only Acoustic Localization Technique for Marine Robotics
  Applications," *OCEANS 2019 MTS/IEEE Seattle*, pp. 1-5, 2019.
- DOI: 10.23919/OCEANS40490.2019.8962765
- Verified via: Crossref metadata; full-text PDF from WHOI staff page read.
- Summary: Adapted RANSAC jointly rejects outliers and estimates beacon position from acoustic
  ranges; run in real time on an AUV (REMUS lineage) to localize a stationary beacon.
- Q (their simulation settings, NOT field statistics): Gaussian noise plus "10% likelihood"
  uniform noise (multipath), a fraction of ambient-noise detections, and 10 % probability of
  no return.
- S: acoustic ranges are "highly non-Gaussian and prone to outliers" from ray bending and
  surface/seafloor multipath; bounce paths overestimate range; "sometimes there is no
  detection, creating gaps".
- Can justify: a mixture noise model (Gaussian core + uniform/one-sided outliers + misses) for
  acoustic ranges; also a published precedent for ~10 % outlier and ~10 % miss rates as
  stress-test settings.

### A13. Olson, Leonard, Teller (2006) - Robust range-only beacon localization
- Citation: E. Olson, J. J. Leonard, S. Teller, "Robust Range-Only Beacon Localization," *IEEE J.
  Oceanic Engineering*, vol. 31, no. 4, pp. 949-958, 2006.
- DOI: 10.1109/JOE.2006.880386
- Verified via: Crossref metadata; full-text PDF (april.eecs.umich.edu/media/pdfs/olson2006joe.pdf).
- Summary: Simultaneous AUV and LBL-beacon localization without surveyed beacons; clustering-
  based outlier rejection; real data from an Odyssey III AUV at GOATS'02 (Italy).
- Q: In one real-data example "over 25% of the measurements are outliers".
- Can justify: outlier fractions of tens of percent in real acoustic ranging; need for robust
  (gated) updates.

### A14. Huang et al. (2026) - Time-delay measurement/compensation for SINS/piUSBL
- Citation: J. Huang, Y. Wang, H. Li, Z. Liu, Z. Wang, Y. Chen, "Precise Time Delay Measurement
  and Compensation for Tightly Coupled Underwater SINS/piUSBL Navigation," *IEEE Trans.
  Instrumentation and Measurement*, vol. 75, pp. 1-14, 2026.
- DOI: 10.1109/TIM.2026.3676179 (arXiv:2510.23286)
- Verified via: Crossref metadata; arXiv abstract page (v2) and PDF text.
- Q: acoustic position fixes "experiences delays ranging from several hundred milliseconds to a
  few seconds" (introduction, citing [17]). Field results (v2 abstract): delay compensation reduced
  position RMSE by 44.02 % and max error by 40.79 % vs uncompensated (v1 reported different
  numbers; cite the published version).
- Can justify: modelling stale acoustic observations with 0.1-few s latency, and that ignoring
  latency measurably degrades accuracy.

### A15. Webster et al. (2012) - delayed-state handling (see B3 for full entry)
- Q: the filter keeps states "for the previous n seconds", n = 6, to accept ranges with travel
  times up to six seconds (~9 km). Supports delayed/out-of-sequence measurement handling.

### A16. Morency, Stilwell, Krauss (2023 preprint) - Low-cost forward-looking sonar for small AUVs
- Citation: C. Morency, D. J. Stilwell, S. T. Krauss, "Use of a low-cost forward-looking sonar
  for collision avoidance in small AUVs, analysis and experimental results," arXiv:2309.05785,
  2023. (A related peer-reviewed paper: C. Morency, D. J. Stilwell, IROS 2022, pp. 8423-8429,
  DOI 10.1109/IROS47612.2022.9982072 - metadata verified only.)
- Verified via: arXiv PDF first page (abstract). Journal venue of the preprint: UNVERIFIED.
- S: Field trials with a 9-beam prototype sonar on the Virginia Tech 690 AUV; varying
  sensitivity shows "the trade-off between the detection and false alarm rates".
- Can justify: false alarms and missed detections are coupled via a detection threshold in
  low-cost sonars, so they should be stress-tested together.

---------------------------------------------------------------------------------------------

## 3. Topic B - One-way underwater beaconing

### B1. Eustice, Singh, Whitcomb (2011) - Synchronous-clock OWTT navigation (journal)
- Citation: R. M. Eustice, H. Singh, L. L. Whitcomb, "Synchronous-clock, one-way-travel-time
  acoustic navigation for underwater vehicles," *J. Field Robotics*, vol. 28, no. 1,
  pp. 121-136, 2011 (online 2010).
- DOI: 10.1002/rob.20365
- Verified via: Crossref metadata only (abstract not retrieved; DeepBlue page 403).
- Can justify (title-level only): canonical reference for synchronous-clock OWTT; read before
  citing specific results.

### B2. Eustice, Whitcomb, Singh, Grund (2007) - Experimental results in synchronous-clock OWTT
- Citation: *Proc. IEEE ICRA 2007*, pp. 4257-4264. DOI: 10.1109/ROBOT.2007.364134
- Verified via: Crossref metadata; abstract text as indexed by U. Michigan DeepBlue (search
  result) - full text not opened.
- S: Modem-based system in which a source node broadcasts its navigation data as a packet and
  "passively receiving nodes" obtain a OWTT pseudo-range plus the sender's position; field
  test with a GPS-equipped ship aiding a Doppler-aided AUV.
- Can justify: one-way broadcast to any number of passive listeners, no interrogation needed.

### B3. Webster, Eustice, Singh, Whitcomb (2012) - Single-beacon OWTT navigation
- Citation: S. E. Webster, R. M. Eustice, H. Singh, L. L. Whitcomb, "Advances in single-beacon
  one-way-travel-time acoustic navigation for underwater vehicles," *Int. J. Robotics Research*,
  vol. 31, no. 8, pp. 935-950, 2012 (PDF header says 935-949).
- DOI: 10.1177/0278364912446166
- Verified via: Crossref metadata; full text (robots.engin.umich.edu/publications/swebster-2012a.pdf).
- Summary: Centralized EKF fusing DVL, depth, gyrocompass and OWTT ranges from a moving ship
  (WHOI Micro-Modem); packets "can be received simultaneously by multiple vehicles". Field data
  from AUV Puma in 4000 m water on the southern Mid-Atlantic Ridge, LBL ground truth.
- Q: clock "maximum drift rate of approximately 1 ms over 14 hrs", i.e. "a 1.5 m error in range".
  Ranges "are not typically made every second" because of channel and packet-duration limits.
  Sound-speed model error in range "of the order of one meter".
- Can justify: OWTT broadcast scales to many receivers; clock drift converts directly into range
  bias (~1.5 m per ms); realistic ping intervals are multi-second for modem-based systems.

### B4. Webster, Walls, Whitcomb, Eustice (2013) - Decentralized EIF for single-beacon cooperative navigation
- Citation: *IEEE Trans. Robotics*, vol. 29, no. 4, pp. 957-974, 2013. DOI: 10.1109/TRO.2013.2252857
- Verified via: Crossref metadata only. Content not read -> cite only for existence of the
  decentralized OWTT line of work.

### B5. Claus, Kepper, Suman, Kinsey (2018) - Closed-loop OWTT navigation with low-grade odometry
- Citation: B. Claus, J. H. Kepper IV, S. Suman, J. C. Kinsey, "Closed-loop one-way-travel-time
  navigation using low-grade odometry for autonomous underwater vehicles," *J. Field Robotics*,
  vol. 35, no. 4, pp. 421-434, 2018 (online 2017).
- DOI: 10.1002/rob.21746
- Verified via: Crossref metadata; abstract via Semantic Scholar; Caltech repository listing.
- Summary: Model-based velocity (no DVL) constrained by OWTT ranges from a topside beacon "as
  well as other AUVs operating in proximity"; EKF vs particle filter with water-current bias
  estimation; Monterey Bay field trials.
- Q: RMS error "on the order of tens of meters" vs GPS over "multiple kilometers".
- Note: the requested "Claus & Bachmayer passive localization of a beacon" could not be found;
  Crossref lists Claus & Bachmayer, "Terrain-aided navigation for an underwater glider," J. Field
  Robotics 32(7):935-951, 2015 (DOI 10.1002/rob.21563), which is not about beacons. -> UNVERIFIED.
- Can justify: OWTT aiding is useful even without DVL, with tens-of-metres accuracy over km.

### B6. Kepper, Claus, Kinsey (2019) - MEMS IMU + model-based DR + OWTT
- Citation: J. H. Kepper, B. C. Claus, J. C. Kinsey, "A Navigation Solution Using a MEMS IMU,
  Model-Based Dead-Reckoning, and One-Way-Travel-Time Acoustic Range Measurements for Autonomous
  Underwater Vehicles," *IEEE J. Oceanic Engineering*, vol. 44, no. 3, pp. 664-682, 2019.
- DOI: 10.1109/JOE.2018.2832878
- Verified via: Crossref metadata; abstract via Semantic Scholar.
- S: Consumer-grade MEMS IMUs "remain noisy and accrue error over time"; EKF with IMU bias
  estimation plus range filter; field trials on two AUV types.
- Can justify: modelling consumer MEMS bias as a state to be estimated; OWTT ranges bound DR drift.

### B7. Binder, Holbrook, Hayden, Krueger (2016) - Code collisions in acoustic telemetry arrays
- Citation: T. R. Binder, C. M. Holbrook, T. A. Hayden, C. C. Krueger, "Spatial and temporal
  variation in positioning probability of acoustic telemetry arrays: fine-scale variability and
  complex interactions," *Animal Biotelemetry*, vol. 4, art. 4, 2016.
- DOI: 10.1186/s40317-016-0097-4 (open access)
- Verified via: Crossref metadata; full-text PDF read.
- Summary: Field study (Lake Huron reef, fish tags + sync tags) of positioning performance of a
  Vemco positioning array. Periodic pingers with randomized intervals still collide when many are
  in range; collisions were "the main driver" of degraded positioning during spawning aggregations.
- Q:
  - Sync tags transmitted "every 500-700 s (nominal delay = 600 s)", uniform random interval.
  - Collision probability depends on number of transmitters, code duration and period; Vemco
    transmission duration "relatively long (up to ~5 s)".
  - Fig. 9 simulation: 1-50 transmitters, duration 5.12 s, intervals random within +-50 % of
    nominal delay.
  - Fish tags: V16-6H, 158 dB, "90 s nominal delay", detection ranges of several kilometres.
- S: Also cites close-proximity detection interference (echoes from strong nearby tags).
- Can justify: the core risk of uncoordinated periodic pinging (pure-ALOHA-like collisions,
  growing with fleet size and ping duration); randomized ping intervals (jitter) as standard
  practice; collision probability analysis as a function of N, duration and period.

### B8. Bosch, Gracias, Ridao, Istenic, Ribas (2016) - Close-range tracking with light beacons
- Citation: J. Bosch, N. Gracias, P. Ridao, K. Istenic, D. Ribas, "Close-Range Tracking of
  Underwater Vehicles Using Light Beacons," *Sensors*, vol. 16, no. 4, art. 429, 2016.
- DOI: 10.3390/s16040429 (PMC4850943, open access)
- Verified via: Crossref metadata/abstract; full text on PMC read.
- Summary: Omnidirectional camera (Ladybug 3) on Girona500 tracks active blinking LED markers on
  Sparus II and Seacat AUVs in formation; sea trials at Sant Feliu de Guixols and the Azores.
- Q:
  - Adequate tracking "at distances less than 10 m"; intermittent 10-15 m; failures at ~13.2 m
    and ~12.4 m; visibility estimated by divers "in the 20-25 m range".
  - Distance std "lower than 0.2 m" at 6-12 m; pose updates ~16 Hz (16 fps camera).
  - Over 11 min 21 s: 122 acoustic updates vs 10,894 optical; acoustic messages "every 5 s".
  - Markers: four, each with "five high-intensity LEDs"; 24 V, 22 W max; blink periods 0.25 s / 0.7 s.
- Can justify: optical relative localization ~10 m range in good visibility (well below
  visibility distance); acoustic update period of seconds vs optical tens of Hz; blink-pattern
  IDs to disambiguate multiple beacons.

### B9. Berlinger, Gauci, Nagpal (2021) - Blueswarm implicit coordination
- Citation: F. Berlinger, M. Gauci, R. Nagpal, "Implicit coordination for 3D underwater collective
  behaviors in a fish-inspired robot swarm," *Science Robotics*, vol. 6, no. 50, eabd8668, 2021.
- DOI: 10.1126/scirobotics.abd8668
- Verified via: Crossref metadata/abstract; Harvard SEAS/Wyss news coverage (search results).
- S: "only local implicit vision-based coordination" (abstract); robots detect neighbours' blue
  LEDs with cameras; aggregation, dispersion, milling demonstrated in a tank. Ranges/FOV not
  verified -> UNVERIFIED.
- Can justify: leaderless, communication-free coordination from presence-only optical beacons.

### B10. Zeng, Fu, Zhang, Dong, Cheng (2017) - Survey of underwater optical wireless communications
- Citation: *IEEE Communications Surveys & Tutorials*, vol. 19, no. 1, pp. 204-238, 2017.
- DOI: 10.1109/COMST.2016.2618841
- Verified via: Crossref/Semantic Scholar metadata only (abstract not returned). No numbers cited.

### B11. Kaushal, Kaddoum (2016) - Underwater optical wireless communication
- Citation: H. Kaushal, G. Kaddoum, "Underwater Optical Wireless Communication," *IEEE Access*,
  vol. 4, pp. 1518-1547, 2016.
- DOI: 10.1109/ACCESS.2016.2552538
- Verified via: Crossref metadata; Semantic Scholar abstract.
- S: UOWC offers higher rates and lower power "for short-range wireless links"; performance is
  governed by water absorption/scattering; proposes hybrid acousto-optic systems.
- Can justify: hybrid acoustic (long range, low rate) + optical (short range) architecture.

### B12. Chitre, Shahabudeen, Stojanovic (2008) - Acoustic communications and networking review
- Citation: *Marine Technology Society Journal*, vol. 42, no. 1, pp. 103-116, 2008.
- DOI: 10.4031/002533208786861263
- Verified via: Crossref metadata; abstract via Semantic Scholar.
- S: Overview of networking protocols and physical-layer techniques; complements S1.

---------------------------------------------------------------------------------------------

## 4. HoloOcean simulator

### H1. Potokar, Ashford, Kaess, Mangelson (2022) - HoloOcean
- Citation: E. Potokar, S. Ashford, M. Kaess, J. G. Mangelson, "HoloOcean: An Underwater Robotics
  Simulator," *Proc. IEEE ICRA 2022*, pp. 3040-3046.
- DOI: 10.1109/ICRA46639.2022.9812353
- Verified via: Crossref metadata; abstract via Semantic Scholar.
- S: UE4-based, multi-agent, "various sensor implementations of common underwater sensors",
  "simulated communications support", octree-based sonar model. Specific noise models are
  documented in the software docs (H4), not in the abstract.

### H2. Potokar et al. (2022) - HoloOcean: Realistic Sonar Simulation
- Citation: E. Potokar, K. Lay, K. Norman, D. Benham, T. B. Neilsen, M. Kaess, J. G. Mangelson,
  *Proc. IEEE/RSJ IROS 2022*, pp. 8450-8456.
- DOI: 10.1109/IROS47612.2022.9981119
- Verified via: Crossref metadata; abstract via Semantic Scholar.
- S: Cluster-based multipath ray tracing, "various probabilistic noise models", material
  dependence; adds side-scan, single-beam and multibeam profiling sonars.

### H3. Romrell et al. (2025) - A Preview of HoloOcean 2.0
- Citation: B. Romrell, A. Austin, B. Meyers, R. Anderson, C. Noh, J. G. Mangelson, "A Preview of
  HoloOcean 2.0," arXiv:2510.06160, 2025 (submitted to ICRA 2025 aq2uasim workshop).
- Verified via: arXiv abstract page.
- S: UE 5.3 migration, Fossen-model vehicle dynamics, ROS 2 bridge; more efficient ray-traced
  sonars listed as in development. No sensor-noise model description in the abstract.

### H4. HoloOcean documentation v2.3.0 - Sensors API
- Source: https://byu-holoocean.github.io/holoocean-docs/v2.3.0/holoocean/sensors.html
- Verified via: page fetched (summarised by the fetch tool; parameter names copied from it).
- Documented noise parameters:
  - DVLSensor: VelSigma/VelCov, RangeSigma/RangeCov, MaxRange, Elevation, ReturnRange.
  - IMUSensor: AccelSigma, AngVelSigma, AccelBiasSigma, AngVelBiasSigma (+Cov forms), ReturnBias.
  - DepthSensor: Sigma/Cov. MagnetometerSensor: Sigma/Cov, MagneticVector.
  - RangeFinderSensor: no noise parameters; negative value when no hit.
  - AcousticBeaconSensor: DistanceSigma/Cov, MaxDistance, CheckVisible; message arrival delayed
    by acoustic travel time.
  - OpticalModemSensor: MaxDistance (default 50 m), LaserAngle (default 60 deg), DistanceSigma,
    AngleSigma.
  - Imaging/Profiling sonar: AddSigma (Rayleigh), MultSigma (Gaussian), RangeSigma (exponential),
    MultiPath, ClusterSize, AzimuthStreaks. SinglebeamSonar: AddSigma, MultSigma, RangeSigma.
  - No built-in random dropout or outlier model for any sensor (only range/LOS gating).
- Can justify: our own dropout bursts, outliers, latency and bias processes must be added on top
  of HoloOcean outputs; the built-in models are Gaussian (plus sonar-image noise) and the
  acoustic beacon is the only sensor with physical latency.

---------------------------------------------------------------------------------------------

## 5. Synthesis for the noise model

Well supported by sources (phenomenon exists on real platforms):
- **Acoustic range outliers and misses (heavy tails).** Multipath/bounce paths, reverberation and
  false maxima are documented in field data (S3, A12, A13). Olson et al. report >25 % outliers in
  one real dataset; Kaeli et al. note detection gaps. A Gaussian + outlier mixture + miss model is
  well justified. Outlier rate is environment-dependent: tens of percent possible (A13).
- **DVL loss of bottom lock / partial beams -> velocity dropout.** Lock is altitude- and
  seabed-dependent (A1, A2), and loss means no velocity at all for a loosely coupled filter (A3);
  DVL-denied operation is a recognised field problem (A4). Durations/rates of dropout are NOT
  quantified in the verified sources -> burst length/rate are stress-test choices.
- **DVL noise level.** Vendor specs: single-ping std ~0.8 cm/s at 1.5 m/s (Nortek, A2); long-term
  accuracy ~0.1-1 % of velocity (A1, A2).
- **MEMS IMU noise and bias.** Consumer IMU (ICM-20602, on the Navigator board): gyro white noise
  ~0.004 deg/s/sqrt(Hz), turn-on zero-rate offset ~+-1 deg/s, temperature drift ~0.01 deg/s/C
  (A6, vendor typ.). White noise + random-walk bias is the standard structure (A7; HoloOcean H4).
- **Heading errors from magnetometer distortion.** Several degrees typical for MEMS AHRS without
  careful calibration (A8); 6-34 deg initial errors reduced to ~0.5-3.4 deg after calibration in
  one study (A9); ~3 deg std after calibration on SandShark AUVs (S3); ~1 deg RMSE achievable (A8).
  A slowly varying heading bias of a few degrees is well supported; the magnitude for our
  vehicles is unknown.
- **Depth.** mm-level resolution, large absolute offset (A10); depth is the least degraded channel.
- **Single-beam altimeter.** ~0.5 % of range resolution; spurious echoes from fish/vegetation/
  multiple echoes (A11). Threshold-dependent false-alarm/miss trade-off in low-cost sonars (A16).
- **Latency of acoustic observations.** Travel time ~0.67 ms per metre (S1); reported acoustic
  fix delays of several hundred ms to a few s (A14); delayed-state filters are standard (A15).
- **DR drift without DVL.** ~2-10 % of distance travelled for small AUVs (S3); 10 % reduced to
  0.5 % with magnetometer/gyro calibration (A9).

Orders of magnitude (for parameter ranges, not hardware claims):
| Quantity | Order of magnitude | Source |
|---|---|---|
| DVL single-ping velocity std | ~1 cm/s | A2 |
| DVL long-term scale error | 0.1-1 % | A1, A2 |
| DVL update rate | 4-15 Hz | A1, A2 |
| MEMS gyro white noise | ~0.004 deg/s/sqrt(Hz) | A6 |
| MEMS gyro turn-on bias | ~1 deg/s | A6 |
| Heading error (calibrated / uncalibrated) | ~1-3 deg / several to tens of deg | A8, A9, S3 |
| Depth resolution | ~mm | A10 |
| Altimeter range resolution | ~0.5 % of range | A11 |
| Acoustic range error (short range, CSAC OWTT) | 68 % < 0.7 m | S3 |
| Acoustic range outlier fraction | up to >25 % in one dataset | A13 |
| Acoustic observation latency | 0.1 s to a few s | S1, A14 |

Stress-testing choices only (not quantified by verified sources):
- DVL dropout burst duration and frequency (e.g. Markov on/off bursts).
- Exact outlier and miss probabilities for our sonar/altimeter (use a sweep; ~10 % each is a
  precedent used in simulation by A12, not a measured rate).
- Echosounder false-return distribution shape (uniform vs short-range clutter).
- Gyro/accelerometer bias random-walk intensities for our specific IMU (no Allan analysis of
  our units).
- Correlated (time-varying) heading bias near thrusters (qualitatively motivated by A8/A9;
  magnitude unknown).

---------------------------------------------------------------------------------------------

## 6. Synthesis for one-way beaconing

What presence-only / one-way pinging can provide:
- **Detection = existence + rough proximity.** Even without payload, a periodic ping tells a
  listener that a peer is within acoustic range; signal choice from a small set can carry a
  few bits (Rypkema's four chirp waveforms select fleet behaviours, S3).
- **Timing information.** With synchronized clocks (CSAC/PPS), arrival time gives one-way range
  (S3, B1-B3); without synchronization, a known/estimated period still allows passive
  localization from multiple receive positions (S2). Clock drift maps to range bias at
  ~1.5 m per ms (B3).
- **Bearing** only with an array (USBL, S3) or optical AoA (S4, B8); a single omni hydrophone
  gives no bearing.
- **Scalability.** One-way broadcast is received by any number of listeners (B2, B3, S3) and
  avoids round-trip handshakes that suffer from the ~1 s per 1.5 km propagation delay (S1).

What it cannot provide:
- No identity unless coded (waveform/frequency/blink ID: S3, S4, B8); no acknowledgement, so a
  sender never knows if it was heard; no relative position from a single omni detection;
  no guarantee against collisions (B7).

Typical periods and ranges reported:
- 1 Hz beacon, ranges generally < 100 m, 20 ms LFM chirps at 6-11 kHz (S3).
- Modem-based OWTT: ranges up to ~9 km accepted (6 s travel time), pings "not typically made every
  second" (B3); acoustic messages every 5 s in a formation trial (B8).
- Fish-tag pingers: 90 s nominal delay (high-power tags, km range) and 500-700 s sync tags (B7).

Collision / interference:
- Uncoordinated periodic pingers collide; probability grows with number of transmitters in range,
  transmission duration and ping rate; random interval jitter (+-50 % of nominal delay) is
  standard but does not eliminate collisions (B7). Close-proximity echoes cause additional
  interference (B7). Long propagation delay makes carrier sensing / ACK protocols ineffective
  (S1). Distinct waveforms or frequency bands (S3, S4) reduce mutual interference.

Optical beacons:
- Pisces: blue 455 nm LEDs, photodiode AoA/RSSR, MAE 0.031 m at 0.32 m and 0.16 m at 1 m, up to
  3 m in a turbid pool, FOV ~60-95 deg, 11 mA LED current (S4).
- Bosch et al.: camera + blinking LED markers, reliable < 10 m with 20-25 m visibility,
  ~16 Hz updates, distance std < 0.2 m at 6-12 m (B8).
- General bound: optical links "cannot travel further than about 100m" even in clear water (S1);
  450 nm best in open ocean, ~540 nm in turbid water (cited in S4).
- HoloOcean OpticalModemSensor defaults: 50 m max distance, 60 deg cone (H4) - a simulator
  default, much longer than the experimentally reported localization ranges above.

Open gaps (not found/verified in this pass): field statistics of DVL dropout duration; measured
false-return rates for BlueROV2-class single-beam altimeters; the Claus & Bachmayer beacon paper;
optical-beacon range vs turbidity curves in NTU.
