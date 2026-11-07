# Sensor degradation: the literature behind the CLEAN, LOW and MEDIUM profiles

This document collects the sources behind the sensor-noise profiles of `holo_fleet/sim/sensor_noise.py` (DVL, compass and AHRS, depth, IMU, sonar, latency). It merges `docs/SENSOR_ROBUSTNESS_LITERATURE.md` (2026-10-22) with the findings of the audits R1 (vehicle and navigation sensors) and R2 (sonar and HoloOcean) of 2026-10-29 (`HoloHybridAutomaton_archive/provenance_work/`). The beaconing and optical-localisation entries of the earlier note are not repeated: the B2 channel literature is in `docs/ACOUSTIC_CHANNEL_MODEL.md`, and the parameter-by-parameter verdicts are in `docs/PARAMETER_PROVENANCE.md`. Citation keys refer to `references.bib`.

Reading rules. Datasheet values describe a vendor's product under the vendor's test conditions; they are orders of magnitude for the simulation, not calibration data for our vehicles. Numbers are copied as printed. Evidence classes: (a) manufacturer specification, (b) peer-reviewed measurement, (c) derivation, (d) our simulator, (e) assumption. UNVERIFIED means the source could not be opened or the value could not be checked; such values are not used as support.

## 1. The profiles at a glance

CLEAN is the deployed suite. LOW and MEDIUM are the realistic operating domain of the validation campaign; HIGH is a stress level beyond it and is not documented here. Values are read from `sensor_noise.py`; the verdict classes are those of `docs/PARAMETER_PROVENANCE.md`.

| Channel | Field | Unit | CLEAN | LOW | MEDIUM | Verdict | Main sources |
|---|---|---|---|---|---|---|---|
| DVL | `dvl_vel_sigma` | m/s per beam | 0.01 | 0.01 | 0.015 | SUPPORTED | `nortek_nd_dvl1000`, `meachum2021wayfinder` |
| DVL | `dvl_range_sigma` | m per beam | 0.02 | 0.02 | 0.05 | STRESS ASSUMPTION | - |
| DVL | `dvl_dropout` | per sample | 0 | 0.01 | 0.03 | STRESS ASSUMPTION | `waterlinked_nd_docs_faq`, `tal2017partial` |
| DVL | `dvl_outage_every_s` | s, mean period | 0 | 0 | 60 | STRESS ASSUMPTION | `cohen2024seamless` |
| DVL | `dvl_outage_len_s` | s | 0 | 0 | 1 | STRESS ASSUMPTION | `cohen2024seamless` |
| DVL | (long-term scale error) | not modelled | - | - | - | DESIGN CHOICE (open issue O-4) | `waterlinked_nd_dvla50_datasheet` |
| IMU | `imu_accel_sigma` | m/s^2 | 0.01 | 0.01 | 0.02 | SUPPORTED | `tdk2019icm20602`, `invensense2015icm20602pb` |
| IMU | `imu_gyro_sigma` | rad/s | 0.005 | 0.005 | 0.01 | SUPPORTED | `tdk2019icm20602`, `invensense2015icm20602pb` |
| IMU | `imu_accel_bias_sigma` | m/s^2 per sample | 0 | 0.0002 | 0.0005 | STRESS ASSUMPTION | `elsheimy2008allan` |
| IMU | `imu_gyro_bias_sigma` | rad/s per sample | 0 | 0.0001 | 0.0003 | STRESS ASSUMPTION | `elsheimy2008allan` |
| Compass | `mag_sigma` | unit field | 0.005 | 0.01 | 0.02 | SUPPORTED | `memsic2019mmc5983ma` |
| Compass | `heading_bias_deg` | deg | 0 | 0.5 | 2 | LOW: PARTIALLY SUPPORTED; MEDIUM: SUPPORTED | `kinsey2006survey`, `spielvogel2022mavbe` |
| Depth | `depth_sigma` | m | 0.02 | 0.02 | 0.03 | SUPPORTED | `te2019ms583730ba`, `bluerobotics_nd_bar30` |
| All streams | `latency_s` | s | 0 | 0.02 | 0.05 | transport SUPPORTED; DVL age CONTRADICTED (O-3); IMU, compass, depth STRESS ASSUMPTION | `waterlinked_nd_docs_json`, `vonbenzon2022benchmark` |
| Sonar | `sonar_add_sigma` | intensity (Rayleigh scale) | 0.05 | 0.06 | 0.075 | CLEAN: DERIVED; LOW, MEDIUM: STRESS ASSUMPTION | `Rice1944noise`, `Lyons1999seafloor`, `Gallaudet2003fluctuations` |
| Sonar | `sonar_mult_sigma` | - | 0.1 | 0.15 | 0.25 | STRESS ASSUMPTION | `Potokar2022icra` |
| Sonar | `sonar_range_sigma` | m (exponential mean) | 0.05 | 0.05 | 0.1 | STRESS ASSUMPTION (MEDIUM against structure_tol_far_m: O-2) | `Potokar2022iros` |
| Sonar | `sonar_range_jitter_m` | m | 0 | 0.015 | 0.03 | LOW: STRESS ASSUMPTION; MEDIUM: CONTRADICTED (O-1) | `BlueRoboticsPing` |
| Sonar | `sonar_dropout` | per capture and sonar | 0 | 0.005 | 0.01 | STRESS ASSUMPTION | - |
| Sonar | `sonar_fade_prob` | per capture (gain 0.2) | 0 | 0.01 | 0.03 | STRESS ASSUMPTION | `kaeli2019range` |
| Sonar | `sonar_blackout_every_s` | s, mean period | 0 | 0 | 60 | STRESS ASSUMPTION | - |
| Sonar | `sonar_blackout_len_s` | s | 0 | 0 | 0.15 | STRESS ASSUMPTION | - |

## 2. DVL

**Hardware.** Water Linked DVL A50, datasheet revision v5 without an issue date (`waterlinked_nd_dvla50_datasheet`): 1 MHz, four-beam convex Janus array, beam angle 22.5 deg, ping rate 4 to 15 Hz (adaptive to altitude; the TCP JSON report rate is 2 to 15 Hz, `waterlinked_nd_docs_json`), altitude 5 cm to 50 m (above 35 m dependent on seabed conditions and salinity), maximum velocity 3.75 m/s, resolution 0.1 mm/s, long-term accuracy +-1.01 % (Standard) or +-0.1 % (Performance). The earlier note read the same figures from a Blueye copy of the sheet. Distributor pages quote other figures (2.6 m/s and 4 to 26 Hz at DeepTrekker): cite the sheet actually used.

**Velocity noise.** The A50 publishes no single-ping noise. Nortek DVL 1000 (`nortek_nd_dvl1000`): single-ping std 0.8 cm/s at 1.5 m/s at half of the maximum altitude, long-term accuracy +-0.1 % and +-0.1 cm/s (export-controlled) or above 1 % (license-free), altitude 0.1 to 75 m, up to 8 Hz, 25 deg beam angle. A Wayfinder 600 kHz on a BlueROV2 Heavy (vendor reprint, `meachum2021wayfinder`): single-ping std 10 mm/s at 1.5 m/s, scale-factor accuracy within +-1.15 %, zero-velocity offset within +-1 mm/s in a test tank. A 600 kHz Explorer DVL at 4 Hz: 0.01 m/s (`spielvogel2022mavbe`, Table VI). HoloOcean applies `VelSigma` to each beam; for a Janus array at 22.5 deg the body-frame horizontal std is sigma_b / (sqrt(2) sin 22.5 deg) = 1.85 sigma_b and the vertical 0.54 sigma_b (probe: 0.0184, 0.0188, 0.0054 m/s at 0.01). LOW therefore gives 1.85 cm/s and MEDIUM 2.8 cm/s per 10 Hz sample, 2.3 and 3.5 times the Nortek figure. The vendor values are at 1.5 m/s and, for Nortek, at half of 75 m, while our vehicles fly at 0.55 m/s or less a few metres above the seabed, so the comparison is conservative. White noise adds 0.055 m (LOW) and 0.083 m (MEDIUM) to a 90 s dead-reckoning run. Range noise (0.02 and 0.05 m per beam) has no source: neither the A50 sheet nor the documentation gives an altitude accuracy.

**Long-term scale error** (+-1 % class: A50 Standard, Nortek license-free, Wayfinder) is not modelled; it adds up to 1 % of the distance along track (open issue O-4).

**Dropouts and outages.** Only qualitative support exists. Bottom lock depends on altitude, bottom conditions, line of sight, acoustic noise, vehicle motion and range mode; thruster bubbles and blocked beams give invalid velocity; in reflective tanks the lock is re-checked every 10 s (`waterlinked_nd_docs_faq`). Each TCP report carries a `velocity_valid` flag (`waterlinked_nd_docs_json`). Partial-beam situations near the seafloor, at extreme tilt or with a beam malfunction leave the DVL without a velocity vector, and a loosely coupled INS then receives no aiding (`tal2017partial`, 6-DOF simulation only). DVL-denied navigation is a recognised failure mode of field AUVs (`topini2023dvldenied`, abstract level). A preprint (`cohen2024seamless`, not peer reviewed) names steep roll and pitch, trenches and rocks, and marine life as causes and uses partial-beam windows of 5 to 25 s and then 30 s in its experiments. No frequency or duration statistic for small vehicles was found, so the 1 to 3 % per-sample dropout and the 1 s outage every 60 s at MEDIUM are stress settings; outage windows of 5 to 30 s would be informative.

**Latency.** The TCP JSON message latency is about 4 ms on average, 2 ms std, 13 ms observed maximum (transport only). The interval from time of validity to transmission was 0.189 s in the documentation's example report, and von Benzon Table 1 lists a DVL delay of 0.1 s (`vonbenzon2022benchmark`). The simulator applies one latency to every stream (0.05 s at MEDIUM): for the DVL this is contradicted by these two sources (open issue O-3).

## 3. Compass and AHRS

- **Hardware.** The Navigator carries an ICM-20602 IMU, an MMC5983 magnetometer, an AK09915 electronic compass and a BMP280 or BMP390 barometer (the BMP390 replaced the end-of-life BMP280 on 9 Aug 2025) (`bluerobotics_nd_navigator`). The ICM-42688 is not listed. Which magnetometer ArduSub uses as primary is UNVERIFIED.
- **Heading accuracy after calibration.** Properly calibrated magnetic heading sensors reach about 1 to 3 deg (`kinsey2006survey`, Section 2.2.1); vehicle disturbance, tilt compensation under acceleration, local anomalies and mounting misalignment are systematic errors. MEMS AHRS heading is typically within several degrees after careful hard- and soft-iron calibration; in a tank with a ROV, heading RMSE was 23.34 and 28.00 deg uncalibrated, 0.75 and 1.12 deg with MAVBE and 2.88 and 2.60 deg with TWOSTEP (`spielvogel2022mavbe`, Table V). MAGYC reduced an initial heading error from 6.21 to 0.55 deg (EXP1-14) and from 34.19 to below 3.41 deg (EXP1-24), and dead-reckoning error from 137 m after 1395 m (10 %) to below 2.85 m (0.20 %) in EXP2-14 (`rodriguezmartinez2025magyc`, arXiv text; published version not opened). On SandShark AUVs the 1-sigma heading error was about 3 deg after magnetometer and IMU calibration (`rypkema2022synchronous`; the ar5iv rendering prints 33 deg, a typesetting artefact: confirm in the PDF). The MMC5983MA sheet gives 1.0 deg typical heading accuracy with the vendor software (the front page claims 0.5 deg) (`memsic2019mmc5983ma`).
- **Mapping to the profiles.** A constant bias b gives a cross-track dead-reckoning error D sin b: 0.87 % of the distance for LOW (0.5 deg, a best case only) and 3.49 % for MEDIUM (2.0 deg). Magnetometer white noise: heading std is about 57.3 sigma deg, i.e. 0.29, 0.57 and 1.15 deg per sample for CLEAN, LOW and MEDIUM (probe: 0.29, 0.56, 1.16); the chip noise is 0.4 to 1.2 mG RMS, so real heading error is dominated by calibration residuals. Heading is `atan2(m_y, m_x)` of the raw sample.
- **Dead-reckoning drift.** Published dead-reckoning drift as a share of the distance travelled: best INS 0.1 %, more typical modestly priced units 2 to 5 % (`paull2014review`); bottom-lock Doppler with an inertial heading unit 0.4 % over five-hour missions (`kinsey2006survey`); 10 % reduced to 0.2 to 0.5 % with MAGYC; 2 to 10 % without a DVL on SandShark. LOW and MEDIUM lie inside this range.
- **Not modelled.** Heading dependence (soft iron), time variation near thrusters, and tilt coupling (the simulated magnetic vector is horizontal).

## 4. Depth

MS5837-30BA (`te2019ms583730ba`): resolution 0.20 mbar RMS at OSR 8192 (0.28, 0.38, 0.54, 0.84, 1.57 mbar at OSR 4096 down to 256); absolute accuracy +-50 mbar (0 to 6 bar), +-100 mbar (0 to 20 bar), +-200 mbar (0 to 30 bar) with autozero at one pressure point; long-term stability +-30 mbar per year; the sensor must typically be dried once a day. Bar30 (`bluerobotics_nd_bar30`): resolution 2 mm (0.2 mbar), absolute accuracy +-200 mbar (204 cm of fresh water), dry at least 2 h daily. Our 0.02 m (LOW) is about 1.96 mbar, ten times the best RMS resolution, and 0.03 m (MEDIUM) about fifteen times; the absolute offset (about +-0.5 m at 0 to 6 bar) is removed by zeroing at the surface and is not modelled. Depth is the least degraded channel.

## 5. IMU

- **Noise.** ICM-20602: accelerometer noise 100 ug/sqrt(Hz), gyro rate noise 0.004 dps/sqrt(Hz), sensitivity error +-1 % (`tdk2019icm20602`, `invensense2015icm20602pb`). The accelerometer figure is 9.81e-4 sqrt(B) m/s^2: 3.8e-3 at 15 Hz and 9.8e-3 at 100 Hz, so LOW is 1.0 to 2.6 and MEDIUM 2.0 to 5.3 times the datasheet. The gyro figure is 6.98e-5 rad/s/sqrt(Hz): 2.7e-4 rad/s at 15 Hz, so LOW is 7 to 18 and MEDIUM 14 to 37 times the datasheet; the excess may stand for thruster vibration. A MicroStrain 3DM-GX5-25 lists 0.0075 m/s^2 and 0.00025 rad/s (`spielvogel2022mavbe`, Table VI). The gyro is used for yaw-rate damping only and the accelerometer for roll and pitch only: no IMU signal enters the horizontal position estimate.
- **Bias random walk.** White noise plus a random-walk bias is the standard structure (Allan-variance analysis, `elsheimy2008allan`; the HoloOcean IMU model). The step sizes (2e-4 and 5e-4 m/s^2; 1e-4 and 3e-4 rad/s per 30 Hz sample) have no source: the in-run stability of the ICM-20602 could not be retrieved (the ZRO tolerance of +-1 dps and its +-0.01 dps/C variation noted earlier from a scrambled table are UNVERIFIED). After 90 s the accumulated gyro bias is 0.30 deg/s (LOW) and 0.89 deg/s (MEDIUM), the equivalent tilt error 0.06 and 0.15 deg. For comparison, MEMS gyroscopes have accuracies of 1 to 5 deg/s (`kinsey2006survey`, Section 2.2.3) and a calibrated A50 AHRS drifts 0.1 to 0.3 deg per minute in yaw (`waterlinked_nd_docs_deadreckoning`).

## 6. Sonar

**6.1 HoloOcean semantics** (R2 section B, engine source; documentation `HoloOceanDocs230`, papers `Potokar2022icra`, `Potokar2022iros`). The `SinglebeamSonar` is a hard-edged cone with uniform sensitivity inside it; range is the Euclidean distance to the octree leaf (no time of flight, no sound speed). Bin value = mean of leaf values (R^2 cos, R from the acoustic impedance) times (1 + w_mult), plus Rayleigh noise added to every bin. `AddSigma` is the Rayleigh scale (documented as "std"; the std of the noise is 0.655 AddSigma). `MultSigma` is a normal factor per occupied bin. `RangeSigma` is the mean of a one-sided exponential, drawn per leaf after shadowing, not a standard deviation. `ShadowEpsilon` defaults to one octree leaf for this sensor (documented as four). The generator is unseeded, so engine noise is not reproducible. A preview of HoloOcean 2.0 (`Romrell2025holoocean2`) lists more efficient ray-traced sonars as work in development and has no noise-model content. HoloOcean has no built-in dropout, outage or outlier model for any sensor, so the wrapper in `holo_env.py` adds capture dropout, blackouts, whole-profile range jitter, fades, DVL outages and latency.

**6.2 Noise statistics.** The per-bin false-alarm probability for the Rayleigh noise is exp(-T^2 / (2 sigma^2)) with T = 0.30 (Rice 1944, `Rice1944noise`; confirmed on the probe within Poisson error, exceedances 0, 1, 8, 44, 296 observed against 0.0, 1.2, 10.3, 45.2, 283.8 expected for sigma 0.05 to 0.11). Six sonars at 10 Hz give 60 captures per second per drone.

| sigma | Level | T / sigma | Per bin | Per capture and sonar (234 bins) | Per drone and second | Confirmed 2-of-3 ghosts per drone and second |
|---|---|---|---|---|---|---|
| 0.05 | CLEAN | 6.0 | 1.52e-8 | 3.6e-6 | 2.1e-4 | 1.4e-10 |
| 0.06 | LOW | 5.0 | 3.73e-6 | 8.7e-4 | 0.052 | 8.2e-6 |
| 0.075 | MEDIUM | 4.0 | 3.36e-4 | 0.076 | 4.5 | 0.064 (one every 16 s) |
| 0.09 | HIGH (stress) | 3.3 | 3.87e-3 | 0.60 | 36 | 5.6 |

Shallow-water field data are heavier tailed than Rayleigh: 80 kHz seafloor reverberation was not always Rayleigh (`Lyons1999seafloor`), and threshold detectors that assume Rayleigh fluctuations suffer significantly higher false-alarm rates from bubbles, zooplankton, fish and boundary reverberation (68 kHz, `Gallaudet2003fluctuations`). The tabulated rates are therefore optimistic against the field. No quantitative false-alarm rate for a small-vehicle sonar was found; a nine-beam forward-looking sonar on a small AUV showed a detection and false-alarm trade-off with the sensitivity setting and considerable boat-noise false alarms, reported qualitatively (`Morency2023lowcost`). The LOW and MEDIUM values of `AddSigma`, `MultSigma` and `RangeSigma` are stress settings (MEDIUM `RangeSigma` 0.10 m against `structure_tol_far_m` 0.70 m: open issue O-2).

**6.3 Cone, range and resolution.** No conical single-beam proximity sonar with a 120 deg beam was found; the 120 deg cone is a design assumption that follows from the coverage requirement of six body-axis cones (half-angle of at least 54.74 deg). Nearest hardware:

| Product | Beam | Range | Resolution | Rate | Key |
|---|---|---|---|---|---|
| Blue Robotics Ping (115 kHz) | 25 deg at the -3 dB level | typ. minimum 0.3 m, usable 100 m | 0.5 % of range, tied by the page to the scan (waterfall) range, not the measured range | not specified | `BlueRoboticsPing`, `BlueRoboticsPingGuide` |
| Cerulean S500 (500 kHz) | about 5 deg, 2-way conical | 0.3 to 100 m and more | 3 mm to 17 m, 9 mm to 50 m | up to 10 pings/s | `CeruleanS500` |
| Tritech MicronEchosounder (500 kHz) | 6 deg conical | 0.3 to 50 m | 1 mm | not specified | `TritechMicronEcho` |
| Imagenex 852 (675 kHz; 6000 m model with 330 kHz option) | 10 deg conical; 20 deg with the option | minimum 0.5 m, maximum 50 m | 20 mm | not specified | `Imagenex852` |
| Valeport VA500 altimeter (500 kHz) | +-3 deg | 0.1 to 100 m | 1 mm | 1, 2, 4 Hz, continuous | `ValeportVA500` |
| DFRobot SEN0599 (ultrasonic) | 8 to 16 deg | 5 to 600 cm | +-(0.5 cm + 0.5 % of distance) below 2 m | 14 ms response | `DFRobotSEN0599` |
| Humminbird Helix 7 G2 transducer (retailer listing, weak) | 60 deg at 83 kHz, 20 deg at 200 kHz, at -10 dB | - | - | - | `HumminbirdHelix7G2retail` |
| Nine-beam forward-looking prototype for small AUVs | centre beam 10 deg, outer beams 20 deg at +-15 and +-35 deg, both planes | 219 bins to 50 m | 0.23 m bins | ping about every 0.1 s | `Morency2023lowcost` |
| Blue Robotics Ping360 (mechanical scan) | 2 x 25 deg | 0.75 to 50 m | 0.08 % of range | 4.17 s per 360 deg at 1 m | `BlueRoboticsPing360` |
| Tritech Gemini 720is (512 beams) | 120 x 20 deg | 0.1 to 120 m | 4 and 8 mm | 5 to 97 Hz | `TritechGemini720is` |
| Blueprint Oculus M750d (512 beams) | 130 x 20 deg | 0.1 to 120 m (40 m at 1.2 MHz) | 4 and 2.5 mm | up to 40 Hz | `BlueprintOculus` |
| Water Linked Sonar 3D-15 | 90 x 40 deg (1.2 MHz), 40 x 40 deg (2.4 MHz) | 15 m, 4 m | 1.5 mm | 5 Hz, 20 Hz | `WaterLinked3D15` |

A research prototype of a wide-angle ultrasonic obstacle-avoidance sensor with a curved radiator exists (`Nolan2005wideangle`, abstract only, no beamwidth value). The ideal directivity index of a uniform cone is 6.0 dB at 120 deg and 19.3 dB at 25 deg, about 13 dB less per pass, which HoloOcean ignores. The minimum range of 0.3 m is supported (Ping, Micron and S500), the 10 Hz rate is the order of the S500 maximum, and the 5 cm bin is close to the Ping's 0.5 % of a 12 m scan (6 cm); resolution is not accuracy, and no opened datasheet states a range accuracy.

**6.4 Wrapper stress and the corrected anchors.** The earlier note justified the whole-profile range jitter by the Ping's "0.5 % of range" resolution. The Ping page labels these rows as relative to the waterfall (scan) range, not the measured range, and a resolution is a quantisation step, not a standard deviation (a uniform step D has std D / sqrt(12)); the anchor does not support the values, and a range-independent shift does not fit a sound-speed error either. At MEDIUM the jitter exceeds the declared `eps_range` in about 1.2 % of captures (open issue O-1). The 10 % outlier and 10 % no-return settings of the earlier anchor A12 (`kaeli2019range`) concern transponder ranging, not echo detection, and were not re-verified, so they do not support the sonar dropout, fade or false-alarm values. These values (dropout 0.5 and 1 %, fade 1 and 3 % with gain 0.2, blackout 0.15 s every 60 s) are stress settings.

**6.5 Sound speed.** HoloOcean ranges contain no sound-speed term. Real sensors convert travel time with an assumed speed (the Ping default is 1 500 000 mm/s). Sound speed changes by about +4.0 m/s per deg C, +1.4 m/s per PSU and +17 m/s per km of depth (`DOSITSsoundspeed`; nine-term equation `mackenzie1981nine`). A mismatch of 20 to 50 m/s gives a range scale error of 1.3 to 3.4 %: +0.03 to +0.08 m at 2.4 m and +0.16 to +0.40 m at 12 m, which `eps_range` does not contain; a hardware deployment needs c to about +-0.5 %.

## 7. Settings with no measured statistics

The following are stress choices, not measured sensor accuracy: DVL dropout and outages; sonar capture dropout, blackouts, fades and the Rayleigh sigma of LOW and MEDIUM; IMU bias random-walk steps; and the IMU, compass and depth latency. The earlier note listed the same gaps: field statistics of DVL dropout duration, measured false-return rates for single-beam altimeters of this class, exact outlier and miss probabilities (about 10 % each is a precedent used in simulation, not a measured rate), and the correlated heading bias near thrusters (qualitatively motivated, magnitude unknown).

## 8. Status of the entries of the earlier note

| Earlier entry | Result of the audits |
|---|---|
| A1 DVL A50 (via a Blueye sheet) | figures confirmed against the Water Linked datasheet v5 |
| A2 Nortek DVL1000 | confirmed |
| A3 Tal et al. 2017 | confirmed (partial-beam loss leaves no velocity; text from PubMed Central, author first names not verified) |
| A4 Topini et al. 2023 | metadata and abstract only |
| A5 Kantor et al. | content UNVERIFIED (search snippet only); not used |
| A6 ICM-20602 | noise figures confirmed in two vendor documents; the ZRO tolerance of DS-000176 could not be retrieved: UNVERIFIED |
| A7 El-Sheimy et al. | metadata only; structure of the model, no numbers |
| A8 Spielvogel et al. | confirmed; tables V and VI added |
| A9 MAGYC | confirmed in degrees; published version not opened |
| A10 Bar30 | confirmed; MS5837-30BA datasheet added |
| A11 Ping | values confirmed; the inference "range-proportional noise" is a misread |
| A12 Kaeli et al. | metadata confirmed; numbers UNVERIFIED and not applicable to sonar |
| A16 Morency et al. | confirmed (nine beams, 219 bins, about 0.1 s ping, qualitative boat-noise false alarms) |
| H1 to H4 HoloOcean | confirmed; H4 was incomplete (section 6.1) |
| S3 Rypkema et al. 2022 | confirmed except the range: the 0.7 m and 7.0 deg statistics come from a static test at about 30 and 57 m, not "30 to 300 m" |

Open gaps carried over: field statistics of DVL bottom-lock loss for small vehicles; coastal or harbour current statistics (a lead exists, `doi 10.3354/aei00310`, abstract only, no magnitudes, not cited for any value); the BlueROV2 configuration in the hydrodynamic load measurements of `gabl2021loads`; the Navigator and ArduSub output-filter bandwidths and the IMU, compass and depth stream latency; the issue date of the A50 documents.
