# Decentralized underwater multi-robot coordination: a structured literature review

**Type of review.** Structured, reproducible review inspired by PRISMA 2020 reporting. It is **not** a systematic review: one reviewer screened all records, there was no registered protocol, no dual screening and no risk-of-bias assessment.

**Date of search:** 2026-11-03. **Reviewer:** automated agent working for the project; all metadata checked against Crossref.

**System mapped against the literature.** A leaderless fleet of 3-6 small AUVs/ROVs (BlueROV2 class, simulated in HoloOcean). Each vehicle has six wide-cone single-beam sonars (one per body axis), DVL/IMU/compass/depth dead reckoning and map-aided relocalisation against a known gate structure. Each vehicle broadcasts a periodic one-way acoustic identity ping (TDMA, no payload, no acknowledgement). Receivers estimate range from one-way travel time (synchronised clocks) and bearing with a small receive array (inverted-USBL style). Vehicles exchange no position, velocity, command, priority or automaton state, and there is no leader. Every vehicle runs the same local hybrid automaton with the priority order failsafe > collision avoidance > separation warning > gate passage/yield/approach > formation recovery > formation keeping. Narrow gates are critical regions (CRs) crossed one vehicle at a time: precedence comes from the local queue geometry and from an onboard occupancy belief (sonar echoes in the gate corridor, positive exit evidence, a timeout fallback, acoustic evidence). Formation slots lie on a shared pre-planned path with a shared clock; a vehicle that perceives a neighbour missing slows its private formation clock, waits a bounded time, declares the slot vacant (degraded formation) and may later re-include it.

**Topics.** T1 decentralised underwater multi-robot coordination; T2 hybrid automata and formal verification; T3 collision avoidance and control barrier functions; T4 mutual exclusion and shared narrow passages/intersections; T5 deadlock avoidance and liveness; T6 sonar-only perception and observability limits; T7 underwater acoustic localisation, OWTT ranging and USBL; T8 passive reception of identity beacons for relative localisation; T9 formation keeping and recovery; T10 fault-tolerant and degraded formations; T11 realistic underwater sensing, currents and communication constraints. A cross-cutting block **X** covers estimation under intermittent observations (existence filters, occupancy filters, multiple-hypothesis tracking, sequential tests), which serves improvement areas I1, I3, I5.

**Improvement areas.** I1 occupancy verification of a narrow passage with intermittent, ambiguous observations; I2 safe entry and guaranteed passage completion; I3 acoustic identification and relative localisation of neighbours; I4 formation rejoining and synchronisation of formation progress or clocks without communication; I5 robustness against intermittent observations; I6 deadlock prevention without centralised communication.

---

## 1. Method

### 1.1 Sources

| Source | Interface and settings | Role |
|---|---|---|
| OpenAlex | `https://api.openalex.org/works?filter=title_and_abstract.search:<string>&per-page=20` (all terms must occur in title or abstract) | primary bibliographic database |
| Crossref | `https://api.crossref.org/works?query=<string>&rows=10` (relevance ranking) | second database; also used for verification |
| Semantic Scholar | `https://api.semanticscholar.org/graph/v1/paper/search?query=<string>&limit=20` | third database; the shared unauthenticated pool returned HTTP 429 for 44 of 46 queries (see Limitations) |
| WebSearch (general web) | 10 queries, standard mode | discovery of recent work and known-item lookups |
| Crossref bibliographic lookup | `query.bibliographic=<author title>&rows=3` | locating the DOI of known works (7 lookups) |
| Known-item list | 99 DOIs of seminal works compiled from the reviewer's domain knowledge, each checked on Crossref | "other methods" stream |

The scripts, raw API responses and Crossref records are kept in the scratchpad folder `litreview/` next to this file: `scripts/queries.json`, `scripts/queries_supp.json`, `scripts/search.py`, `scripts/verify.py`, `scripts/cr_bib.py`, `scripts/mkbib.py`, `scripts/flow.py`, `scripts/fill.py`; raw search responses in `raw/`; Crossref work records in `verify/`. Re-running `search.py` re-queries the APIs; results will drift as the databases change.

### 1.2 Search strings per topic and records looked at

Main strings (34 queries) were defined before searching. Twelve supplementary short strings were added after the first pass because the OpenAlex filter requires every term to match, so several long strings returned no OpenAlex record (for example T4q2, T4q3, T8q1, T8q2). "Records looked at" counts the records whose title, year, venue and DOI were screened, after removing duplicates within each query. The per-query breakdown by source is in Appendix A.

| Topic | Search strings (main; supplementary in italics) | Records looked at |
|---|---|---|
| T1 | "decentralized multi-AUV coordination"; "underwater swarm robotics implicit coordination without communication"; "multiple autonomous underwater vehicles cooperative control survey" | 66 |
| T2 | "hybrid automata formal verification robots"; "multi-agent hybrid systems conflict resolution safety verification" | 40 |
| T3 | "control barrier functions multi-robot collision avoidance"; "collision avoidance multiple autonomous underwater vehicles"; "decentralized collision avoidance without communication robots" | 87 |
| T4 | "mutual exclusion intersection autonomous vehicles distributed"; "multi-robot coordination narrow passage critical section"; "decentralized intersection crossing robots without communication"; *"narrow passage multi-robot"*; *"intersection management autonomous vehicles survey"*; *"AGV zone control deadlock"* | 117 |
| T5 | "deadlock resolution multi-robot systems decentralized"; "provably safe deadlock-free multi-robot coordination" | 55 |
| T6 | "wide-angle sonar occupancy grid uncertainty"; "single beam sonar obstacle detection underwater vehicle"; "observability range-only bearing-only underwater navigation"; *"range-only observability underwater"* | 67 |
| T7 | "one-way travel time acoustic navigation synchronous clock"; "inverted ultra-short baseline localization AUV"; "cooperative localization autonomous underwater vehicles acoustic ranging" | 50 |
| T8 | "acoustic beacon passive localization multiple AUVs swarm"; "relative localization underwater robots acoustic broadcast neighbors"; *"passive inverted USBL"*; *"low-cost acoustic modem localization micro AUV"*; *"single beacon swarm AUV acoustic"* | 55 |
| T9 | "coordinated path following marine vehicles communication losses"; "formation control autonomous underwater vehicles ocean currents"; "synchronization path parameter formation without communication"; *"range-based formation control marine vehicles"*; *"cooperative path following event-triggered communication marine"* | 97 |
| T10 | "fault-tolerant formation control multi-agent agent loss reconfiguration"; "robot swarm fault detection heartbeat synchronization"; "failure detector timeout distributed systems accrual"; *"formation reconfiguration agent failure"*; *"graceful degradation multi-robot formation"* | 72 |
| T11 | "underwater acoustic communication channel challenges networking"; "underwater robotics simulator sonar acoustic"; "AUV navigation localization review"; *"BlueROV2 multi-vehicle"* | 108 |
| X | "target existence probability tracking intermittent detection Bernoulli filter"; "Bayesian occupancy filter dynamic environment tracking"; "multiple hypothesis tracking track confirmation"; "pulse-coupled oscillators synchronization" | 91 |
| **Total** | 46 queries | **905** (867 unique across queries) |

**WebSearch queries (other methods).** (1) single-hydrophone low-cost underwater vehicle swarming one-way travel time beacon; (2) deadlock analysis and resolution multi-robot control barrier functions Grover Sycara; (3) provably safe multi-robot coordination unreliable communication critical sections Mannucci Pallottino Pecora; (4) decentralized multi-robot narrow corridor doorway passage deadlock without communication; (5) Rypkema Schmidt Fischell synchronous-clock range-angle relative acoustic navigation multi-AUV Field Robotics; (6) heartbeat-based fault detection robot swarm synchronized flashing fireflies Christensen Dorigo; (7) Fischell Kroo O'Neill "Single-Hydrophone Low-Cost Underwater Vehicle Swarming" IEEE Robotics and Automation Letters; (8) fault-tolerant formation control multiple AUVs vehicle failure reconfiguration ocean engineering; (9) control barrier functions autonomous underwater vehicles multi-AUV safety-critical collision avoidance; (10) communication-free coordinated path following marine vehicles range measurements neighbour progress estimation. Queries 2, 3, 5, 6 and 7 name authors and are known-item searches. About 100 web results were looked at.

### 1.3 Eligibility criteria

*Inclusion:* peer-reviewed journal or conference papers, or established books and handbook chapters; English; directly relevant to a topic and to the system above. Widely cited arXiv preprints were allowed if marked as preprints (none was included in the end). Seminal works of any year and recent work (2015-2026) were both sought.

*Exclusion:* not verifiable (no DOI and no stable publisher record), no abstract or summary available, off-topic, superseded by a later version of the same work. Theses, repository deposits, SSRN/Research Square/Preprints.org records and Qeios reviews were excluded at screening as not peer-reviewed. A scope cap of about 60 included items was applied, so at eligibility a work was excluded when an included work already covered the same contribution (reason R1 below).

### 1.4 Verification

Every included item was checked in two ways: (a) `https://api.crossref.org/works/<DOI>` returned a record whose title, authors, venue and year match the reference given here; (b) the DOI handle resolves (`https://doi.org/api/handles/<DOI>` returned HTTP 200 for all 62 included DOIs). Crossref lacks author metadata for one included item (Stilwell and Bishop 2000) and for two excluded works (Bar-Shalom et al. 2009; Anderson et al. 2008); their authors were taken from the OpenAlex record of the same DOI. Four DOIs that the reviewer first typed from memory failed verification: two were not registered (`10.1109/9.665008`, `10.1109/TRO.2021.3075001`) and two resolved to unrelated papers (`10.1109/TAES.1981.309142`, `10.1109/IROS.2016.7759239`). The correct DOIs (Tomlin et al. 1998; Mannucci et al. 2021; Nardone and Aidala 1981; Cap et al. 2016) were found by Crossref bibliographic search and verified. Verification covers bibliographic metadata, not the content of each paper; content statements rely on abstracts (Crossref or OpenAlex) and, for seminal works, on the reviewer's prior knowledge.

### 1.5 Counts and PRISMA-style flow

```
IDENTIFICATION
  Databases (46 queries): OpenAlex 489 + Crossref 460 + Semantic Scholar 40 = 989 records
      -> 84 within-query duplicates removed            -> 905 records
      -> 38 cross-query duplicates removed             -> 867 unique records
  Other methods: 99 known-item DOIs (4 wrong identifiers corrected, see 1.4),
                 10 WebSearch queries (~100 web results), 7 Crossref bibliographic lookups

SCREENING (title, venue, year, DOI)
  867 unique database records screened
      excluded 832: no DOI / not verifiable 45; preprint or non-peer-reviewed 38;
                    thesis or repository deposit 43; off-topic, lower relevance or
                    earlier version of a work also retrieved 706
      retained 35
  Other methods: 84 further works passed to eligibility

ELIGIBILITY (Crossref record + abstract)
  119 works assessed
      excluded 57 (R1 redundant 28; R2 superseded 1; R3 needs messages/coordinator 12;
                   R4 less transferable or tangential 16)  [Appendix B]

INCLUDED
  62 works (26 identified by the database queries, 36 by other methods)
  62/62 DOIs verified on Crossref and resolving at doi.org
```

The 706 title-level exclusions were not individually coded. They include clearly off-topic matches (for example medical records retrieved by "track confirmation"), works on unrelated platforms, and earlier versions of included works (for example the conference version of Ghabcheloo et al. 2009 and the Research Square preprint of Chandra et al. 2025).

**Reasons used at eligibility.**

{{REASONS}}

### 1.6 Limitations

The review has one screener and no second rater, so screening errors and selection bias cannot be quantified. Semantic Scholar, the source best suited to relevance-ranked retrieval, was rate-limited (HTTP 429) for 44 of the 46 queries, and IEEE Xplore, Scopus and Web of Science were not queried directly. OpenAlex's all-terms filter returned nothing for long strings, and Crossref's ranking is noisy; supplementary short strings only partly compensate. As a result, 36 of the 62 included works entered through other methods: a known-item list built from the reviewer's knowledge, known-item WebSearch queries and Crossref bibliographic lookups. This favours well-known, highly cited work and may miss recent or less visible papers. Full texts were not read for most items; relevance judgements and the technique assessments in Section 3 rest on abstracts and prior knowledge, and the WELL-SUPPORTED / SPECULATIVE labels are the reviewer's judgement. Theses and grey literature were excluded even where directly relevant (for example an MIT/WHOI thesis on absorption-based ranging for undersea swarms, and an EPFL thesis on cooperative path following under communication constraints, both seen during the WebSearch step). The scope cap forced the exclusion of relevant but redundant works (Appendix B). Statements of novelty in Section 4 hold only relative to the records screened here.

---

## 2. Per-topic synthesis and included works

### T1 Decentralised underwater multi-robot coordination

*Established.* Multi-AUV operations have mostly coordinated through explicit communication. Glider fleets were coordinated for adaptive ocean sampling with information exchanged when the gliders surfaced (Leonard et al. 2007), and recent surveys tie formation performance to acoustic communication capability (Yang et al. 2021) and catalogue the communication options open to small vehicles (Connor et al. 2021). Blueswarm showed 3D collective behaviours from local, implicit vision-based coordination with no message exchange, in a laboratory tank (Berlinger et al. 2021).

*Open.* Implicit coordination at acoustic ranges (tens of metres, beyond optical range) and coordination through constrained geometry such as gates are rarely treated. No screened record combined leaderless operation, acoustic identity pings and critical regions.

{{TABLE:T1}}

### T2 Hybrid automata and formal verification for robotic systems

*Established.* Hybrid automata have a mature semantics and reachability theory (Alur et al. 1995), with conditions for existence and uniqueness of executions and an analysis of Zeno behaviour (Lygeros et al. 2003). Multi-agent conflict resolution has been posed as a hybrid system whose manoeuvres are verified safe (Tomlin et al. 1998). Theorem proving in differential dynamic logic has verified ground-robot obstacle avoidance with sensor uncertainty and moving obstacles (Mitsch et al. 2017). Architecturally, a fixed priority among layers (Brooks 1986) and a verified safety controller that can override a complex one (Sha 2001) match the failsafe-first ordering of the automaton studied here.

*Open.* Verification where mode guards depend on probabilistic beliefs built from intermittent sensing, and where the number of interacting automata grows, remains difficult. The screened work verifies either continuous safety under bounded sensing error or discrete protocols with reliable messages, rarely both.

{{TABLE:T2}}

### T3 Collision avoidance and control barrier functions

*Established.* Control barrier functions in a quadratic program enforce forward invariance of a safe set with minimal modification of a nominal command (Ames et al. 2017), and decentralised barrier certificates extend this to robot teams (Wang et al. 2017). Reciprocal velocity obstacles (van den Berg et al. 2011) and buffered Voronoi cells (Zhou et al. 2017) need only sensed neighbour states, so they fit communication-free operation. Underwater, CBFs have been combined with behaviour trees for multi-AUV missions (Ozkahraman and Ogren 2020).

*Open.* The guarantees assume accurate, current neighbour states; acoustic relative positions arrive every TDMA cycle (seconds) with metre-level noise. Reactive barrier controllers can also stall in deadlock (Grover et al. 2023), which matters in front of a gate.

{{TABLE:T3}}

### T4 Mutual exclusion and shared narrow passages or intersections

*Established.* Mutual exclusion has a classical theory; the bakery algorithm orders processes by ticket and breaks ties by identifier (Lamport 1974). Intersection management uses reservations granted by a manager (Dresner and Stone 2008), least-restrictive supervisors that intervene only when needed (Colombo and Del Vecchio 2015), or distributed mutual exclusion over V2V messages (Wu et al. 2015). For robot fleets, critical sections on overlapping paths with precedence constraints give provable safety (Pecora et al. 2018), also under delayed or lossy communication (Mannucci et al. 2019). Leases give time-bounded exclusive access that stays safe if the holder becomes unreachable (Gray and Cheriton 1989).

*Open.* All of these rely on request, grant or release messages, or on a coordinator. Mutual exclusion in which the shared region's state is only sensed (no messages of any kind) was not found in the screened records.

{{TABLE:T4}}

### T5 Deadlock avoidance and liveness in multi-robot systems

*Established.* Deadlock requires mutual exclusion, hold-and-wait, no preemption and circular wait, and can be prevented, avoided or detected (Coffman et al. 1971). In robotics, reactive CBF controllers have characterised deadlock sets and escape strategies (Grover et al. 2023). Decentralised, non-communicative navigation through doorways and corridor intersections with deadlock-free guarantees has been shown on physical robots (Chandra et al. 2025), and decentralised priorities avoid deadlock in dense, narrow settings (Zheng and Li 2024). Plan execution under delaying disturbances can be made safe and deadlock-free by precedence-respecting waiting (Cap et al. 2016), and AGV fleets use priorities with private zones to avoid deadlock and livelock (Draganjac et al. 2016).

*Open.* Liveness analysis usually assumes correct knowledge of other agents. False beliefs (a phantom occupant, a missed vehicle) and slow, intermittent acoustic updates in 3D are not covered.

{{TABLE:T5}}

### T6 Sonar-only perception and its observability limits

*Established.* Wide-angle sonar returns are angularly ambiguous; probabilistic occupancy maps combine many returns to recover structure (Moravec and Elfes 1985). Sonar physics (specular reflection, multiple echoes, crosstalk) limits what a single beam reveals (Kleeman and Kuc 2016). Underwater, forward-looking sonar supports obstacle detection and avoidance (Petillot et al. 2001), and sonar-based SLAM in man-made structured environments localises the vehicle against structural features (Ribas et al. 2008). For relative localisation, observability of single-range navigation depends on the trajectory, which must be sufficiently rich (Batista et al. 2011).

*Open.* A single wide-cone echo cannot tell a neighbour from structure or say which neighbour it is. Occupancy of a gate corridor is therefore only partially observable from sonar alone, which motivates fusion with the acoustic identity pings.

{{TABLE:T6}}

### T7 Underwater acoustic localisation, one-way travel-time ranging and USBL

*Established.* With synchronised clocks, a receiver obtains range from one-way travel time of a scheduled broadcast, so any number of vehicles can listen passively (Eustice et al. 2011), and single-beacon OWTT navigation has been validated in the field (Webster et al. 2012). Cooperative localisation with acoustic ranging and exchanged navigation data is well developed (Bahr et al. 2009), and the review by Paull et al. (2014) places these methods among inertial and geophysical alternatives.

*Open.* OWTT accuracy depends on the clock-offset budget, and TDMA cycle length grows with the number of vehicles, which lowers the update rate per neighbour.

{{TABLE:T7}}

### T8 Passive reception of identity beacons and one-way broadcasts for relative localisation

*Established.* A passive receive array on the vehicle (inverted USBL) yields range and bearing to a broadcasting beacon on low-cost AUVs (Rypkema et al. 2017). In ASV experiments, piUSBL range and angle errors were reported as 0.03 ± 1.49 m and -0.11 ± 3.16 degrees (mean ± standard deviation; Rypkema and Schmidt 2019). Three SandShark AUVs navigated relative to a single periodically broadcasting beacon, with coordination achieved without inter-vehicle communication (Rypkema et al. 2022). Single-hydrophone followers have tracked a leader's multi-frequency source without time synchronisation (Fischell et al. 2020). Time synchronisation in high-latency acoustic networks has been addressed directly (Syed and Heidemann 2006).

*Open.* The demonstrated systems use one dedicated beacon or leader. Peer-to-peer identity pings from every vehicle, used both for relative localisation and as evidence of presence or occupancy, were not found in the screened records.

{{TABLE:T8}}

### T9 Formation keeping and recovery under disturbances

*Established.* Formations can be defined as slots in a virtual rigid structure (Lewis and Tan 1997) or classified by sensing and interaction topology (Oh et al. 2015). Coordinated path following separates path following from the synchronisation of along-path coordination states, and remains stable under communication losses and delays (Ghabcheloo et al. 2009). Range-only formation keeping has been run at sea with low-rate, unreliable acoustic ranging (Soares et al. 2013). Pulse-coupled oscillators synchronise from event-only coupling (Mirollo and Strogatz 1990).

*Open.* Synchronising formation progress from sensing alone, with no exchanged coordination state, has little direct literature.

{{TABLE:T9}}

### T10 Fault-tolerant and degraded formations

*Established.* Robots can detect failed peers from the absence of expected periodic signals (Christensen et al. 2009). Failure-detector theory separates completeness (every crashed process is eventually suspected) from accuracy (correct processes are not wrongly suspected) and shows which combinations suffice for agreement (Chandra and Toueg 1996), and accrual detectors replace a fixed timeout with a suspicion level computed from heartbeat statistics (Hayashibara et al. 2004). Multi-AUV formations have been reconfigured under currents with affine transformations (Pang et al. 2024).

*Open.* In the T10 records screened here, underwater fault-tolerant formation work mostly addresses actuator faults with communicating controllers. Declaring a slot vacant from perception alone and re-including the vehicle later was not found.

{{TABLE:T10}}

### T11 Realistic underwater sensing, currents and communication constraints

*Established.* Acoustic networks face low bandwidth, long and variable delay and high loss (Akyildiz et al. 2005); the channel shows multipath, Doppler and fading (Stojanovic and Preisig 2009). Platoon control with broadcast-only communication was analysed early (Stilwell and Bishop 2000). Marine-craft dynamics and current models are standard (Fossen 2011), and HoloOcean simulates multi-agent sonar and acoustic links (Potokar et al. 2022).

*Open.* Results from simulated acoustic channels transfer to sea only partly; validated loss and multipath models for short-range, multi-vehicle pinging near structures are scarce in the screened records.

{{TABLE:T11}}

### X Estimation under intermittent observations (cross-cutting)

*Established.* Existence-aware filters handle missed detections and clutter: integrated PDA propagates a probability of target existence (Musicki et al. 1994), the Bernoulli filter is the Bayes-optimal filter for one object that may or may not be present (Ristic et al. 2013) and has been applied to underwater acoustic targets observed with a vector sensor (Gunes and Guldogan 2016). Multiple-hypothesis tracking defers association decisions and uses explicit track confirmation and deletion (Blackman 2004). The Bayesian occupancy filter propagates grid occupancy through short occlusions (Coue et al. 2006). The sequential probability ratio test reaches a decision with bounded error probabilities (Wald 1945).

*Open.* These tools assume calibrated detection and false-alarm models; for wide-cone sonar near structure such models must be identified per sensor and per geometry.

{{TABLE:X}}

---

## 3. Techniques applicable to the system

All techniques below run onboard from each vehicle's own sonar, navigation and received pings. None adds a message, payload, acknowledgement or coordinator. Labels: **WELL-SUPPORTED** = demonstrated in a comparable setting (estimation under missed detections, robot fleets, acoustic navigation); **SPECULATIVE** = plausible transfer that the literature does not demonstrate for this setting. Complexity is a qualitative estimate of implementation and tuning effort.

### I1 Occupancy verification of a narrow passage with intermittent, ambiguous observations

| Technique | Fit to the architecture (no communication, no coordinator) | Expected benefit | Complexity | Label |
|---|---|---|---|---|
| Bernoulli / IPDA existence filter for "gate occupied" (Ristic et al. 2013; Musicki et al. 1994; Gunes and Guldogan 2016) | Replace the ad hoc occupancy belief by one existence probability r per gate. Measurements: sonar echoes inside the corridor polygon (detection probability from beam geometry, clutter rate near walls), ping-derived positions inside the gate (with ID). Birth = entry, death = exit, both with set probabilities per cycle. | One calibrated number that fuses sonar and acoustic evidence, decays at a known rate without evidence, and gives tunable false-clear and false-occupied rates. | Low | WELL-SUPPORTED (as an estimator; not yet shown for robot CR occupancy) |
| Corridor occupancy grid with wide-beam inverse sensor model, plus occupancy-filter prediction (Moravec and Elfes 1985; Coue et al. 2006) | A small grid restricted to the gate corridor, updated from the six wide-cone sonars and own pose; known gate walls masked out using the map already used for relocalisation (Ribas et al. 2008). | Resolves part of the angular ambiguity by combining returns over time and pose; keeps occupancy through short dropouts. | Medium | WELL-SUPPORTED (indoor and automotive); underwater tuning needed |
| SPRT for the "exit confirmed" and "gate clear" decisions (Wald 1945) | Accumulate log-likelihood ratios of "occupied" vs "empty" from successive sonar and ping observations; declare clear only when the lower threshold is crossed, otherwise keep waiting or fall back to the timeout. | Explicit bound on the probability of entering an occupied gate, under the stated independence assumptions. | Low | WELL-SUPPORTED (statistical test); SPECULATIVE where successive sonar returns are correlated |
| Identity-keyed positive evidence (ping track crossing the exit plane) | Use the per-ID ping track (I3) as the primary exit witness, with sonar as a secondary witness; occupancy cleared by "ID k observed beyond the exit plane", not by "no echo". | Converts absence of evidence into positive evidence; reduces false clears after sonar dropouts. | Low | SPECULATIVE (follows from T7/T8 sensing; not demonstrated) |

### I2 Safe entry and guaranteed passage completion

| Technique | Fit to the architecture | Expected benefit | Complexity | Label |
|---|---|---|---|---|
| Stop-before-CR invariant with a commit point (Pecora et al. 2018; Mannucci et al. 2019; Colombo and Del Vecchio 2015) | Define a commit point at braking distance before the gate. Before it, the vehicle may still stop; past it, it completes the passage unless the failsafe fires. The commit check (occupancy belief below threshold, own precedence confirmed) is evaluated locally instead of being granted by a coordinator. | Nobody stops inside the CR in nominal operation; the safety argument of the critical-section literature carries over if the local belief is correct with known probability. | Low | WELL-SUPPORTED for the invariant (central coordinator in the sources); SPECULATIVE for the local, message-free version |
| Implicit lease on the gate (Gray and Cheriton 1989) | Treat first entry evidence for the gate as the start of a lease of length T_max (worst-case passage time plus margin) on the shared clock. Others regard the gate as occupied until positive exit evidence or lease expiry, whichever comes first. This formalises the existing timeout fallback. | A safety argument that does not need the occupant to be observed throughout, and a liveness argument (a phantom occupant expires). | Low | SPECULATIVE (leases are granted by messages in the source; here the start is inferred) |
| Simplex-style in-gate monitor (Sha 2001) | A runtime monitor checks progress rate and wall clearance inside the gate and switches to a verified simple controller (hold the corridor axis at minimum speed, or back out along it) on violation. | Bounded time inside the CR even under controller faults; maps onto the failsafe layer. | Low-medium | WELL-SUPPORTED (pattern widely used in safety-critical control) |
| Formal verification of the gate protocol (Alur et al. 1995; Tomlin et al. 1998; Mitsch et al. 2017) | Model N copies of the automaton with an abstract sensor (bounded missed-detection runs, bounded false alarms) and check mutual exclusion and eventual passage by reachability or theorem proving. | Turns the gate rules into proven properties under stated sensing assumptions; exposes corner cases such as simultaneous approach. | High | WELL-SUPPORTED as methodology (comparable verified obstacle avoidance and multi-agent conflict resolution) |

### I3 Acoustic identification and relative localisation of neighbours

| Technique | Fit to the architecture | Expected benefit | Complexity | Label |
|---|---|---|---|---|
| Synchronous-clock OWTT range plus inverted-USBL bearing with slot-based identity (Eustice et al. 2011; Rypkema et al. 2017, 2019, 2022) | Already the system's sensing model. The TDMA slot fixes both the emission time and the identity, so no data payload is needed; in single-beacon OWTT navigation the broadcast also carries data such as the beacon position (Webster et al. 2012). Use the published piUSBL error figures to set measurement noise. | Realistic noise models (about 1.5 m range and 3 degrees angle standard deviation in the piUSBL experiments) for simulation and filters. | Low | WELL-SUPPORTED |
| Per-neighbour existence-aware track (Bernoulli or IPDA) with multipath gating (Musicki et al. 1994; Ristic et al. 2013; Stojanovic and Preisig 2009) | One track per ID; gates reject multipath arrivals (late, off-bearing); existence probability falls with missed slots. | Robust neighbour estimates; a principled "neighbour present" probability reused by I4 and I5. | Low-medium | WELL-SUPPORTED |
| Association of anonymous sonar echoes with ID-labelled acoustic tracks by MHT (Blackman 2004) | Sonar echoes have no identity; MHT keeps alternative assignments (neighbour k, structure, clutter) until resolved. | Identity-labelled sonar contacts at the higher sonar rate between pings. | Medium-high | WELL-SUPPORTED in tracking; SPECULATIVE for six wide-cone single-beam sonars |
| Observability-aware use of range-only fallbacks (Batista et al. 2011; Soares et al. 2013) | When bearing is unreliable, range-only tracking needs relative motion; the formation geometry and planned path can supply it. | Avoids divergent estimates in steady formation when only ranges are trusted. | Low | WELL-SUPPORTED |

### I4 Formation rejoining and synchronisation of formation progress or clocks without communication

| Technique | Fit to the architecture | Expected benefit | Complexity | Label |
|---|---|---|---|---|
| Coordination-state synchronisation from sensed progress (Ghabcheloo et al. 2009) | Each vehicle projects each neighbour's acoustic position onto the shared path to estimate that neighbour's path parameter, then applies a bounded speed correction of the coordinated path-following type to its own private clock. This replaces the binary "neighbour missing, slow down" rule with a continuous correction. | Smooth convergence of progress; the source proves robustness to losses and delays of the coordination information, which here come from missed pings. | Medium | SPECULATIVE (sources exchange coordination states by communication; sensed substitute not demonstrated) |
| Range-only or range-bearing slot keeping (Soares et al. 2013; Rypkema et al. 2022) | Keep the slot relative to sensed neighbours rather than only to the clock when navigation drift grows. | Formation holds with dead-reckoning drift; demonstrated at sea with unreliable ranging. | Medium | WELL-SUPPORTED |
| Pulse-coupled phase alignment of private clocks (Mirollo and Strogatz 1990; Christensen et al. 2009) | Treat each received identity ping, corrected for its travel time, as a pulse that nudges the private formation phase. | Re-aligns private clocks after a degraded phase without exchanging time values. | Medium | SPECULATIVE (TDMA itself relies on synchronised clocks; risk of interference with the slot schedule) |
| Hysteresis for re-inclusion (Hayashibara et al. 2004) | Re-include a vacant slot only when the neighbour's suspicion level stays below a lower threshold for a dwell time; declare vacancy above a higher threshold. | Avoids oscillation between degraded and full formation when pings are intermittent. | Low | WELL-SUPPORTED in distributed systems; SPECULATIVE underwater |

### I5 Robustness against intermittent observations

| Technique | Fit to the architecture | Expected benefit | Complexity | Label |
|---|---|---|---|---|
| Accrual failure detection on the TDMA ping schedule (Hayashibara et al. 2004; Chandra and Toueg 1996; Christensen et al. 2009) | The expected arrival time of each ID is known, so the suspicion level can be computed from the number of consecutive missed slots and the observed loss rate; vacancy is declared on a suspicion threshold rather than a fixed timeout. | Detection delay adapts to channel quality; the false-vacancy probability can be set explicitly. | Low | WELL-SUPPORTED (heartbeat failure detection in networks and robot swarms) |
| Track confirmation and deletion logic (Blackman 2004; Musicki et al. 1994) | Apply M-of-N or score-based confirmation to sonar contacts before they can trigger yield or avoidance, and existence-based deletion before they are dropped. | Fewer spurious yields from single echoes; fewer premature clears. | Low | WELL-SUPPORTED |
| Occupancy-filter persistence and sequential tests (Coue et al. 2006; Wald 1945) | See I1; the same machinery applies to separation-warning contacts. | Stable mode switching under dropouts; fewer guard chatters (relevant to well-posedness, Lygeros et al. 2003). | Low-medium | WELL-SUPPORTED |

### I6 Deadlock prevention without centralised communication

| Technique | Fit to the architecture | Expected benefit | Complexity | Label |
|---|---|---|---|---|
| Total order from geometry with ID tie-break (Lamport 1974) | Precedence by distance to the gate along the approach path (or arrival time on the shared clock), with ties broken by TDMA slot index, which every vehicle observes for every neighbour. | Removes symmetric "after you" livelock; each vehicle can compute the same order if estimates agree within a margin. | Low | WELL-SUPPORTED (bakery tie-breaking); SPECULATIVE where estimates disagree near the tie |
| Waiting zones outside the approach and exit corridors (Draganjac et al. 2016) | Pre-allocate holding positions, in the spirit of the private zones used for lower-priority removal actions, so that yielding vehicles never block the exit path of the occupant. | Breaks the hold-and-wait chain at the gate (Coffman et al. 1971). | Low | WELL-SUPPORTED (AGV fleets) |
| Deadlock-aware avoidance layer (Grover et al. 2023; Chandra et al. 2025; Zheng and Li 2024) | Detect a near-equilibrium of the collision-avoidance/separation layers near the gate and apply a minimal, priority-dependent perturbation. | Prevents the reactive layers from stalling the gate protocol. | Medium | WELL-SUPPORTED for ground robots in doorways; SPECULATIVE underwater |
| Lease expiry and bounded waits (Gray and Cheriton 1989; Cap et al. 2016) | Every wait in the automaton is bounded (lease on the gate, bounded formation wait); expiry triggers a defined action, so no state can wait forever on a false belief. | Liveness argument under phantom occupancy or missed vehicles. | Low | SPECULATIVE (sources assume messages or shared progress information) |
| Liveness verification on a timed abstraction (Alur et al. 1995) | Model the bounded waits and leases as clock constraints and check for reachable states with no progress. | Confirms or refutes deadlock freedom for 3-6 vehicles before simulation campaigns. | Medium-high | WELL-SUPPORTED as methodology |

---

## 4. Comparison with the closest published methodologies

**Rypkema et al. (2022), synchronous-clock range-angle relative navigation.** *Similar:* synchronised clocks, OWTT range plus array bearing, passive reception, coordination of several AUVs without inter-vehicle communication. *Different:* one dedicated broadcasting beacon is the reference and the command channel (broadcast signal selects the behaviour), vehicles do not ping each other, and there are no critical regions. The system studied here makes every vehicle a beacon and removes the reference beacon.

**Fischell et al. (2020), single-hydrophone swarming.** *Similar:* passive reception of a broadcast to keep a group together cheaply. *Different:* leader-follower with an unsynchronised multi-frequency source; the studied system is leaderless and synchronised.

**Pecora et al. (2018) and Mannucci et al. (2019), critical sections with precedence.** *Similar:* shared regions treated as critical sections, a stop-before-entry rule, precedence constraints, provable safety including degraded information. *Different:* a central coordinator computes and revises precedences from robot-reported states over a network; here precedence and occupancy are inferred locally from sensing.

**Chandra et al. (2025); Zheng and Li (2024); Grover et al. (2023), decentralised passage and deadlock handling.** *Similar:* decentralised passage of doorways, corridors and narrow spaces with explicit deadlock treatment; Chandra et al. state a non-communicative, partially observable setting, and Grover et al. analyse when barrier-based controllers deadlock. *Different:* continuous control (discrete-time CBFs within receding-horizon control, or velocity-level rules) of ground robots; no explicit occupancy belief of the passage and no one-at-a-time exclusivity rule.

**Wu et al. (2015); Lamport (1974), mutual exclusion.** *Similar:* the gate is a one-at-a-time resource with an ordering rule and identifier tie-breaks. *Different:* Wu et al. exchange V2V messages; the bakery algorithm reads shared variables. The studied system replaces both with sensed state.

**Ghabcheloo et al. (2009), coordinated path following.** *Similar:* slots on a shared path and synchronisation of along-path progress, robust to missing information. *Different:* coordination states are communicated; here a vehicle only slows its own private clock when it perceives a neighbour missing.

**Christensen et al. (2009); Hayashibara et al. (2004), failure detection.** *Similar:* a neighbour is declared failed from the absence of periodic signals. *Different:* the studied system also uses the same pings for localisation and for gate occupancy, and re-includes a vehicle after vacancy.

**Tomlin et al. (1998); Mitsch et al. (2017); Brooks (1986); Sha (2001), hybrid control architectures.** *Similar:* priority-ordered modes, safety-first override, conflict resolution as a hybrid system. *Different:* verification there is done with reliable state information; here guards depend on beliefs from intermittent sensors.

**What appears already known.** OWTT and inverted-USBL relative navigation; heartbeat-style failure detection; critical sections with stop-before-entry; priority-layered and Simplex-style architectures; barrier-function and reciprocal collision avoidance; coordinated path following with synchronised progress.

**What appears new relative to the screened records.** (a) Peer-to-peer, payload-free identity pings from every vehicle, used at once for relative localisation, presence detection and gate-occupancy evidence; (b) mutual exclusion of a physical critical region with no request, grant or release messages, based on a sensed occupancy belief and locally computed precedence; (c) formation degradation and recovery driven by perceived absence through a private clock slowdown, a bounded wait, a vacancy declaration and later re-inclusion. These statements are limited by the search described in Section 1 and should be rechecked against IEEE Xplore and Scopus before being used in a manuscript.

---

## 5. Overlap with the project bibliography (`references.bib`, read-only, compared last)

`references.bib` has 77 entries, of which 30 carry a DOI; many entries are datasheets and product pages without DOIs. **7 of the 62 included works share a DOI with `references.bib`:**

1. Paull et al. (2014), AUV Navigation and Localization: A Review, 10.1109/joe.2013.2278891
2. Eustice et al. (2011), Synchronous-clock, one-way-travel-time acoustic navigation for underwater vehicles, 10.1002/rob.20365
3. Webster et al. (2012), Advances in single-beacon one-way-travel-time acoustic navigation for underwater vehicles, 10.1177/0278364912446166
4. Rypkema et al. (2017), One-way travel-time inverted ultra-short baseline localization for low-cost autonomous underwater vehicles, 10.1109/icra.2017.7989570
5. Rypkema and Schmidt (2019), Passive Inverted Ultra-Short Baseline (piUSBL) Localization: An Experimental Evaluation of Accuracy, 10.1109/iros40897.2019.8967800
6. Rypkema et al. (2022), Synchronous-Clock Range-Angle Relative Acoustic Navigation, 10.55417/fr.2022026
7. Potokar et al. (2022), HoloOcean: An Underwater Robotics Simulator, 10.1109/icra46639.2022.9812353

No further title-level matches were found among the entries without DOI. Two more `references.bib` works appeared in this review's pipeline but were not included: Chitre et al. (2008), assessed and excluded as redundant (10.4031/002533208786861263), and the ahoi modem paper (10.1145/3376921), retrieved by query T8sq2 but not retained at screening. The overlap is concentrated in T7, T8 and T11 (acoustic navigation and simulation). Judging by titles, `references.bib` contains no entry for topics T2-T5, T9, T10 or X, so the remaining 55 included works are candidates for extending it.

---

## 6. BibTeX of included and verified works (62 entries, generated from Crossref records)

```bibtex
{{BIB}}
```

---

## Appendix A. Per-query records by source

{{QUERIES}}

## Appendix B. Works excluded at eligibility (57)

{{EXCLUDED}}
