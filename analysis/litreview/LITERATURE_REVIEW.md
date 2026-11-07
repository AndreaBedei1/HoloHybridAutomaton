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

- **R1**: Redundant with an included work that covers the same contribution (scope cap of about 60 items) (n = 28)
- **R2**: Superseded by a later version of the same work (n = 1)
- **R3**: Depends on inter-vehicle message exchange or a central coordinator; the contrast is already represented by an included work (n = 12)
- **R4**: Platform or problem setting less transferable to the system (surface vessels, spacecraft, manipulators, vision, broad survey) or tangential (n = 16)

### 1.6 Limitations

The review has one screener and no second rater, so screening errors and selection bias cannot be quantified. Semantic Scholar, the source best suited to relevance-ranked retrieval, was rate-limited (HTTP 429) for 44 of the 46 queries, and IEEE Xplore, Scopus and Web of Science were not queried directly. OpenAlex's all-terms filter returned nothing for long strings, and Crossref's ranking is noisy; supplementary short strings only partly compensate. As a result, 36 of the 62 included works entered through other methods: a known-item list built from the reviewer's knowledge, known-item WebSearch queries and Crossref bibliographic lookups. This favours well-known, highly cited work and may miss recent or less visible papers. Full texts were not read for most items; relevance judgements and the technique assessments in Section 3 rest on abstracts and prior knowledge, and the WELL-SUPPORTED / SPECULATIVE labels are the reviewer's judgement. Theses and grey literature were excluded even where directly relevant (for example an MIT/WHOI thesis on absorption-based ranging for undersea swarms, and an EPFL thesis on cooperative path following under communication constraints, both seen during the WebSearch step). The scope cap forced the exclusion of relevant but redundant works (Appendix B). Statements of novelty in Section 4 hold only relative to the records screened here.

---

## 2. Per-topic synthesis and included works

### T1 Decentralised underwater multi-robot coordination

*Established.* Multi-AUV operations have mostly coordinated through explicit communication. Glider fleets were coordinated for adaptive ocean sampling with information exchanged when the gliders surfaced (Leonard et al. 2007), and recent surveys tie formation performance to acoustic communication capability (Yang et al. 2021) and catalogue the communication options open to small vehicles (Connor et al. 2021). Blueswarm showed 3D collective behaviours from local, implicit vision-based coordination with no message exchange, in a laboratory tank (Berlinger et al. 2021).

*Open.* Implicit coordination at acoustic ranges (tens of metres, beyond optical range) and coordination through constrained geometry such as gates are rarely treated. No screened record combined leaderless operation, acoustic identity pings and critical regions.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Leonard, N. E., Paley, D. A., Lekien, F., Sepulchre, R., Fratantoni, D. M. and Davis, R. E. (2007). Collective Motion, Sensor Networks, and Ocean Sampling. *Proceedings of the IEEE*, 95(1), 48-74. | [10.1109/jproc.2006.887295](https://doi.org/10.1109/jproc.2006.887295) | yes | Glider-fleet coordination for ocean sampling; reference point for underwater collective motion where vehicles share information only intermittently. |
| Yang, Y., Xiao, Y. and Li, T. (2021). A Survey of Autonomous Underwater Vehicle Formation: Performance, Formation Control, and Communication Capability. *IEEE Communications Surveys & Tutorials*, 23(2), 815-841. | [10.1109/comst.2021.3059998](https://doi.org/10.1109/comst.2021.3059998) | yes | Survey of AUV formation control that ties formation performance to acoustic communication capability; frames the communication-limited setting. |
| Connor, J., Champion, B. and Joordens, M. A. (2021). Current Algorithms, Communication Methods and Designs for Underwater Swarm Robotics: A Review. *IEEE Sensors Journal*, 21(1), 153-169. | [10.1109/jsen.2020.3013265](https://doi.org/10.1109/jsen.2020.3013265) | yes | Review of underwater swarm algorithms, communication methods and vehicle designs; maps the communication options open to small vehicles. |
| Berlinger, F., Gauci, M. and Nagpal, R. (2021). Implicit coordination for 3D underwater collective behaviors in a fish-inspired robot swarm. *Science Robotics*, 6(50), eabd8668. | [10.1126/scirobotics.abd8668](https://doi.org/10.1126/scirobotics.abd8668) | yes | Blueswarm: 3D underwater collective behaviours from local implicit (vision-based) coordination with no message exchange; closest precedent for implicit coordination underwater. |

### T2 Hybrid automata and formal verification for robotic systems

*Established.* Hybrid automata have a mature semantics and reachability theory (Alur et al. 1995), with conditions for existence and uniqueness of executions and an analysis of Zeno behaviour (Lygeros et al. 2003). Multi-agent conflict resolution has been posed as a hybrid system whose manoeuvres are verified safe (Tomlin et al. 1998). Theorem proving in differential dynamic logic has verified ground-robot obstacle avoidance with sensor uncertainty and moving obstacles (Mitsch et al. 2017). Architecturally, a fixed priority among layers (Brooks 1986) and a verified safety controller that can override a complex one (Sha 2001) match the failsafe-first ordering of the automaton studied here.

*Open.* Verification where mode guards depend on probabilistic beliefs built from intermittent sensing, and where the number of interacting automata grows, remains difficult. The screened work verifies either continuous safety under bounded sensing error or discrete protocols with reliable messages, rarely both.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Alur, R., Courcoubetis, C., Halbwachs, N., Henzinger, T. A., Ho, P.-H., Nicollin, X., Olivero, A., Sifakis, J. and Yovine, S. (1995). The algorithmic analysis of hybrid systems. *Theoretical Computer Science*, 138(1), 3-34. | [10.1016/0304-3975(94)00202-t](https://doi.org/10.1016/0304-3975(94)00202-t) | yes | Foundational hybrid-automaton semantics and reachability analysis; basis for modelling the per-vehicle mode logic and verifying safety properties. |
| Lygeros, J., Johansson, K. H., Simic, S. N., Zhang, J. and Sastry, S. S. (2003). Dynamical properties of hybrid automata. *IEEE Transactions on Automatic Control*, 48(1), 2-17. | [10.1109/tac.2002.806650](https://doi.org/10.1109/tac.2002.806650) | yes | Existence, uniqueness and Zeno behaviour of hybrid-automaton executions; relevant to guard design that avoids chattering between priority modes. |
| Tomlin, C., Pappas, G. J. and Sastry, S. (1998). Conflict resolution for air traffic management: a study in multiagent hybrid systems. *IEEE Transactions on Automatic Control*, 43(4), 509-521. | [10.1109/9.664154](https://doi.org/10.1109/9.664154) | yes | Multi-agent hybrid-system conflict resolution with verified manoeuvres; methodological template for a verified gate/yield protocol. |
| Mitsch, S., Ghorbal, K., Vogelbacher, D. and Platzer, A. (2017). Formal verification of obstacle avoidance and navigation of ground robots. *The International Journal of Robotics Research*, 36(12), 1312-1340. | [10.1177/0278364917733549](https://doi.org/10.1177/0278364917733549) | yes | Formally verified (differential dynamic logic) obstacle avoidance with sensor uncertainty and moving obstacles; template for verifying collision avoidance under imperfect sensing. |
| Brooks, R. (1986). A robust layered control system for a mobile robot. *IEEE Journal on Robotics and Automation*, 2(1), 14-23. | [10.1109/jra.1986.1087032](https://doi.org/10.1109/jra.1986.1087032) | yes | Layered priority (subsumption) control; ancestor of the fixed priority order failsafe > avoidance > ... > formation keeping. |
| Sha, L. (2001). Using simplicity to control complexity. *IEEE Software*, 18(4), 20-28. | [10.1109/ms.2001.936213](https://doi.org/10.1109/ms.2001.936213) | yes | Simplex architecture: a simple verified safety controller supervises a complex controller; pattern for failsafe and watchdog layers. |

### T3 Collision avoidance and control barrier functions

*Established.* Control barrier functions in a quadratic program enforce forward invariance of a safe set with minimal modification of a nominal command (Ames et al. 2017), and decentralised barrier certificates extend this to robot teams (Wang et al. 2017). Reciprocal velocity obstacles (van den Berg et al. 2011) and buffered Voronoi cells (Zhou et al. 2017) need only sensed neighbour states, so they fit communication-free operation. Underwater, CBFs have been combined with behaviour trees for multi-AUV missions (Ozkahraman and Ogren 2020).

*Open.* The guarantees assume accurate, current neighbour states; acoustic relative positions arrive every TDMA cycle (seconds) with metre-level noise. Reactive barrier controllers can also stall in deadlock (Grover et al. 2023), which matters in front of a gate.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Ames, A. D., Xu, X., Grizzle, J. W. and Tabuada, P. (2017). Control Barrier Function Based Quadratic Programs for Safety Critical Systems. *IEEE Transactions on Automatic Control*, 62(8), 3861-3876. | [10.1109/tac.2016.2638961](https://doi.org/10.1109/tac.2016.2638961) | yes | CBF-based quadratic programs that enforce forward invariance of a safe set; standard tool for the collision-avoidance and separation layers. |
| Wang, L., Ames, A. D. and Egerstedt, M. (2017). Safety Barrier Certificates for Collisions-Free Multirobot Systems. *IEEE Transactions on Robotics*, 33(3), 661-674. | [10.1109/tro.2017.2659727](https://doi.org/10.1109/tro.2017.2659727) | yes | Decentralised safety barrier certificates for multi-robot collision avoidance from neighbour states; basis of barrier-based separation layers. |
| van den Berg, J., Guy, S. J., Lin, M. and Manocha, D. (2011). Reciprocal n-Body Collision Avoidance. *Springer Tracts in Advanced Robotics*, 3-19. | [10.1007/978-3-642-19457-3_1](https://doi.org/10.1007/978-3-642-19457-3_1) | yes | ORCA: reciprocal collision avoidance where each agent takes half the responsibility using only observed neighbour states. |
| Zhou, D., Wang, Z., Bandyopadhyay, S. and Schwager, M. (2017). Fast, On-line Collision Avoidance for Dynamic Vehicles Using Buffered Voronoi Cells. *IEEE Robotics and Automation Letters*, 2(2), 1047-1054. | [10.1109/lra.2017.2656241](https://doi.org/10.1109/lra.2017.2656241) | yes | Buffered Voronoi cells: collision avoidance that needs only sensed neighbour positions, with no communication. |
| Ozkahraman, O. and Ogren, P. (2020). Combining Control Barrier Functions and Behavior Trees for Multi-Agent Underwater Coverage Missions. *2020 59th IEEE Conference on Decision and Control (CDC)*, 5275-5282. | [10.1109/cdc42340.2020.9304151](https://doi.org/10.1109/cdc42340.2020.9304151) | yes | Combines CBFs with behaviour trees for multi-AUV underwater missions; close analogue of a priority-structured mode logic with barrier constraints. |

### T4 Mutual exclusion and shared narrow passages or intersections

*Established.* Mutual exclusion has a classical theory; the bakery algorithm orders processes by ticket and breaks ties by identifier (Lamport 1974). Intersection management uses reservations granted by a manager (Dresner and Stone 2008), least-restrictive supervisors that intervene only when needed (Colombo and Del Vecchio 2015), or distributed mutual exclusion over V2V messages (Wu et al. 2015). For robot fleets, critical sections on overlapping paths with precedence constraints give provable safety (Pecora et al. 2018), also under delayed or lossy communication (Mannucci et al. 2019). Leases give time-bounded exclusive access that stays safe if the holder becomes unreachable (Gray and Cheriton 1989).

*Open.* All of these rely on request, grant or release messages, or on a coordinator. Mutual exclusion in which the shared region's state is only sensed (no messages of any kind) was not found in the screened records.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Lamport, L. (1974). A new solution of Dijkstra's concurrent programming problem. *Communications of the ACM*, 17(8), 453-455. | [10.1145/361082.361093](https://doi.org/10.1145/361082.361093) | yes | Bakery mutual exclusion: order by ticket with ties broken by process identifier; template for precedence from local queue order plus ID tie-break. |
| Dresner, K. and Stone, P. (2008). A Multiagent Approach to Autonomous Intersection Management. *Journal of Artificial Intelligence Research*, 31, 591-656. | [10.1613/jair.2502](https://doi.org/10.1613/jair.2502) | yes | Reservation-based autonomous intersection management (central manager, vehicle requests); reference design for space-time reservation of a shared region. |
| Colombo, A. and Del Vecchio, D. (2015). Least Restrictive Supervisors for Intersection Collision Avoidance: A Scheduling Approach. *IEEE Transactions on Automatic Control*, 60(6), 1515-1527. | [10.1109/tac.2014.2381453](https://doi.org/10.1109/tac.2014.2381453) | yes | Least-restrictive supervisor for intersection collision avoidance via scheduling; formal basis for intervening only when entry would become unsafe. |
| Wu, W., Zhang, J., Luo, A. and Cao, J. (2015). Distributed Mutual Exclusion Algorithms for Intersection Traffic Control. *IEEE Transactions on Parallel and Distributed Systems*, 26(1), 65-74. | [10.1109/tpds.2013.2297097](https://doi.org/10.1109/tpds.2013.2297097) | yes | Maps intersection crossing to distributed mutual exclusion over V2V messages; shows the protocol view of a critical region (requires messages). |
| Pecora, F., Andreasson, H., Mansouri, M. and Petkov, V. (2018). A Loosely-Coupled Approach for Multi-Robot Coordination, Motion Planning and Control. *Proceedings of the International Conference on Automated Planning and Scheduling*, 28, 485-493. | [10.1609/icaps.v28i1.13923](https://doi.org/10.1609/icaps.v28i1.13923) | yes | Critical sections on overlapping paths with online precedence constraints; robots stop before a critical section unless granted (central coordinator). |
| Mannucci, A., Pallottino, L. and Pecora, F. (2019). Provably Safe Multi-Robot Coordination With Unreliable Communication. *IEEE Robotics and Automation Letters*, 4(4), 3232-3239. | [10.1109/lra.2019.2924849](https://doi.org/10.1109/lra.2019.2924849) | yes | Extends critical-section coordination to delayed/lossy communication with safety proofs; closest formal treatment of CR safety under degraded information. |
| Gray, C. and Cheriton, D. (1989). Leases: an efficient fault-tolerant mechanism for distributed file cache consistency. *Proceedings of the twelfth ACM symposium on Operating systems principles*, 202-210. | [10.1145/74850.74870](https://doi.org/10.1145/74850.74870) | yes | Leases: time-bounded grants that remain safe when the holder becomes unreachable; formal analogue of the occupancy timeout fallback. |

### T5 Deadlock avoidance and liveness in multi-robot systems

*Established.* Deadlock requires mutual exclusion, hold-and-wait, no preemption and circular wait, and can be prevented, avoided or detected (Coffman et al. 1971). In robotics, reactive CBF controllers have characterised deadlock sets and escape strategies (Grover et al. 2023). Decentralised, non-communicative navigation through doorways and corridor intersections with deadlock-free guarantees has been shown on physical robots (Chandra et al. 2025), and decentralised priorities avoid deadlock in dense, narrow settings (Zheng and Li 2024). Plan execution under delaying disturbances can be made safe and deadlock-free by precedence-respecting waiting (Cap et al. 2016), and AGV fleets use priorities with private zones to avoid deadlock and livelock (Draganjac et al. 2016).

*Open.* Liveness analysis usually assumes correct knowledge of other agents. False beliefs (a phantom occupant, a missed vehicle) and slow, intermittent acoustic updates in 3D are not covered.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Coffman, E. G., Elphick, M. and Shoshani, A. (1971). System Deadlocks. *ACM Computing Surveys*, 3(2), 67-78. | [10.1145/356586.356588](https://doi.org/10.1145/356586.356588) | yes | Necessary conditions for deadlock (mutual exclusion, hold-and-wait, no preemption, circular wait) and prevention/avoidance/detection strategies. |
| Cap, M., Gregoire, J. and Frazzoli, E. (2016). Provably safe and deadlock-free execution of multi-robot plans under delaying disturbances. *2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 5113-5118. | [10.1109/iros.2016.7759750](https://doi.org/10.1109/iros.2016.7759750) | yes | Safe and deadlock-free execution of coordinated plans when robots are delayed; precedence-respecting waiting rule with proofs. |
| Grover, J., Liu, C. and Sycara, K. (2023). The Before, During, and After of Multi-robot Deadlock. *The International Journal of Robotics Research*, 42(6), 317-336. | [10.1177/02783649221074718](https://doi.org/10.1177/02783649221074718) | yes | Characterises when CBF-controlled multi-robot systems deadlock and how to leave deadlock; directly relevant to avoidance layers stalling at a gate. |
| Chandra, R., Zinage, V., Bakolas, E., Stone, P. and Biswas, J. (2025). Deadlock-free, safe, and decentralized multi-robot navigation in social mini-games via discrete-time control barrier functions. *Autonomous Robots*, 49(2), 12. | [10.1007/s10514-025-10194-8](https://doi.org/10.1007/s10514-025-10194-8) | yes | Non-communicative, decentralised doorway and intersection navigation with deadlock-free guarantees via discrete-time CBFs; closest decentralised narrow-passage method. |
| Zheng, R. and Li, S. (2024). MCCA: A Decentralized Method for Collision and Deadlock Avoidance With Nonholonomic Robots. *IEEE Robotics and Automation Letters*, 9(3), 2710-2717. | [10.1109/lra.2024.3358623](https://doi.org/10.1109/lra.2024.3358623) | yes | Fully decentralised collision and deadlock avoidance in dense, narrow environments using decentralised priorities and masked velocities. |
| Draganjac, I., Miklic, D., Kovacic, Z., Vasiljevic, G. and Bogdan, S. (2016). Decentralized Control of Multi-AGV Systems in Autonomous Warehousing Applications. *IEEE Transactions on Automation Science and Engineering*, 13(4), 1433-1447. | [10.1109/tase.2016.2603781](https://doi.org/10.1109/tase.2016.2603781) | yes | Decentralised multi-AGV coordination with priority-based conflict resolution and private zones; analysed for deadlock and livelock freedom. |

### T6 Sonar-only perception and its observability limits

*Established.* Wide-angle sonar returns are angularly ambiguous; probabilistic occupancy maps combine many returns to recover structure (Moravec and Elfes 1985). Sonar physics (specular reflection, multiple echoes, crosstalk) limits what a single beam reveals (Kleeman and Kuc 2016). Underwater, forward-looking sonar supports obstacle detection and avoidance (Petillot et al. 2001), and sonar-based SLAM in man-made structured environments localises the vehicle against structural features (Ribas et al. 2008). For relative localisation, observability of single-range navigation depends on the trajectory, which must be sufficiently rich (Batista et al. 2011).

*Open.* A single wide-cone echo cannot tell a neighbour from structure or say which neighbour it is. Occupancy of a gate corridor is therefore only partially observable from sonar alone, which motivates fusion with the acoustic identity pings.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Moravec, H. and Elfes, A. (1985). High resolution maps from wide angle sonar. *Proceedings. 1985 IEEE International Conference on Robotics and Automation*, 2, 116-121. | [10.1109/robot.1985.1087316](https://doi.org/10.1109/robot.1985.1087316) | yes | Probabilistic occupancy maps from wide-angle sonar; the original treatment of the angular ambiguity of wide-cone single-beam returns. |
| Kleeman, L. and Kuc, R. (2016). Sonar Sensing. *Springer Handbooks*, 753-782. | [10.1007/978-3-319-32552-1_30](https://doi.org/10.1007/978-3-319-32552-1_30) | yes | Handbook chapter on sonar physics (beam patterns, specular reflection, multiple echoes, crosstalk) that bounds what single-beam sonar can observe. |
| Petillot, Y., Ruiz, I. T. and Lane, D. M. (2001). Underwater vehicle obstacle avoidance and path planning using a multi-beam forward looking sonar. *IEEE Journal of Oceanic Engineering*, 26(2), 240-251. | [10.1109/48.922790](https://doi.org/10.1109/48.922790) | yes | AUV obstacle detection, tracking and avoidance from forward-looking sonar; underwater sonar-only perception for collision avoidance. |
| Ribas, D., Ridao, P., Tardós, J. D. and Neira, J. (2008). Underwater SLAM in man-made structured environments. *Journal of Field Robotics*, 25(11-12), 898-921. | [10.1002/rob.20249](https://doi.org/10.1002/rob.20249) | yes | Sonar-based SLAM in man-made structured underwater environments using structural features; precedent for localisation against known structure. |
| Batista, P., Silvestre, C. and Oliveira, P. (2011). Single range aided navigation and source localization: Observability and filter design. *Systems & Control Letters*, 60(8), 665-673. | [10.1016/j.sysconle.2011.05.004](https://doi.org/10.1016/j.sysconle.2011.05.004) | yes | Observability analysis and filter design for single-range navigation; formalises when range-only relative information is sufficient. |

### T7 Underwater acoustic localisation, one-way travel-time ranging and USBL

*Established.* With synchronised clocks, a receiver obtains range from one-way travel time of a scheduled broadcast, so any number of vehicles can listen passively (Eustice et al. 2011), and single-beacon OWTT navigation has been validated in the field (Webster et al. 2012). Cooperative localisation with acoustic ranging and exchanged navigation data is well developed (Bahr et al. 2009), and the review by Paull et al. (2014) places these methods among inertial and geophysical alternatives.

*Open.* OWTT accuracy depends on the clock-offset budget, and TDMA cycle length grows with the number of vehicles, which lowers the update rate per neighbour.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Paull, L., Saeedi, S., Seto, M. and Li, H. (2014). AUV Navigation and Localization: A Review. *IEEE Journal of Oceanic Engineering*, 39(1), 131-149. | [10.1109/joe.2013.2278891](https://doi.org/10.1109/joe.2013.2278891) | yes | Review of AUV navigation (inertial/DVL, acoustic LBL/USBL/OWTT, geophysical); context for dead reckoning plus acoustic aiding. |
| Eustice, R. M., Singh, H. and Whitcomb, L. L. (2011). Synchronous-clock, one-way-travel-time acoustic navigation for underwater vehicles. *Journal of Field Robotics*, 28(1), 121-136. | [10.1002/rob.20365](https://doi.org/10.1002/rob.20365) | yes | Synchronous-clock one-way-travel-time ranging for underwater vehicles; the basis of passive OWTT range from scheduled broadcasts. |
| Webster, S. E., Eustice, R. M., Singh, H. and Whitcomb, L. L. (2012). Advances in single-beacon one-way-travel-time acoustic navigation for underwater vehicles. *The International Journal of Robotics Research*, 31(8), 935-950. | [10.1177/0278364912446166](https://doi.org/10.1177/0278364912446166) | yes | Single-beacon OWTT navigation results in field trials; error characteristics of one-way ranging from a broadcasting source. |
| Bahr, A., Leonard, J. J. and Fallon, M. F. (2009). Cooperative Localization for Autonomous Underwater Vehicles. *The International Journal of Robotics Research*, 28(6), 714-728. | [10.1177/0278364908100561](https://doi.org/10.1177/0278364908100561) | yes | Cooperative AUV localisation with acoustic ranging and shared navigation data; contrast case that requires exchanging position estimates. |

### T8 Passive reception of identity beacons and one-way broadcasts for relative localisation

*Established.* A passive receive array on the vehicle (inverted USBL) yields range and bearing to a broadcasting beacon on low-cost AUVs (Rypkema et al. 2017). In ASV experiments, piUSBL range and angle errors were reported as 0.03 ± 1.49 m and -0.11 ± 3.16 degrees (mean ± standard deviation; Rypkema and Schmidt 2019). Three SandShark AUVs navigated relative to a single periodically broadcasting beacon, with coordination achieved without inter-vehicle communication (Rypkema et al. 2022). Single-hydrophone followers have tracked a leader's multi-frequency source without time synchronisation (Fischell et al. 2020). Time synchronisation in high-latency acoustic networks has been addressed directly (Syed and Heidemann 2006).

*Open.* The demonstrated systems use one dedicated beacon or leader. Peer-to-peer identity pings from every vehicle, used both for relative localisation and as evidence of presence or occupancy, were not found in the screened records.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Rypkema, N. R., Fischell, E. M. and Schmidt, H. (2017). One-way travel-time inverted ultra-short baseline localization for low-cost autonomous underwater vehicles. *2017 IEEE International Conference on Robotics and Automation (ICRA)*, 4920-4926. | [10.1109/icra.2017.7989570](https://doi.org/10.1109/icra.2017.7989570) | yes | OWTT inverted USBL: a passive receiver array gives range and bearing to a broadcasting beacon on a low-cost AUV; core sensing model of the system. |
| Rypkema, N. R. and Schmidt, H. (2019). Passive Inverted Ultra-Short Baseline (piUSBL) Localization: An Experimental Evaluation of Accuracy. *2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)*, 7197-7204. | [10.1109/iros40897.2019.8967800](https://doi.org/10.1109/iros40897.2019.8967800) | yes | Experimental accuracy of passive inverted USBL (piUSBL) range and angle; empirical error figures for the ping-based relative localisation. |
| Rypkema, N. R., Schmidt, H. and Fischell, E. M. (2022). Synchronous-Clock Range-Angle Relative Acoustic Navigation: A Unified Approach to Multi-AUV Localization, Command, Control, and Coordination. *Field Robotics*, 2, 774-806. | [10.55417/fr.2022026](https://doi.org/10.55417/fr.2022026) | yes | Synchronous-clock range-angle navigation of several AUVs relative to one broadcasting beacon, coordinated without inter-vehicle communication; closest sensing architecture. |
| Fischell, E. M., Kroo, A. R. and O'Neill, B. W. (2020). Single-Hydrophone Low-Cost Underwater Vehicle Swarming. *IEEE Robotics and Automation Letters*, 5(2), 354-361. | [10.1109/lra.2019.2958774](https://doi.org/10.1109/lra.2019.2958774) | yes | Leader-follower underwater swarming with a single hydrophone and no time synchronisation; low-cost passive reception of a broadcast for relative navigation. |
| Syed, A. A. and Heidemann, J. (2006). Time Synchronization for High Latency Acoustic Networks. *Proceedings IEEE INFOCOM 2006. 25TH IEEE International Conference on Computer Communications*, 1-12. | [10.1109/infocom.2006.161](https://doi.org/10.1109/infocom.2006.161) | yes | Time synchronisation for high-latency acoustic networks; relevant to the synchronised-clock assumption behind OWTT and TDMA. |

### T9 Formation keeping and recovery under disturbances

*Established.* Formations can be defined as slots in a virtual rigid structure (Lewis and Tan 1997) or classified by sensing and interaction topology (Oh et al. 2015). Coordinated path following separates path following from the synchronisation of along-path coordination states, and remains stable under communication losses and delays (Ghabcheloo et al. 2009). Range-only formation keeping has been run at sea with low-rate, unreliable acoustic ranging (Soares et al. 2013). Pulse-coupled oscillators synchronise from event-only coupling (Mirollo and Strogatz 1990).

*Open.* Synchronising formation progress from sensing alone, with no exchanged coordination state, has little direct literature.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Ghabcheloo, R., Aguiar, A. P., Pascoal, A., Silvestre, C., Kaminer, I. and Hespanha, J. (2009). Coordinated Path-Following in the Presence of Communication Losses and Time Delays. *SIAM Journal on Control and Optimization*, 48(1), 234-265. | [10.1137/060678993](https://doi.org/10.1137/060678993) | yes | Coordinated path following that synchronises along-path coordination states under communication losses and delays; formal model of formation progress synchronisation. |
| Lewis, M. A. and Tan, K. (1997). High Precision Formation Control of Mobile Robots Using Virtual Structures. *Autonomous Robots*, 4(4), 387-403. | [10.1023/a:1008814708459](https://doi.org/10.1023/a:1008814708459) | yes | Virtual-structure formation control; formation slots defined on a shared rigid reference, as with slots on a shared path. |
| Oh, K., Park, M. and Ahn, H. (2015). A survey of multi-agent formation control. *Automatica*, 53, 424-440. | [10.1016/j.automatica.2014.10.022](https://doi.org/10.1016/j.automatica.2014.10.022) | yes | Survey classifying formation control by sensing and interaction topology (position, displacement, distance based). |
| Soares, J. M., Aguiar, A. P., Pascoal, A. M. and Martinoli, A. (2013). Joint ASV/AUV range-based formation control: Theory and experimental results. *2013 IEEE International Conference on Robotics and Automation*, 5579-5585. | [10.1109/icra.2013.6631378](https://doi.org/10.1109/icra.2013.6631378) | yes | Range-only formation keeping of an AUV with respect to two leaders, with unreliable low-rate acoustic ranging and sea trials. |
| Mirollo, R. E. and Strogatz, S. H. (1990). Synchronization of Pulse-Coupled Biological Oscillators. *SIAM Journal on Applied Mathematics*, 50(6), 1645-1662. | [10.1137/0150098](https://doi.org/10.1137/0150098) | yes | Pulse-coupled oscillators synchronise from event-only coupling; theoretical basis for aligning clocks or phases from pings without payload. |

### T10 Fault-tolerant and degraded formations

*Established.* Robots can detect failed peers from the absence of expected periodic signals (Christensen et al. 2009). Failure-detector theory separates completeness (every crashed process is eventually suspected) from accuracy (correct processes are not wrongly suspected) and shows which combinations suffice for agreement (Chandra and Toueg 1996), and accrual detectors replace a fixed timeout with a suspicion level computed from heartbeat statistics (Hayashibara et al. 2004). Multi-AUV formations have been reconfigured under currents with affine transformations (Pang et al. 2024).

*Open.* In the T10 records screened here, underwater fault-tolerant formation work mostly addresses actuator faults with communicating controllers. Declaring a slot vacant from perception alone and re-including the vehicle later was not found.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Christensen, A. L., O'Grady, R. and Dorigo, M. (2009). From Fireflies to Fault-Tolerant Swarms of Robots. *IEEE Transactions on Evolutionary Computation*, 13(4), 754-766. | [10.1109/tevc.2009.2017516](https://doi.org/10.1109/tevc.2009.2017516) | yes | Decentralised detection of failed robots from the absence of expected synchronised flashes; heartbeat-style failure detection in a robot swarm. |
| Chandra, T. D. and Toueg, S. (1996). Unreliable failure detectors for reliable distributed systems. *Journal of the ACM*, 43(2), 225-267. | [10.1145/226643.226647](https://doi.org/10.1145/226643.226647) | yes | Theory of unreliable failure detectors (completeness and accuracy properties); formal vocabulary for declaring a neighbour missing. |
| Hayashibara, N., Defago, X., Yared, R. and Katayama, T. (2004). The phi accrual failure detector. *Proceedings of the 23rd IEEE International Symposium on Reliable Distributed Systems, 2004.*, 66-78. | [10.1109/reldis.2004.1353004](https://doi.org/10.1109/reldis.2004.1353004) | yes | Accrual failure detector: a continuous suspicion level from heartbeat inter-arrival statistics instead of a fixed timeout. |
| Pang, W., Zhu, D. and Sun, C. (2024). Multi-AUV Formation Reconfiguration Obstacle Avoidance Algorithm Based on Affine Transformation and Improved Artificial Potential Field Under Ocean Currents Disturbance. *IEEE Transactions on Automation Science and Engineering*, 21(2), 1469-1487. | [10.1109/tase.2023.3245818](https://doi.org/10.1109/tase.2023.3245818) | yes | Multi-AUV formation reconfiguration (affine transformation) with obstacle avoidance under ocean currents; formation reshaping in the underwater domain. |

### T11 Realistic underwater sensing, currents and communication constraints

*Established.* Acoustic networks face low bandwidth, long and variable delay and high loss (Akyildiz et al. 2005); the channel shows multipath, Doppler and fading (Stojanovic and Preisig 2009). Platoon control with broadcast-only communication was analysed early (Stilwell and Bishop 2000). Marine-craft dynamics and current models are standard (Fossen 2011), and HoloOcean simulates multi-agent sonar and acoustic links (Potokar et al. 2022).

*Open.* Results from simulated acoustic channels transfer to sea only partly; validated loss and multipath models for short-range, multi-vehicle pinging near structures are scarce in the screened records.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Akyildiz, I. F., Pompili, D. and Melodia, T. (2005). Underwater acoustic sensor networks: research challenges. *Ad Hoc Networks*, 3(3), 257-279. | [10.1016/j.adhoc.2005.01.004](https://doi.org/10.1016/j.adhoc.2005.01.004) | yes | Research challenges of underwater acoustic networks: low bandwidth, long and variable delay, high loss. |
| Stojanovic, M. and Preisig, J. (2009). Underwater acoustic communication channels: Propagation models and statistical characterization. *IEEE Communications Magazine*, 47(1), 84-89. | [10.1109/mcom.2009.4752682](https://doi.org/10.1109/mcom.2009.4752682) | yes | Propagation models and statistics of the underwater acoustic channel (multipath, Doppler, fading); explains missed and spurious pings. |
| Stilwell, D. J. and Bishop, B. E. (2000). Platoons of underwater vehicles. *IEEE Control Systems*, 20(6), 45-52. | [10.1109/37.887448](https://doi.org/10.1109/37.887448) | yes | Decentralised platoon control of underwater vehicles with a broadcast-only communication structure; early analysis of minimal communication. |
| Fossen, T. I. (2011). Handbook of Marine Craft Hydrodynamics and Motion Control. *Wiley*. | [10.1002/9781119994138](https://doi.org/10.1002/9781119994138) | yes | Reference for marine-craft dynamics and current modelling used in simulation and controller design. |
| Potokar, E., Ashford, S., Kaess, M. and Mangelson, J. G. (2022). HoloOcean: An Underwater Robotics Simulator. *2022 International Conference on Robotics and Automation (ICRA)*, 3040-3046. | [10.1109/icra46639.2022.9812353](https://doi.org/10.1109/icra46639.2022.9812353) | yes | HoloOcean multi-agent underwater simulator with sonar and acoustic communication models; simulation platform of the system. |

### X Estimation under intermittent observations (cross-cutting)

*Established.* Existence-aware filters handle missed detections and clutter: integrated PDA propagates a probability of target existence (Musicki et al. 1994), the Bernoulli filter is the Bayes-optimal filter for one object that may or may not be present (Ristic et al. 2013) and has been applied to underwater acoustic targets observed with a vector sensor (Gunes and Guldogan 2016). Multiple-hypothesis tracking defers association decisions and uses explicit track confirmation and deletion (Blackman 2004). The Bayesian occupancy filter propagates grid occupancy through short occlusions (Coue et al. 2006). The sequential probability ratio test reaches a decision with bounded error probabilities (Wald 1945).

*Open.* These tools assume calibrated detection and false-alarm models; for wide-cone sonar near structure such models must be identified per sensor and per geometry.

| Reference | DOI | Verified | Relevance to the system |
|---|---|---|---|
| Blackman, S. S. (2004). Multiple hypothesis tracking for multiple target tracking. *IEEE Aerospace and Electronic Systems Magazine*, 19(1), 5-18. | [10.1109/maes.2004.1263228](https://doi.org/10.1109/maes.2004.1263228) | yes | Multiple hypothesis tracking overview, including track confirmation and deletion logic. |
| Musicki, D., Evans, R. and Stankovic, S. (1994). Integrated probabilistic data association. *IEEE Transactions on Automatic Control*, 39(6), 1237-1241. | [10.1109/9.293185](https://doi.org/10.1109/9.293185) | yes | Integrated PDA: recursive probability of target existence with missed detections and clutter; basis for occupancy and neighbour-existence beliefs. |
| Ristic, B., Vo, B., Vo, B. and Farina, A. (2013). A Tutorial on Bernoulli Filters: Theory, Implementation and Applications. *IEEE Transactions on Signal Processing*, 61(13), 3406-3430. | [10.1109/tsp.2013.2257765](https://doi.org/10.1109/tsp.2013.2257765) | yes | Bernoulli filter tutorial: optimal Bayes filter for one object that may or may not be present, with detection probability below one and clutter. |
| Coué, C., Pradalier, C., Laugier, C., Fraichard, T. and Bessière, P. (2006). Bayesian Occupancy Filtering for Multitarget Tracking: An Automotive Application. *The International Journal of Robotics Research*, 25(1), 19-30. | [10.1177/0278364906061158](https://doi.org/10.1177/0278364906061158) | yes | Bayesian occupancy filter: grid occupancy with a prediction step that persists through short occlusions. |
| Wald, A. (1945). Sequential Tests of Statistical Hypotheses. *The Annals of Mathematical Statistics*, 16(2), 117-186. | [10.1214/aoms/1177731118](https://doi.org/10.1214/aoms/1177731118) | yes | Sequential probability ratio test: accumulate evidence until a decision threshold with bounded error probabilities; basis for exit or vacancy decisions. |
| Gunes, A. and Guldogan, M. B. (2016). Joint underwater target detection and tracking with the Bernoulli filter using an acoustic vector sensor. *Digital Signal Processing*, 48, 246-258. | [10.1016/j.dsp.2015.09.020](https://doi.org/10.1016/j.dsp.2015.09.020) | yes | Bernoulli filter for joint detection and tracking of an underwater target with an acoustic vector sensor; underwater use of existence filtering. |

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
@article{leonard2007collective,
  author = {Leonard, Naomi Ehrich and Paley, Derek A. and Lekien, Francois and Sepulchre, Rodolphe and Fratantoni, David M. and Davis, Russ E.},
  title = {{Collective Motion, Sensor Networks, and Ocean Sampling}},
  year = {2007},
  doi = {10.1109/jproc.2006.887295},
  journal = {Proceedings of the IEEE},
  volume = {95},
  number = {1},
  pages = {48--74}
}

@article{yang2021survey,
  author = {Yang, Yue and Xiao, Yang and Li, Tieshan},
  title = {{A Survey of Autonomous Underwater Vehicle Formation: Performance, Formation Control, and Communication Capability}},
  year = {2021},
  doi = {10.1109/comst.2021.3059998},
  journal = {IEEE Communications Surveys & Tutorials},
  volume = {23},
  number = {2},
  pages = {815--841}
}

@article{connor2021current,
  author = {Connor, Jack and Champion, Benjamin and Joordens, Matthew A.},
  title = {{Current Algorithms, Communication Methods and Designs for Underwater Swarm Robotics: A Review}},
  year = {2021},
  doi = {10.1109/jsen.2020.3013265},
  journal = {IEEE Sensors Journal},
  volume = {21},
  number = {1},
  pages = {153--169}
}

@article{berlinger2021implicit,
  author = {Berlinger, Florian and Gauci, Melvin and Nagpal, Radhika},
  title = {{Implicit coordination for 3D underwater collective behaviors in a fish-inspired robot swarm}},
  year = {2021},
  doi = {10.1126/scirobotics.abd8668},
  journal = {Science Robotics},
  volume = {6},
  number = {50},
  pages = {eabd8668}
}

@article{alur1995algorithmic,
  author = {Alur, R. and Courcoubetis, C. and Halbwachs, N. and Henzinger, T.A. and Ho, P.-H. and Nicollin, X. and Olivero, A. and Sifakis, J. and Yovine, S.},
  title = {{The algorithmic analysis of hybrid systems}},
  year = {1995},
  doi = {10.1016/0304-3975(94)00202-t},
  journal = {Theoretical Computer Science},
  volume = {138},
  number = {1},
  pages = {3--34}
}

@article{lygeros2003dynamical,
  author = {Lygeros, J. and Johansson, K.H. and Simic, S.N. and Zhang, Jun and Sastry, S.S.},
  title = {{Dynamical properties of hybrid automata}},
  year = {2003},
  doi = {10.1109/tac.2002.806650},
  journal = {IEEE Transactions on Automatic Control},
  volume = {48},
  number = {1},
  pages = {2--17}
}

@article{tomlin1998conflict,
  author = {Tomlin, C. and Pappas, G.J. and Sastry, S.},
  title = {{Conflict resolution for air traffic management: a study in multiagent hybrid systems}},
  year = {1998},
  doi = {10.1109/9.664154},
  journal = {IEEE Transactions on Automatic Control},
  volume = {43},
  number = {4},
  pages = {509--521}
}

@article{mitsch2017formal,
  author = {Mitsch, Stefan and Ghorbal, Khalil and Vogelbacher, David and Platzer, André},
  title = {{Formal verification of obstacle avoidance and navigation of ground robots}},
  year = {2017},
  doi = {10.1177/0278364917733549},
  journal = {The International Journal of Robotics Research},
  volume = {36},
  number = {12},
  pages = {1312--1340}
}

@article{brooks1986robust,
  author = {Brooks, R.},
  title = {{A robust layered control system for a mobile robot}},
  year = {1986},
  doi = {10.1109/jra.1986.1087032},
  journal = {IEEE Journal on Robotics and Automation},
  volume = {2},
  number = {1},
  pages = {14--23}
}

@article{sha2001using,
  author = {Sha, Lui},
  title = {{Using simplicity to control complexity}},
  year = {2001},
  doi = {10.1109/ms.2001.936213},
  journal = {IEEE Software},
  volume = {18},
  number = {4},
  pages = {20--28}
}

@article{ames2017control,
  author = {Ames, Aaron D. and Xu, Xiangru and Grizzle, Jessy W. and Tabuada, Paulo},
  title = {{Control Barrier Function Based Quadratic Programs for Safety Critical Systems}},
  year = {2017},
  doi = {10.1109/tac.2016.2638961},
  journal = {IEEE Transactions on Automatic Control},
  volume = {62},
  number = {8},
  pages = {3861--3876}
}

@article{wang2017safety,
  author = {Wang, Li and Ames, Aaron D. and Egerstedt, Magnus},
  title = {{Safety Barrier Certificates for Collisions-Free Multirobot Systems}},
  year = {2017},
  doi = {10.1109/tro.2017.2659727},
  journal = {IEEE Transactions on Robotics},
  volume = {33},
  number = {3},
  pages = {661--674}
}

@incollection{vandenberg2011reciprocal,
  author = {van den Berg, Jur and Guy, Stephen J. and Lin, Ming and Manocha, Dinesh},
  title = {{Reciprocal n-Body Collision Avoidance}},
  year = {2011},
  doi = {10.1007/978-3-642-19457-3_1},
  booktitle = {Springer Tracts in Advanced Robotics},
  publisher = {Springer Berlin Heidelberg},
  pages = {3--19}
}

@article{zhou2017fast,
  author = {Zhou, Dingjiang and Wang, Zijian and Bandyopadhyay, Saptarshi and Schwager, Mac},
  title = {{Fast, On-line Collision Avoidance for Dynamic Vehicles Using Buffered Voronoi Cells}},
  year = {2017},
  doi = {10.1109/lra.2017.2656241},
  journal = {IEEE Robotics and Automation Letters},
  volume = {2},
  number = {2},
  pages = {1047--1054}
}

@inproceedings{ozkahraman2020combining,
  author = {Ozkahraman, Ozer and Ogren, Petter},
  title = {{Combining Control Barrier Functions and Behavior Trees for Multi-Agent Underwater Coverage Missions}},
  year = {2020},
  doi = {10.1109/cdc42340.2020.9304151},
  booktitle = {2020 59th IEEE Conference on Decision and Control (CDC)},
  pages = {5275--5282}
}

@article{lamport1974new,
  author = {Lamport, Leslie},
  title = {{A new solution of Dijkstra's concurrent programming problem}},
  year = {1974},
  doi = {10.1145/361082.361093},
  journal = {Communications of the ACM},
  volume = {17},
  number = {8},
  pages = {453--455}
}

@article{dresner2008multiagent,
  author = {Dresner, K. and Stone, P.},
  title = {{A Multiagent Approach to Autonomous Intersection Management}},
  year = {2008},
  doi = {10.1613/jair.2502},
  journal = {Journal of Artificial Intelligence Research},
  volume = {31},
  pages = {591--656}
}

@article{colombo2015least,
  author = {Colombo, Alessandro and Del Vecchio, Domitilla},
  title = {{Least Restrictive Supervisors for Intersection Collision Avoidance: A Scheduling Approach}},
  year = {2015},
  doi = {10.1109/tac.2014.2381453},
  journal = {IEEE Transactions on Automatic Control},
  volume = {60},
  number = {6},
  pages = {1515--1527}
}

@article{wu2015distributed,
  author = {Wu, Weigang and Zhang, Jiebin and Luo, Aoxue and Cao, Jiannong},
  title = {{Distributed Mutual Exclusion Algorithms for Intersection Traffic Control}},
  year = {2015},
  doi = {10.1109/tpds.2013.2297097},
  journal = {IEEE Transactions on Parallel and Distributed Systems},
  volume = {26},
  number = {1},
  pages = {65--74}
}

@article{pecora2018loosely,
  author = {Pecora, Federico and Andreasson, Henrik and Mansouri, Masoumeh and Petkov, Vilian},
  title = {{A Loosely-Coupled Approach for Multi-Robot Coordination, Motion Planning and Control}},
  year = {2018},
  doi = {10.1609/icaps.v28i1.13923},
  journal = {Proceedings of the International Conference on Automated Planning and Scheduling},
  volume = {28},
  pages = {485--493}
}

@article{mannucci2019provably,
  author = {Mannucci, Anna and Pallottino, Lucia and Pecora, Federico},
  title = {{Provably Safe Multi-Robot Coordination With Unreliable Communication}},
  year = {2019},
  doi = {10.1109/lra.2019.2924849},
  journal = {IEEE Robotics and Automation Letters},
  volume = {4},
  number = {4},
  pages = {3232--3239}
}

@inproceedings{gray1989leases,
  author = {Gray, C. and Cheriton, D.},
  title = {{Leases: an efficient fault-tolerant mechanism for distributed file cache consistency}},
  year = {1989},
  doi = {10.1145/74850.74870},
  booktitle = {Proceedings of the twelfth ACM symposium on Operating systems principles},
  pages = {202--210}
}

@article{coffman1971system,
  author = {Coffman, E. G. and Elphick, M. and Shoshani, A.},
  title = {{System Deadlocks}},
  year = {1971},
  doi = {10.1145/356586.356588},
  journal = {ACM Computing Surveys},
  volume = {3},
  number = {2},
  pages = {67--78}
}

@inproceedings{cap2016provably,
  author = {Cap, Michal and Gregoire, Jean and Frazzoli, Emilio},
  title = {{Provably safe and deadlock-free execution of multi-robot plans under delaying disturbances}},
  year = {2016},
  doi = {10.1109/iros.2016.7759750},
  booktitle = {2016 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages = {5113--5118}
}

@article{grover2023before,
  author = {Grover, Jaskaran and Liu, Changliu and Sycara, Katia},
  title = {{The Before, During, and After of Multi-robot Deadlock}},
  year = {2023},
  doi = {10.1177/02783649221074718},
  journal = {The International Journal of Robotics Research},
  volume = {42},
  number = {6},
  pages = {317--336}
}

@article{chandra2025deadlock,
  author = {Chandra, Rohan and Zinage, Vrushabh and Bakolas, Efstathios and Stone, Peter and Biswas, Joydeep},
  title = {{Deadlock-free, safe, and decentralized multi-robot navigation in social mini-games via discrete-time control barrier functions}},
  year = {2025},
  doi = {10.1007/s10514-025-10194-8},
  journal = {Autonomous Robots},
  volume = {49},
  number = {2},
  pages = {12}
}

@article{zheng2024mcca,
  author = {Zheng, Ruochen and Li, Siyu},
  title = {{MCCA: A Decentralized Method for Collision and Deadlock Avoidance With Nonholonomic Robots}},
  year = {2024},
  doi = {10.1109/lra.2024.3358623},
  journal = {IEEE Robotics and Automation Letters},
  volume = {9},
  number = {3},
  pages = {2710--2717}
}

@article{draganjac2016decentralized,
  author = {Draganjac, Ivica and Miklic, Damjan and Kovacic, Zdenko and Vasiljevic, Goran and Bogdan, Stjepan},
  title = {{Decentralized Control of Multi-AGV Systems in Autonomous Warehousing Applications}},
  year = {2016},
  doi = {10.1109/tase.2016.2603781},
  journal = {IEEE Transactions on Automation Science and Engineering},
  volume = {13},
  number = {4},
  pages = {1433--1447}
}

@inproceedings{moravec1985high,
  author = {Moravec, H. and Elfes, A.},
  title = {{High resolution maps from wide angle sonar}},
  year = {1985},
  doi = {10.1109/robot.1985.1087316},
  booktitle = {Proceedings. 1985 IEEE International Conference on Robotics and Automation},
  pages = {116--121}
}

@incollection{kleeman2016sonar,
  author = {Kleeman, Lindsay and Kuc, Roman},
  title = {{Sonar Sensing}},
  year = {2016},
  doi = {10.1007/978-3-319-32552-1_30},
  booktitle = {Springer Handbooks},
  publisher = {Springer International Publishing},
  pages = {753--782}
}

@article{petillot2001underwater,
  author = {Petillot, Y. and Ruiz, I.T. and Lane, D.M.},
  title = {{Underwater vehicle obstacle avoidance and path planning using a multi-beam forward looking sonar}},
  year = {2001},
  doi = {10.1109/48.922790},
  journal = {IEEE Journal of Oceanic Engineering},
  volume = {26},
  number = {2},
  pages = {240--251}
}

@article{ribas2008underwater,
  author = {Ribas, David and Ridao, Pere and Tardós, Juan Domingo and Neira, José},
  title = {{Underwater SLAM in man-made structured environments}},
  year = {2008},
  doi = {10.1002/rob.20249},
  journal = {Journal of Field Robotics},
  volume = {25},
  number = {11-12},
  pages = {898--921}
}

@article{batista2011single,
  author = {Batista, Pedro and Silvestre, Carlos and Oliveira, Paulo},
  title = {{Single range aided navigation and source localization: Observability and filter design}},
  year = {2011},
  doi = {10.1016/j.sysconle.2011.05.004},
  journal = {Systems & Control Letters},
  volume = {60},
  number = {8},
  pages = {665--673}
}

@article{paull2014auv,
  author = {Paull, Liam and Saeedi, Sajad and Seto, Mae and Li, Howard},
  title = {{AUV Navigation and Localization: A Review}},
  year = {2014},
  doi = {10.1109/joe.2013.2278891},
  journal = {IEEE Journal of Oceanic Engineering},
  volume = {39},
  number = {1},
  pages = {131--149}
}

@article{eustice2011synchronous,
  author = {Eustice, Ryan M. and Singh, Hanumant and Whitcomb, Louis L.},
  title = {{Synchronous-clock, one-way-travel-time acoustic navigation for underwater vehicles}},
  year = {2011},
  doi = {10.1002/rob.20365},
  journal = {Journal of Field Robotics},
  volume = {28},
  number = {1},
  pages = {121--136}
}

@article{webster2012advances,
  author = {Webster, Sarah E and Eustice, Ryan M and Singh, Hanumant and Whitcomb, Louis L},
  title = {{Advances in single-beacon one-way-travel-time acoustic navigation for underwater vehicles}},
  year = {2012},
  doi = {10.1177/0278364912446166},
  journal = {The International Journal of Robotics Research},
  volume = {31},
  number = {8},
  pages = {935--950}
}

@article{bahr2009cooperative,
  author = {Bahr, Alexander and Leonard, John J. and Fallon, Maurice F.},
  title = {{Cooperative Localization for Autonomous Underwater Vehicles}},
  year = {2009},
  doi = {10.1177/0278364908100561},
  journal = {The International Journal of Robotics Research},
  volume = {28},
  number = {6},
  pages = {714--728}
}

@inproceedings{rypkema2017oneway,
  author = {Rypkema, Nicholas R. and Fischell, Erin M. and Schmidt, Henrik},
  title = {{One-way travel-time inverted ultra-short baseline localization for low-cost autonomous underwater vehicles}},
  year = {2017},
  doi = {10.1109/icra.2017.7989570},
  booktitle = {2017 IEEE International Conference on Robotics and Automation (ICRA)},
  pages = {4920--4926}
}

@inproceedings{rypkema2019passive,
  author = {Rypkema, Nicholas R. and Schmidt, Henrik},
  title = {{Passive Inverted Ultra-Short Baseline (piUSBL) Localization: An Experimental Evaluation of Accuracy}},
  year = {2019},
  doi = {10.1109/iros40897.2019.8967800},
  booktitle = {2019 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages = {7197--7204}
}

@article{rypkema2022synchronous,
  author = {Rypkema, Nicholas R. and Schmidt, Henrik and Fischell, Erin M.},
  title = {{Synchronous-Clock Range-Angle Relative Acoustic Navigation: A Unified Approach to Multi-AUV Localization, Command, Control, and Coordination}},
  year = {2022},
  doi = {10.55417/fr.2022026},
  journal = {Field Robotics},
  volume = {2},
  pages = {774--806}
}

@article{fischell2020single,
  author = {Fischell, Erin Marie and Kroo, Anne R. and O'Neill, Brendan W.},
  title = {{Single-Hydrophone Low-Cost Underwater Vehicle Swarming}},
  year = {2020},
  doi = {10.1109/lra.2019.2958774},
  journal = {IEEE Robotics and Automation Letters},
  volume = {5},
  number = {2},
  pages = {354--361}
}

@inproceedings{syed2006time,
  author = {Syed, A. A. and Heidemann, J.},
  title = {{Time Synchronization for High Latency Acoustic Networks}},
  year = {2006},
  doi = {10.1109/infocom.2006.161},
  booktitle = {Proceedings IEEE INFOCOM 2006. 25TH IEEE International Conference on Computer Communications},
  pages = {1--12}
}

@article{ghabcheloo2009coordinated,
  author = {Ghabcheloo, R. and Aguiar, A. P. and Pascoal, A. and Silvestre, C. and Kaminer, I. and Hespanha, J.},
  title = {{Coordinated Path-Following in the Presence of Communication Losses and Time Delays}},
  year = {2009},
  doi = {10.1137/060678993},
  journal = {SIAM Journal on Control and Optimization},
  volume = {48},
  number = {1},
  pages = {234--265}
}

@article{lewis1997high,
  author = {Lewis, M. Anthony and Tan, Kar-Han},
  title = {{High Precision Formation Control of Mobile Robots Using Virtual Structures}},
  year = {1997},
  doi = {10.1023/a:1008814708459},
  journal = {Autonomous Robots},
  volume = {4},
  number = {4},
  pages = {387--403}
}

@article{oh2015survey,
  author = {Oh, Kwang-Kyo and Park, Myoung-Chul and Ahn, Hyo-Sung},
  title = {{A survey of multi-agent formation control}},
  year = {2015},
  doi = {10.1016/j.automatica.2014.10.022},
  journal = {Automatica},
  volume = {53},
  pages = {424--440}
}

@inproceedings{soares2013joint,
  author = {Soares, Jorge M. and Aguiar, A. Pedro and Pascoal, Antonio M. and Martinoli, Alcherio},
  title = {{Joint ASV/AUV range-based formation control: Theory and experimental results}},
  year = {2013},
  doi = {10.1109/icra.2013.6631378},
  booktitle = {2013 IEEE International Conference on Robotics and Automation},
  pages = {5579--5585}
}

@article{mirollo1990synchronization,
  author = {Mirollo, Renato E. and Strogatz, Steven H.},
  title = {{Synchronization of Pulse-Coupled Biological Oscillators}},
  year = {1990},
  doi = {10.1137/0150098},
  journal = {SIAM Journal on Applied Mathematics},
  volume = {50},
  number = {6},
  pages = {1645--1662}
}

@article{christensen2009fireflies,
  author = {Christensen, A.L. and O'Grady, R. and Dorigo, M.},
  title = {{From Fireflies to Fault-Tolerant Swarms of Robots}},
  year = {2009},
  doi = {10.1109/tevc.2009.2017516},
  journal = {IEEE Transactions on Evolutionary Computation},
  volume = {13},
  number = {4},
  pages = {754--766}
}

@article{chandra1996unreliable,
  author = {Chandra, Tushar Deepak and Toueg, Sam},
  title = {{Unreliable failure detectors for reliable distributed systems}},
  year = {1996},
  doi = {10.1145/226643.226647},
  journal = {Journal of the ACM},
  volume = {43},
  number = {2},
  pages = {225--267}
}

@inproceedings{hayashibara2004phi,
  author = {Hayashibara, N. and Defago, X. and Yared, R. and Katayama, T.},
  title = {{The $\varphi$ accrual failure detector}},
  year = {2004},
  doi = {10.1109/reldis.2004.1353004},
  booktitle = {Proceedings of the 23rd IEEE International Symposium on Reliable Distributed Systems, 2004.},
  pages = {66--78}
}

@article{pang2024multi,
  author = {Pang, Wen and Zhu, Daqi and Sun, Changyin},
  title = {{Multi-AUV Formation Reconfiguration Obstacle Avoidance Algorithm Based on Affine Transformation and Improved Artificial Potential Field Under Ocean Currents Disturbance}},
  year = {2024},
  doi = {10.1109/tase.2023.3245818},
  journal = {IEEE Transactions on Automation Science and Engineering},
  volume = {21},
  number = {2},
  pages = {1469--1487}
}

@article{akyildiz2005underwater,
  author = {Akyildiz, Ian F. and Pompili, Dario and Melodia, Tommaso},
  title = {{Underwater acoustic sensor networks: research challenges}},
  year = {2005},
  doi = {10.1016/j.adhoc.2005.01.004},
  journal = {Ad Hoc Networks},
  volume = {3},
  number = {3},
  pages = {257--279}
}

@article{stojanovic2009underwater,
  author = {Stojanovic, Milica and Preisig, James},
  title = {{Underwater acoustic communication channels: Propagation models and statistical characterization}},
  year = {2009},
  doi = {10.1109/mcom.2009.4752682},
  journal = {IEEE Communications Magazine},
  volume = {47},
  number = {1},
  pages = {84--89}
}

@article{stilwell2000platoons,
  author = {Stilwell, Daniel J. and Bishop, Bradley E.},
  title = {{Platoons of underwater vehicles}},
  year = {2000},
  doi = {10.1109/37.887448},
  journal = {IEEE Control Systems},
  volume = {20},
  number = {6},
  pages = {45--52}
}

@book{fossen2011handbook,
  author = {Fossen, Thor I.},
  title = {{Handbook of Marine Craft Hydrodynamics and Motion Control}},
  year = {2011},
  doi = {10.1002/9781119994138},
  publisher = {Wiley}
}

@inproceedings{potokar2022holoocean,
  author = {Potokar, Easton and Ashford, Spencer and Kaess, Michael and Mangelson, Joshua G.},
  title = {{HoloOcean: An Underwater Robotics Simulator}},
  year = {2022},
  doi = {10.1109/icra46639.2022.9812353},
  booktitle = {2022 International Conference on Robotics and Automation (ICRA)},
  pages = {3040--3046}
}

@article{blackman2004multiple,
  author = {Blackman, S.S.},
  title = {{Multiple hypothesis tracking for multiple target tracking}},
  year = {2004},
  doi = {10.1109/maes.2004.1263228},
  journal = {IEEE Aerospace and Electronic Systems Magazine},
  volume = {19},
  number = {1},
  pages = {5--18}
}

@article{musicki1994integrated,
  author = {Musicki, D. and Evans, R. and Stankovic, S.},
  title = {{Integrated probabilistic data association}},
  year = {1994},
  doi = {10.1109/9.293185},
  journal = {IEEE Transactions on Automatic Control},
  volume = {39},
  number = {6},
  pages = {1237--1241}
}

@article{ristic2013tutorial,
  author = {Ristic, Branko and Vo, Ba-Tuong and Vo, Ba-Ngu and Farina, Alfonso},
  title = {{A Tutorial on Bernoulli Filters: Theory, Implementation and Applications}},
  year = {2013},
  doi = {10.1109/tsp.2013.2257765},
  journal = {IEEE Transactions on Signal Processing},
  volume = {61},
  number = {13},
  pages = {3406--3430}
}

@article{coue2006bayesian,
  author = {Coué, Christophe and Pradalier, Cédric and Laugier, Christian and Fraichard, Thierry and Bessière, Pierre},
  title = {{Bayesian Occupancy Filtering for Multitarget Tracking: An Automotive Application}},
  year = {2006},
  doi = {10.1177/0278364906061158},
  journal = {The International Journal of Robotics Research},
  volume = {25},
  number = {1},
  pages = {19--30}
}

@article{wald1945sequential,
  author = {Wald, A.},
  title = {{Sequential Tests of Statistical Hypotheses}},
  year = {1945},
  doi = {10.1214/aoms/1177731118},
  journal = {The Annals of Mathematical Statistics},
  volume = {16},
  number = {2},
  pages = {117--186}
}

@article{gunes2016joint,
  author = {Gunes, Ahmet and Guldogan, Mehmet B.},
  title = {{Joint underwater target detection and tracking with the Bernoulli filter using an acoustic vector sensor}},
  year = {2016},
  doi = {10.1016/j.dsp.2015.09.020},
  journal = {Digital Signal Processing},
  volume = {48},
  pages = {246--258}
}
```

---

## Appendix A. Per-query records by source

| Tag | Search string | OpenAlex | Crossref | Semantic Scholar | Unique records |
|---|---|---|---|---|---|
| T1q1 | decentralized multi-AUV coordination | 20 | 10 | 0 | 26 |
| T1q2 | underwater swarm robotics implicit coordination without communication | 2 | 10 | 0 | 11 |
| T1q3 | multiple autonomous underwater vehicles cooperative control survey | 20 | 10 | 0 | 29 |
| T2q1 | hybrid automata formal verification robots | 20 | 10 | 0 | 29 |
| T2q2 | multi-agent hybrid systems conflict resolution safety verification | 1 | 10 | 0 | 11 |
| T3q1 | control barrier functions multi-robot collision avoidance | 20 | 10 | 0 | 28 |
| T3q2 | collision avoidance multiple autonomous underwater vehicles | 20 | 10 | 0 | 29 |
| T3q3 | decentralized collision avoidance without communication robots | 20 | 10 | 0 | 30 |
| T4q1 | mutual exclusion intersection autonomous vehicles distributed | 7 | 10 | 0 | 14 |
| T4q2 | multi-robot coordination narrow passage critical section | 0 | 10 | 0 | 10 |
| T4q3 | decentralized intersection crossing robots without communication | 0 | 10 | 0 | 10 |
| T5q1 | deadlock resolution multi-robot systems decentralized | 11 | 10 | 20 | 31 |
| T5q2 | provably safe deadlock-free multi-robot coordination | 0 | 10 | 20 | 24 |
| T6q1 | wide-angle sonar occupancy grid uncertainty | 0 | 10 | 0 | 10 |
| T6q2 | single beam sonar obstacle detection underwater vehicle | 7 | 10 | 0 | 17 |
| T6q3 | observability range-only bearing-only underwater navigation | 4 | 10 | 0 | 14 |
| T7q1 | one-way travel time acoustic navigation synchronous clock | 9 | 10 | 0 | 12 |
| T7q2 | inverted ultra-short baseline localization AUV | 5 | 10 | 0 | 11 |
| T7q3 | cooperative localization autonomous underwater vehicles acoustic ranging | 20 | 10 | 0 | 27 |
| T8q1 | acoustic beacon passive localization multiple AUVs swarm | 0 | 10 | 0 | 10 |
| T8q2 | relative localization underwater robots acoustic broadcast neighbors | 0 | 10 | 0 | 10 |
| T9q1 | coordinated path following marine vehicles communication losses | 0 | 10 | 0 | 10 |
| T9q2 | formation control autonomous underwater vehicles ocean currents | 20 | 10 | 0 | 27 |
| T9q3 | synchronization path parameter formation without communication | 9 | 10 | 0 | 19 |
| T10q1 | fault-tolerant formation control multi-agent agent loss reconfiguration | 1 | 10 | 0 | 11 |
| T10q2 | robot swarm fault detection heartbeat synchronization | 0 | 10 | 0 | 10 |
| T10q3 | failure detector timeout distributed systems accrual | 2 | 10 | 0 | 12 |
| T11q1 | underwater acoustic communication channel challenges networking | 20 | 10 | 0 | 30 |
| T11q2 | underwater robotics simulator sonar acoustic | 17 | 10 | 0 | 25 |
| T11q3 | AUV navigation localization review | 20 | 10 | 0 | 29 |
| Xq1 | target existence probability tracking intermittent detection Bernoulli filter | 0 | 10 | 0 | 10 |
| Xq2 | Bayesian occupancy filter dynamic environment tracking | 20 | 10 | 0 | 25 |
| Xq3 | multiple hypothesis tracking track confirmation | 20 | 10 | 0 | 28 |
| Xq4 | pulse-coupled oscillators synchronization | 20 | 10 | 0 | 28 |
| T4sq1 | narrow passage multi-robot | 20 | 10 | 0 | 28 |
| T4sq2 | intersection management autonomous vehicles survey | 20 | 10 | 0 | 30 |
| T4sq3 | AGV zone control deadlock | 20 | 10 | 0 | 25 |
| T6sq1 | range-only observability underwater | 20 | 10 | 0 | 26 |
| T8sq1 | passive inverted USBL | 7 | 10 | 0 | 14 |
| T8sq2 | low-cost acoustic modem localization micro AUV | 5 | 10 | 0 | 11 |
| T8sq3 | single beacon swarm AUV acoustic | 1 | 10 | 0 | 10 |
| T9sq1 | range-based formation control marine vehicles | 20 | 10 | 0 | 27 |
| T9sq2 | cooperative path following event-triggered communication marine | 4 | 10 | 0 | 14 |
| T10sq1 | formation reconfiguration agent failure | 20 | 10 | 0 | 28 |
| T10sq2 | graceful degradation multi-robot formation | 2 | 10 | 0 | 11 |
| T11sq1 | BlueROV2 multi-vehicle | 15 | 10 | 0 | 24 |

## Appendix B. Works excluded at eligibility (57)

| Work | DOI (verified on Crossref) | Reason |
|---|---|---|
| Morgado et al. (2013). Tightly coupled ultrashort baseline and inertial navigation system for underwater vehicles: An experimental validation | 10.1002/rob.21442 | R1 |
| Grover et al. (2021). Deadlock Analysis and Resolution for Multi-robot Systems | 10.1007/978-3-030-66723-8_18 | R2 |
| Aguilera et al. (1997). Heartbeat: A timeout-free failure detector for quiescent reliable communication | 10.1007/bfb0030680 | R1 |
| Platzer (2008). Differential Dynamic Logic for Hybrid Systems | 10.1007/s10817-008-9103-8 | R1 |
| Ihle et al. (2007). Passivity-based designs for synchronized path-following | 10.1016/j.automatica.2007.02.018 | R1 |
| Borrmann et al. (2015). Control Barrier Certificates for Safe Swarm Behavior | 10.1016/j.ifacol.2015.11.154 | R1 |
| Leucker and Schallhart (2009). A brief account of runtime verification | 10.1016/j.jlap.2008.08.004 | R1 |
| Cui et al. (2010). Leader-follower formation control of underactuated autonomous underwater vehicles | 10.1016/j.oceaneng.2010.07.006 | R3 |
| Tan et al. (2011). A survey of techniques and challenges in underwater localization | 10.1016/j.oceaneng.2011.07.017 | R1 |
| Lynch et al. (2003). Hybrid I/O automata | 10.1016/s0890-5401(03)00067-1 | R1 |
| Fanti (2002). Event-based controller to avoid deadlock and collisions in zone-control AGVS | 10.1080/00207540110118073 | R3 |
| Elfes (1989). Using occupancy grids for mobile robot perception and navigation | 10.1109/2.30720 | R1 |
| Taek Lyul Song (1999). Observability of target tracking with range-only measurements | 10.1109/48.775299 | R1 |
| Balch and Arkin (1998). Behavior-based formation control for multirobot teams | 10.1109/70.736776 | R1 |
| Leonard and Durrant-Whyte (1991). Mobile robot localization by tracking geometric beacons | 10.1109/70.88147 | R1 |
| Beard et al. (2001). A coordination architecture for spacecraft formation control | 10.1109/87.960341 | R4 |
| Cook et al. (2014). A survey of AUV and robot simulators for multi-vehicle operations | 10.1109/auv.2014.7054411 | R4 |
| Haraldsen et al. (2024). Safety-Critical Guidance of Underactuated Marine Vehicles in Dynamic Environments using Velocity Obstacles * | 10.1109/ccta60707.2024.10666518 | R4 |
| Jager and Nebel (2001). Decentralized collision avoidance, deadlock detection, and deadlock resolution for multiple mobile robots | 10.1109/iros.2001.977148 | R1 |
| Gadre and Stilwell (2005). A complete solution to underwater navigation in the presence of unknown currents based on range measurements from a single location | 10.1109/iros.2005.1545230 | R1 |
| Fiorelli et al. (2006). Multi-AUV Control and Adaptive Sampling in Monterey Bay | 10.1109/joe.2006.880429 | R1 |
| Kuwata et al. (2014). Safe Maritime Autonomous Navigation With COLREGS, Using Velocity Obstacles | 10.1109/joe.2013.2254214 | R4 |
| Olfati-Saber et al. (2007). Consensus and Cooperation in Networked Multi-Agent Systems | 10.1109/jproc.2006.887293 | R3 |
| Henzinger (1996). The theory of hybrid automata | 10.1109/lics.1996.561342 | R1 |
| Cecchi et al. (2021). Priority-Based Distributed Coordination for Heterogeneous Multi-Robot Systems With Realistic Assumptions | 10.1109/lra.2021.3091016 | R3 |
| Anderson et al. (2008). Rigid graph control architectures for autonomous formations | 10.1109/mcs.2008.929280 | R4 |
| Bar-Shalom et al. (2009). The probabilistic data association filter | 10.1109/mcs.2009.934469 | R1 |
| Reid (1979). An algorithm for tracking multiple targets | 10.1109/tac.1979.1102177 | R1 |
| Ogren et al. (2004). Cooperative Control of Mobile Sensor Networks: Adaptive Gradient Climbing in a Distributed Environment | 10.1109/tac.2004.832203 | R4 |
| Olfati-Saber (2006). Flocking for Multi-Agent Dynamic Systems: Algorithms and Theory | 10.1109/tac.2005.864190 | R4 |
| Zhao and Zelazo (2016). Bearing Rigidity and Almost Global Bearing-Only Formation Stabilization | 10.1109/tac.2015.2459191 | R4 |
| Nardone and Aidala (1981). Observability Criteria for Bearings-Only Target Motion Analysis | 10.1109/taes.1981.309141 | R1 |
| Mahler (2003). Multitarget bayes filtering via first-order multitarget moments | 10.1109/taes.2003.1261119 | R4 |
| Cap et al. (2015). Prioritized Planning Algorithms for Trajectory Coordination of Multiple Mobile Robots | 10.1109/tase.2015.2445780 | R3 |
| Børhaug et al. (2011). Straight Line Path Following for Formations of Underactuated Marine Surface Vessels | 10.1109/tcst.2010.2050889 | R4 |
| Bayat et al. (2016). Range-Based Underwater Vehicle Localization in the Presence of Unknown Ocean Currents: Theory and Experiments | 10.1109/tcst.2015.2420636 | R1 |
| Hafner et al. (2013). Cooperative Collision Avoidance at Intersections: Algorithms and Experiments | 10.1109/tits.2013.2252901 | R3 |
| Chen and Englund (2016). Cooperative Intersection Management: A Survey | 10.1109/tits.2015.2471812 | R3 |
| Das et al. (2002). A vision-based formation control framework | 10.1109/tra.2002.803463 | R4 |
| Kress-Gazit et al. (2009). Temporal-Logic-Based Reactive Mission and Motion Planning | 10.1109/tro.2009.2030225 | R4 |
| Webster et al. (2013). Decentralized Extended Information Filter for Single-Beacon Cooperative Acoustic Navigation: Theory and Experiments | 10.1109/tro.2013.2252857 | R3 |
| Mannucci et al. (2021). On Provably Safe and Live Multirobot Coordination With Online Goal Posting | 10.1109/tro.2021.3075371 | R3 |
| Werner-Allen et al. (2005). Firefly-inspired sensor network synchronicity with realistic radio effects | 10.1145/1098918.1098934 | R1 |
| Partan et al. (2007). A survey of practical issues in underwater networks | 10.1145/1347364.1347372 | R1 |
| Luckcuck et al. (2020). Formal Specification and Verification of Autonomous Robotic Systems | 10.1145/3342355 | R4 |
| Ricart and Agrawala (1981). An optimal algorithm for mutual exclusion in computer networks | 10.1145/358527.358537 | R3 |
| Lamport (1978). Time, clocks, and the ordering of events in a distributed system | 10.1145/359545.359563 | R3 |
| Dijkstra (1974). Self-stabilizing systems in spite of distributed control | 10.1145/361179.361202 | R4 |
| Dijkstra (1965). Solution of a problem in concurrent programming control | 10.1145/365559.365617 | R1 |
| Reynolds (1987). Flocks, herds and schools: A distributed behavioral model | 10.1145/37402.37406 | R4 |
| Fallon et al. (2010). Cooperative AUV Navigation using a Single Maneuvering Surface Craft | 10.1177/0278364910380760 | R3 |
| Khatib (1986). Real-Time Obstacle Avoidance for Manipulators and Mobile Robots | 10.1177/027836498600500106 | R4 |
| Fiorini and Shiller (1998). Motion Planning in Dynamic Environments Using Velocity Obstacles | 10.1177/027836499801700706 | R1 |
| Winfield and Nembrini (2006). Safety in numbers: fault-tolerance in robot swarms | 10.1504/ijmic.2006.008645 | R1 |
| Ames et al. (2019). Control Barrier Functions: Theory and Applications | 10.23919/ecc.2019.8796030 | R1 |
| González-García et al. (2020). Autonomous Underwater Vehicles: Localization, Navigation, and Communication for Collaborative Missions | 10.3390/app10041256 | R1 |
| Chitre et al. (2008). Underwater Acoustic Communications and Networking: Recent Advances and Future Challenges | 10.4031/002533208786861263 | R1 |
