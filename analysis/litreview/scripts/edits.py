import sys
p, f = sys.argv[1], sys.argv[2]
rep_t = [
 ("Crossref lacks author metadata for two included items (Stilwell and Bishop 2000; and, among excluded works, Bar-Shalom et al. 2009 and Anderson et al. 2008); their authors were taken from the OpenAlex record of the same DOI.",
  "Crossref lacks author metadata for one included item (Stilwell and Bishop 2000) and for two excluded works (Bar-Shalom et al. 2009; Anderson et al. 2008); their authors were taken from the OpenAlex record of the same DOI."),
 ("Glider fleets were coordinated for adaptive ocean sampling with information shared at the surface (Leonard et al. 2007)",
  "Glider fleets were coordinated for adaptive ocean sampling with information exchanged when the gliders surfaced (Leonard et al. 2007)"),
 ("with conditions for well-posed executions and for excluding Zeno behaviour (Lygeros et al. 2003)",
  "with conditions for existence and uniqueness of executions and an analysis of Zeno behaviour (Lygeros et al. 2003)"),
 ("Reactive barrier controllers can also stall in deadlock (Wang et al. 2017; Grover et al. 2023), which matters in front of a gate.",
  "Reactive barrier controllers can also stall in deadlock (Grover et al. 2023), which matters in front of a gate."),
 ("and sonar-based mapping in man-made structured environments supports localisation against known structure (Ribas et al. 2008). For relative localisation, range-only information is observable only under sufficiently exciting relative motion (Batista et al. 2011).",
  "and sonar-based SLAM in man-made structured environments localises the vehicle against structural features (Ribas et al. 2008). For relative localisation, observability of single-range navigation depends on the trajectory, which must be sufficiently rich (Batista et al. 2011)."),
 ("Failure-detector theory makes the trade-off between detection time and false suspicion explicit (Chandra and Toueg 1996)",
  "Failure-detector theory separates completeness (every crashed process is eventually suspected) from accuracy (correct processes are not wrongly suspected) and shows which combinations suffice for agreement (Chandra and Toueg 1996)"),
 ("and has been applied to underwater acoustic targets at low SNR (Gunes and Guldogan 2016)",
  "and has been applied to underwater acoustic targets observed with a vector sensor (Gunes and Guldogan 2016)"),
 ("Already the system's sensing model; the TDMA slot gives both the emission time and the identity, as scheduled broadcasts do in the sources. Use the published piUSBL error figures to set measurement noise.",
  "Already the system's sensing model. The TDMA slot fixes both the emission time and the identity, so no data payload is needed; in single-beacon OWTT navigation the broadcast also carries data such as the beacon position (Webster et al. 2012). Use the published piUSBL error figures to set measurement noise."),
 ("**Chandra et al. (2025); Zheng and Li (2024); Grover et al. (2023), decentralised doorway and deadlock handling.** *Similar:* non-communicative, decentralised passage of narrow spaces with explicit deadlock treatment. *Different:* continuous control (CBF, MPC) of ground robots with good relative perception; no explicit occupancy belief of the passage and no exclusivity rule.",
  "**Chandra et al. (2025); Zheng and Li (2024); Grover et al. (2023), decentralised passage and deadlock handling.** *Similar:* decentralised passage of doorways, corridors and narrow spaces with explicit deadlock treatment; Chandra et al. state a non-communicative, partially observable setting, and Grover et al. analyse when barrier-based controllers deadlock. *Different:* continuous control (discrete-time CBFs within receding-horizon control, or velocity-level rules) of ground robots; no explicit occupancy belief of the passage and no one-at-a-time exclusivity rule."),
 ("`references.bib` contains no entry for topics T2-T5, T9, T10 or X,", "Judging by titles, `references.bib` contains no entry for topics T2-T5, T9, T10 or X,"),
 ("| Waiting zones outside the approach and exit corridors (Draganjac et al. 2016) | Pre-allocate holding positions so yielding vehicles never block the exit path of the occupant.",
  "| Waiting zones outside the approach and exit corridors (Draganjac et al. 2016) | Pre-allocate holding positions, in the spirit of the private zones used for lower-priority removal actions, so that yielding vehicles never block the exit path of the occupant."),
]
s = open(p, encoding='utf-8').read()
for a, b in rep_t:
    assert a in s, a[:60]
    s = s.replace(a, b)
open(p, 'w', encoding='utf-8').write(s)
rep_f = [
 ("'Decentralised safety barrier certificates for multi-robot collision avoidance; reports that reactive barrier controllers can stall in deadlock.'",
  "'Decentralised safety barrier certificates for multi-robot collision avoidance from neighbour states; basis of barrier-based separation layers.'"),
 ("'Sonar-based localisation and mapping in man-made structured underwater environments; precedent for map-aided relocalisation against known structures.'",
  "'Sonar-based SLAM in man-made structured underwater environments using structural features; precedent for localisation against known structure.'"),
 ("'Theory of unreliable failure detectors (completeness vs accuracy); limits of declaring a neighbour missing from timeouts.'",
  "'Theory of unreliable failure detectors (completeness and accuracy properties); formal vocabulary for declaring a neighbour missing.'"),
 ("'Bernoulli filter for joint detection and tracking of an underwater acoustic target at low SNR; underwater evidence for existence filtering.'",
  "'Bernoulli filter for joint detection and tracking of an underwater target with an acoustic vector sensor; underwater use of existence filtering.'"),
 ("'Well-posedness, Zeno behaviour and existence of executions of hybrid automata; relevant to guard design that avoids chattering between priority modes.'",
  "'Existence, uniqueness and Zeno behaviour of hybrid-automaton executions; relevant to guard design that avoids chattering between priority modes.'"),
]
s = open(f, encoding='utf-8').read()
for a, b in rep_f:
    assert a in s, a[:60]
    s = s.replace(a, b)
open(f, 'w', encoding='utf-8').write(s)
print('ok')
