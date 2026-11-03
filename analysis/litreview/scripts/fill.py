"""Fill the review template with tables generated from verified Crossref records."""
import json, os, re, sys, glob, html

D = sys.argv[1]
REL = {
 'leonard2007collective': 'Glider-fleet coordination for ocean sampling; reference point for underwater collective motion where vehicles share information only intermittently.',
 'yang2021survey': 'Survey of AUV formation control that ties formation performance to acoustic communication capability; frames the communication-limited setting.',
 'connor2021current': 'Review of underwater swarm algorithms, communication methods and vehicle designs; maps the communication options open to small vehicles.',
 'berlinger2021implicit': 'Blueswarm: 3D underwater collective behaviours from local implicit (vision-based) coordination with no message exchange; closest precedent for implicit coordination underwater.',
 'alur1995algorithmic': 'Foundational hybrid-automaton semantics and reachability analysis; basis for modelling the per-vehicle mode logic and verifying safety properties.',
 'lygeros2003dynamical': 'Existence, uniqueness and Zeno behaviour of hybrid-automaton executions; relevant to guard design that avoids chattering between priority modes.',
 'tomlin1998conflict': 'Multi-agent hybrid-system conflict resolution with verified manoeuvres; methodological template for a verified gate/yield protocol.',
 'mitsch2017formal': 'Formally verified (differential dynamic logic) obstacle avoidance with sensor uncertainty and moving obstacles; template for verifying collision avoidance under imperfect sensing.',
 'brooks1986robust': 'Layered priority (subsumption) control; ancestor of the fixed priority order failsafe > avoidance > ... > formation keeping.',
 'sha2001using': 'Simplex architecture: a simple verified safety controller supervises a complex controller; pattern for failsafe and watchdog layers.',
 'ames2017control': 'CBF-based quadratic programs that enforce forward invariance of a safe set; standard tool for the collision-avoidance and separation layers.',
 'wang2017safety': 'Decentralised safety barrier certificates for multi-robot collision avoidance from neighbour states; basis of barrier-based separation layers.',
 'vandenberg2011reciprocal': 'ORCA: reciprocal collision avoidance where each agent takes half the responsibility using only observed neighbour states.',
 'zhou2017fast': 'Buffered Voronoi cells: collision avoidance that needs only sensed neighbour positions, with no communication.',
 'ozkahraman2020combining': 'Combines CBFs with behaviour trees for multi-AUV underwater missions; close analogue of a priority-structured mode logic with barrier constraints.',
 'lamport1974new': 'Bakery mutual exclusion: order by ticket with ties broken by process identifier; template for precedence from local queue order plus ID tie-break.',
 'dresner2008multiagent': 'Reservation-based autonomous intersection management (central manager, vehicle requests); reference design for space-time reservation of a shared region.',
 'colombo2015least': 'Least-restrictive supervisor for intersection collision avoidance via scheduling; formal basis for intervening only when entry would become unsafe.',
 'wu2015distributed': 'Maps intersection crossing to distributed mutual exclusion over V2V messages; shows the protocol view of a critical region (requires messages).',
 'pecora2018loosely': 'Critical sections on overlapping paths with online precedence constraints; robots stop before a critical section unless granted (central coordinator).',
 'mannucci2019provably': 'Extends critical-section coordination to delayed/lossy communication with safety proofs; closest formal treatment of CR safety under degraded information.',
 'gray1989leases': 'Leases: time-bounded grants that remain safe when the holder becomes unreachable; formal analogue of the occupancy timeout fallback.',
 'coffman1971system': 'Necessary conditions for deadlock (mutual exclusion, hold-and-wait, no preemption, circular wait) and prevention/avoidance/detection strategies.',
 'cap2016provably': 'Safe and deadlock-free execution of coordinated plans when robots are delayed; precedence-respecting waiting rule with proofs.',
 'grover2023before': 'Characterises when CBF-controlled multi-robot systems deadlock and how to leave deadlock; directly relevant to avoidance layers stalling at a gate.',
 'chandra2025deadlock': 'Non-communicative, decentralised doorway and intersection navigation with deadlock-free guarantees via discrete-time CBFs; closest decentralised narrow-passage method.',
 'zheng2024mcca': 'Fully decentralised collision and deadlock avoidance in dense, narrow environments using decentralised priorities and masked velocities.',
 'draganjac2016decentralized': 'Decentralised multi-AGV coordination with priority-based conflict resolution and private zones; analysed for deadlock and livelock freedom.',
 'moravec1985high': 'Probabilistic occupancy maps from wide-angle sonar; the original treatment of the angular ambiguity of wide-cone single-beam returns.',
 'kleeman2016sonar': 'Handbook chapter on sonar physics (beam patterns, specular reflection, multiple echoes, crosstalk) that bounds what single-beam sonar can observe.',
 'petillot2001underwater': 'AUV obstacle detection, tracking and avoidance from forward-looking sonar; underwater sonar-only perception for collision avoidance.',
 'ribas2008underwater': 'Sonar-based SLAM in man-made structured underwater environments using structural features; precedent for localisation against known structure.',
 'batista2011single': 'Observability analysis and filter design for single-range navigation; formalises when range-only relative information is sufficient.',
 'paull2014auv': 'Review of AUV navigation (inertial/DVL, acoustic LBL/USBL/OWTT, geophysical); context for dead reckoning plus acoustic aiding.',
 'eustice2011synchronous': 'Synchronous-clock one-way-travel-time ranging for underwater vehicles; the basis of passive OWTT range from scheduled broadcasts.',
 'webster2012advances': 'Single-beacon OWTT navigation results in field trials; error characteristics of one-way ranging from a broadcasting source.',
 'bahr2009cooperative': 'Cooperative AUV localisation with acoustic ranging and shared navigation data; contrast case that requires exchanging position estimates.',
 'rypkema2017oneway': 'OWTT inverted USBL: a passive receiver array gives range and bearing to a broadcasting beacon on a low-cost AUV; core sensing model of the system.',
 'rypkema2019passive': 'Experimental accuracy of passive inverted USBL (piUSBL) range and angle; empirical error figures for the ping-based relative localisation.',
 'rypkema2022synchronous': 'Synchronous-clock range-angle navigation of several AUVs relative to one broadcasting beacon, coordinated without inter-vehicle communication; closest sensing architecture.',
 'fischell2020single': 'Leader-follower underwater swarming with a single hydrophone and no time synchronisation; low-cost passive reception of a broadcast for relative navigation.',
 'syed2006time': 'Time synchronisation for high-latency acoustic networks; relevant to the synchronised-clock assumption behind OWTT and TDMA.',
 'ghabcheloo2009coordinated': 'Coordinated path following that synchronises along-path coordination states under communication losses and delays; formal model of formation progress synchronisation.',
 'lewis1997high': 'Virtual-structure formation control; formation slots defined on a shared rigid reference, as with slots on a shared path.',
 'oh2015survey': 'Survey classifying formation control by sensing and interaction topology (position, displacement, distance based).',
 'soares2013joint': 'Range-only formation keeping of an AUV with respect to two leaders, with unreliable low-rate acoustic ranging and sea trials.',
 'mirollo1990synchronization': 'Pulse-coupled oscillators synchronise from event-only coupling; theoretical basis for aligning clocks or phases from pings without payload.',
 'christensen2009fireflies': 'Decentralised detection of failed robots from the absence of expected synchronised flashes; heartbeat-style failure detection in a robot swarm.',
 'chandra1996unreliable': 'Theory of unreliable failure detectors (completeness and accuracy properties); formal vocabulary for declaring a neighbour missing.',
 'hayashibara2004phi': 'Accrual failure detector: a continuous suspicion level from heartbeat inter-arrival statistics instead of a fixed timeout.',
 'pang2024multi': 'Multi-AUV formation reconfiguration (affine transformation) with obstacle avoidance under ocean currents; formation reshaping in the underwater domain.',
 'akyildiz2005underwater': 'Research challenges of underwater acoustic networks: low bandwidth, long and variable delay, high loss.',
 'stojanovic2009underwater': 'Propagation models and statistics of the underwater acoustic channel (multipath, Doppler, fading); explains missed and spurious pings.',
 'stilwell2000platoons': 'Decentralised platoon control of underwater vehicles with a broadcast-only communication structure; early analysis of minimal communication.',
 'fossen2011handbook': 'Reference for marine-craft dynamics and current modelling used in simulation and controller design.',
 'potokar2022holoocean': 'HoloOcean multi-agent underwater simulator with sonar and acoustic communication models; simulation platform of the system.',
 'blackman2004multiple': 'Multiple hypothesis tracking overview, including track confirmation and deletion logic.',
 'musicki1994integrated': 'Integrated PDA: recursive probability of target existence with missed detections and clutter; basis for occupancy and neighbour-existence beliefs.',
 'ristic2013tutorial': 'Bernoulli filter tutorial: optimal Bayes filter for one object that may or may not be present, with detection probability below one and clutter.',
 'coue2006bayesian': 'Bayesian occupancy filter: grid occupancy with a prediction step that persists through short occlusions.',
 'wald1945sequential': 'Sequential probability ratio test: accumulate evidence until a decision threshold with bounded error probabilities; basis for exit or vacancy decisions.',
 'gunes2016joint': 'Bernoulli filter for joint detection and tracking of an underwater target with an acoustic vector sensor; underwater use of existence filtering.',
}
REASON = {
 'R1': 'Redundant with an included work that covers the same contribution (scope cap of about 60 items)',
 'R2': 'Superseded by a later version of the same work',
 'R3': 'Depends on inter-vehicle message exchange or a central coordinator; the contrast is already represented by an included work',
 'R4': 'Platform or problem setting less transferable to the system (surface vessels, spacecraft, manipulators, vision, broad survey) or tangential',
}
EXC = {
 '10.1002/rob.21442': 'R1', '10.1007/978-3-030-66723-8_18': 'R2', '10.1007/bfb0030680': 'R1', '10.1007/s10817-008-9103-8': 'R1',
 '10.1016/j.automatica.2007.02.018': 'R1', '10.1016/j.ifacol.2015.11.154': 'R1', '10.1016/j.jlap.2008.08.004': 'R1',
 '10.1016/j.oceaneng.2010.07.006': 'R3', '10.1016/j.oceaneng.2011.07.017': 'R1', '10.1016/s0890-5401(03)00067-1': 'R1',
 '10.1080/00207540110118073': 'R3', '10.1109/2.30720': 'R1', '10.1109/48.775299': 'R1', '10.1109/70.736776': 'R1',
 '10.1109/70.88147': 'R1', '10.1109/87.960341': 'R4', '10.1109/auv.2014.7054411': 'R4', '10.1109/ccta60707.2024.10666518': 'R4',
 '10.1109/iros.2001.977148': 'R1', '10.1109/iros.2005.1545230': 'R1', '10.1109/joe.2006.880429': 'R1', '10.1109/joe.2013.2254214': 'R4',
 '10.1109/jproc.2006.887293': 'R3', '10.1109/lics.1996.561342': 'R1', '10.1109/lra.2021.3091016': 'R3', '10.1109/mcs.2008.929280': 'R4',
 '10.1109/mcs.2009.934469': 'R1', '10.1109/tac.1979.1102177': 'R1', '10.1109/tac.2004.832203': 'R4', '10.1109/tac.2005.864190': 'R4',
 '10.1109/tac.2015.2459191': 'R4', '10.1109/taes.1981.309141': 'R1', '10.1109/taes.2003.1261119': 'R4', '10.1109/tase.2015.2445780': 'R3',
 '10.1109/tcst.2010.2050889': 'R4', '10.1109/tcst.2015.2420636': 'R1', '10.1109/tits.2013.2252901': 'R3', '10.1109/tits.2015.2471812': 'R3',
 '10.1109/tra.2002.803463': 'R4', '10.1109/tro.2009.2030225': 'R4', '10.1109/tro.2013.2252857': 'R3', '10.1109/tro.2021.3075371': 'R3',
 '10.1145/1098918.1098934': 'R1', '10.1145/1347364.1347372': 'R1', '10.1145/3342355': 'R4', '10.1145/358527.358537': 'R3',
 '10.1145/359545.359563': 'R3', '10.1145/361179.361202': 'R4', '10.1145/365559.365617': 'R1', '10.1145/37402.37406': 'R4',
 '10.1177/0278364910380760': 'R3', '10.1177/027836498600500106': 'R4', '10.1177/027836499801700706': 'R1', '10.1504/ijmic.2006.008645': 'R1',
 '10.23919/ecc.2019.8796030': 'R1', '10.3390/app10041256': 'R1', '10.4031/002533208786861263': 'R1',
}
OA_AUTH = {'10.1109/37.887448': ['Stilwell, D. J.', 'Bishop, B. E.'], '10.1109/mcs.2009.934469': ['Bar-Shalom, Y.', 'Daum, F. E.', 'Huang, J.'],
           '10.1109/mcs.2008.929280': ['Anderson, B. D. O.', 'Yu, C.', 'Fidan, B.', 'Hendrickx, J. M.']}
YEAR_FIX = {'10.1109/robot.1985.1087316': 1985, '10.1109/iros.2001.977148': 2001, '10.1109/lics.1996.561342': 1996}
TITLE_FIX = {'10.1109/reldis.2004.1353004': 'The phi accrual failure detector'}

def clean(s):
    s = html.unescape(re.sub(r'<[^>]+>', '', s or '')).replace('‐', '-').replace('–', '-').replace('—', ', ')
    return re.sub(r'\s+', ' ', s).strip()

def initials(g):
    g = clean(g)
    return ' '.join(p[0] + '.' for p in re.split(r'[\s.]+', g) if p) if g else ''

def load(doi):
    for f in glob.glob(os.path.join(D, 'verify', '*.json')):
        pass
    cands = [doi.replace('/', '_').replace('(', '').replace(')', ''), doi.lower().replace('/', '_').replace('(', '').replace(')', '')]
    for c in cands:
        p = os.path.join(D, 'verify', c + '.json')
        try:
            return json.load(open(p))['message']
        except Exception:
            continue
    raise FileNotFoundError(doi)

def year(m, doi):
    if doi.lower() in YEAR_FIX: return YEAR_FIX[doi.lower()]
    for k in ('published-print', 'issued', 'published-online'):
        dp = (m.get(k) or {}).get('date-parts') or [[None]]
        if dp[0][0]: return dp[0][0]

def fullref(m, doi):
    au = m.get('author') or []
    if au:
        names = [f"{clean(a.get('family'))}, {initials(a.get('given'))}".strip(', ') if a.get('given') else clean(a.get('family') or a.get('name')) for a in au]
    else:
        names = OA_AUTH.get(doi.lower(), ['[authors not in Crossref]'])
    if names == ['Lui Sha']: names = ['Sha, L.']
    if len(names) > 1: auth = ', '.join(names[:-1]) + ' and ' + names[-1]
    else: auth = names[0]
    title = TITLE_FIX.get(doi.lower(), clean((m.get('title') or [''])[0]))
    ct = clean((m.get('container-title') or [''])[0]) or clean(m.get('publisher', ''))
    vol, iss, pg = m.get('volume') or '', m.get('issue') or '', m.get('page') or m.get('article-number') or ''
    loc = (f', {vol}' + (f'({iss})' if iss else '') if vol else '') + (f', {pg}' if pg else '')
    return f'{auth} ({year(m, doi)}). {title}. *{ct}*{loc}.'

inc = [l.split() for l in open(os.path.join(D, 'scripts', 'included.txt')) if l.strip()]
tables = {}
for t, key, doi in inc:
    m = load(doi)
    row = f"| {fullref(m, doi)} | [{m['DOI']}](https://doi.org/{m['DOI']}) | yes | {REL[key]} |"
    tables.setdefault(t, []).append(row)
hdr = '| Reference | DOI | Verified | Relevance to the system |\n|---|---|---|---|\n'
tpl = open(os.path.join(D, 'scripts', 'template.md'), encoding='utf-8').read()
for t, rows in tables.items():
    tpl = tpl.replace('{{TABLE:' + t + '}}', hdr + '\n'.join(rows))
# excluded table
rows = []
for doi, r in sorted(EXC.items()):
    m = load(doi)
    au = m.get('author') or []
    fa = clean(au[0].get('family')) if au else OA_AUTH.get(doi, ['?'])[0].split(',')[0]
    etal = ' et al.' if (len(au) > 2 or (not au and len(OA_AUTH.get(doi, [])) > 2)) else (' and ' + clean(au[1].get('family')) if len(au) == 2 else '')
    rows.append(f"| {fa}{etal} ({year(m, doi)}). {TITLE_FIX.get(doi, clean((m.get('title') or [''])[0]))} | {m['DOI'].lower()} | {r} |")
tpl = tpl.replace('{{EXCLUDED}}', '| Work | DOI (verified on Crossref) | Reason |\n|---|---|---|\n' + '\n'.join(rows))
tpl = tpl.replace('{{REASONS}}', '\n'.join(f'- **{k}**: {v} (n = {sum(1 for x in EXC.values() if x == k)})' for k, v in REASON.items()))
# per-query table
qrows = []
for fn in ('search_compact.txt', 'search_supp.txt'):
    for line in open(os.path.join(D, fn), encoding='utf-8'):
        mm = re.match(r'## (\S+) "(.*)" OA=(\d+) CR=(\d+) S2=(\d+) unique=(\d+)', line)
        if mm: qrows.append('| ' + ' | '.join(mm.groups()) + ' |')
tpl = tpl.replace('{{QUERIES}}', '| Tag | Search string | OpenAlex | Crossref | Semantic Scholar | Unique records |\n|---|---|---|---|---|---|\n' + '\n'.join(qrows))
tpl = tpl.replace('{{BIB}}', open(os.path.join(D, 'included.bib'), encoding='utf-8').read().strip())
out = os.path.join(D, 'LITERATURE_REVIEW.md')
open(out, 'w', encoding='utf-8').write(tpl)
left = re.findall(r'\{\{[A-Z]+[:A-Z0-9]*\}\}', tpl)
print('written', out, 'unfilled:', left, 'excluded rows', len(rows), 'em-dashes', tpl.count('—'))
