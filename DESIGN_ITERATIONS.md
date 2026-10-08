# Design iterations (v2: realistic sonar, generic formations)

Every problem met during the v2 work, with its cause and the fix actually adopted.
v1 iterations are summarised in REPORT.md section 6.

---

## DI-1 - Runtime-spawned arena gates are invisible to HoloOcean sonars

* **Problem.** A sonar created in the scenario never echoes the Marine Race Arena gates or any other
  runtime-spawned prop. It does echo other BlueROV2s. Probe session k1: gate and box 0 % detection
  (`figures/v2/sonar_probe/octree_props_k1_k2_k3.png`). v1 had observed the same symptom with the imaging
  sonar and worked around it with ray-cast rings.
* **Cause.** HoloOcean sonars run on a static octree. It is built by `ECC_WorldStatic` overlap tests when
  the first sonar initialises, which happens inside `env.reset()`, before any prop can be spawned. It is
  cached on disk keyed only by map name and octree size (engine source `Octree.cpp`,
  `HolodeckSonar.cpp`). Agents are excluded and get moving per-agent octrees, which is why vehicles stayed
  visible.
* **Fix.**
  1. Spawn the static scene first.
  2. Attach the sonars afterwards with `agent.add_sensors`.
  3. Use a project-private octree cache folder (`octree_min` 0.06 m) that is invalidated whenever the
     static scene changes.
* **Why the fix is principled.** It changes nothing in the sensor model. It restores the assumption the
  sonar model is built on, namely that the octree describes the actual static scene. No substitute sensor,
  no ray casting, no hand-made echoes.
* **Result.** Session k2: gates, a runtime box and BlueROV2s are detected in 100 % of captures within
  range, with errors of at most 0.055 m (`docs/v2/SONAR_PROBE.md`).

## DI-2 - A stale octree cache produces ghost obstacles

* **Problem.** In session k3, nothing was spawned. The sonar nevertheless echoed a gate pillar at
  1.875 / 2.875 m and a box at 2.025 / 3.025 m: exactly where session k2 had them.
* **Cause.** The cache key (map name and octree size) does not include the spawned props. A cache written
  in a run with props is replayed in a later run without them.
* **Fix.** The cache is keyed by a signature of the static scene (gate layout and props). The
  project-private folder is deleted whenever the signature differs. HoloOcean's own
  `delete_world_octrees` is never used, because it would wipe other projects' caches.
* **Why the fix is principled.** A cache must be invalidated when the data it summarises changes. The key
  is extended with the missing variable instead of disabling the cache.
* **Result.** Verified by construction in k1, k2 and k3. It becomes part of the simulator wrapper in
  Phase 3.

## DI-3 - Probe truth: PoseSensor is not at the hull origin

* **Problem.** The first analysis of the probe showed an apparent +0.22 m range bias for a head-on
  BlueROV2 and +0.10 to +0.14 m for static props.
* **Cause.** This was an analysis error, not a sensor error. `PoseSensor` sits at the BlueROV2
  `IMUSocket`, at (+0.11, 0, -0.065) m from the agent origin in the body frame. This offset was measured
  from the probe's pinned poses with zero spread. The truth geometry had used the socket as the hull
  centre.
* **Fix.** The truth geometry uses the agent origin (pose minus the socket offset) and the BlueROV2 heavy
  hull box (half extents 0.229 x 0.288 x 0.127 m).
* **Why the fix is principled.** The offset is measured, not fitted to the sonar. The sonar's own echoes
  (0.225 m ahead, 0.275 m abeam) independently confirm the hull extents within one bin.
* **Result.** Errors drop to at most one bin (`figures/v2/sonar_probe/range_true_vs_sonar.png`). The same
  correction will be used by the v2 referee, whose `d_ij` is measured between agent origins.

## DI-4 - Root fix instead of a workaround: a HoloOcean `RebuildSonarOctree` command

* **Problem.** DI-1 and DI-2 had a working but fragile workaround: spawn everything first, attach the
  sonars afterwards, and wipe the cache when the scene changes. It breaks as soon as the scene changes
  while sonars exist, and it imposes an order on the application code.
* **Cause.** HoloOcean has no way to tell its sonars that the static scene changed.
* **Fix.** A minimal patch to HoloOcean 2.3.0, kept in its own checkout
  `F:\Andrea\holoocean-octree-patch`. The diff is in `patches/` and is documented in
  `docs/HOLOOCEAN_OCTREE_PATCH.md`. It adds:
  * an engine command, `RebuildSonarOctree`, which invalidates the cache of the current level and octree
    size (the whole folder, or only `roots.json` plus the 7.68 m cells touching a region);
  * `UHolodeckSonar::ResetOctree()` on every sonar, so the octrees are rebuilt from the current scene
    with no restart;
  * the client API `env.rebuild_sonar_octree(region=None)`.

  The patched engine is built with UE 5.3.2 and installed into a separate HoloOcean root
  (`F:\Andrea\holoocean_patched_root`). Its content is a junction to the official package and its octree
  cache is private. The official installation, also used by the Marine Race Arena's frozen benchmark, is
  untouched.
* **Why the fix is principled.**
  * The sonar model is unchanged.
  * The engine gets the one capability it lacked: invalidating its cache when its input changes.
  * Caches of other levels and octree sizes are never touched.
* **Result.** `probe/octree_rebuild_regression.py` passes every check, starting from the hard case
  (sonar created first):
  * box, arena gate and runtime BlueROV2 are invisible before the rebuild and visible after it;
  * a region rebuild works;
  * after a reset the ghost box is present before the rebuild and absent after it;
  * a moved box is seen at its new place, and the old place is silent;
  * the official cache is untouched.

  A rebuild costs 0.2-0.45 s (15 ticks) with the project's 128 x 128 x 40 m environment box.

## DI-5 - Engine crash: a sonar dereferenced an agent destroyed by a reset

* **Problem.** The first regression run crashed the engine (`EXCEPTION_ACCESS_VIOLATION` in
  `AHolodeckBuoyantAgent::makeOctree`, called from `UHolodeckSonar::initOctree`) right after a
  `reset()`. The Python client then waited forever, because timeouts are off while a sonar exists.
* **Cause.** This is a pre-existing HoloOcean bug. Agents add themselves to the server's `AgentMap`, a
  plain `TMap` of raw pointers, and are never removed. A reset dropped the runtime-spawned agent "late",
  and the next sonar initialisation iterated the map and dereferenced the dangling pointer. The
  `static_cast` in the loop would also accept agents of other classes.
* **Fix.**
  * `AHolodeckAgent::EndPlay` removes the agent from `AgentMap`.
  * The sonar loops use `Cast<>` plus `IsValid()` (second commit of the patch).
  * On our side, `holo_fleet/sim/engine_watchdog.py` terminates the Python process with a clear message
    if the engine dies, instead of hanging.
* **Why the fix is principled.** The object that registers itself also unregisters itself; no stale
  state is left for later readers. The watchdog does not hide the crash, it reports it.
* **Result.** Reset after a runtime spawn, then sonar re-initialisation: no crash (regression section
  F-H).

## DI-6 - Axis-aligned faces lying on octree cell boundaries vanish

* **Problem.** A 0.5 m box 3 m ahead was invisible head-on but visible from the side, even with a fresh
  octree. The official engine showed the same box, but only because that run used 7 cm leaves.
* **Cause.** This is pre-existing HoloOcean discretisation. The box face was at x = 324 cm, an exact
  multiple of the 6 cm leaf size:
  * leaves behind the face are "full" and are dropped (the octree keeps only surface leaves);
  * leaves in front only touch the face, and the overlap test counts touching as empty;
  * so the face disappears.

  It would affect any axis-aligned prop face at a multiple of the leaf size. The arena gates are rotated
  and were not affected.
* **Fix.** The occupancy tests use a box 5 mm larger than the cell, so a cell touching a surface is
  occupied and becomes a surface leaf (third commit of the patch).
* **Why the fix is principled.** It removes a measure-zero geometric degeneracy without changing the
  sensor model. The induced range bias is at most half a leaf, on the safe side (the echo comes slightly
  earlier).
* **Result.** The same box is seen head-on at 2.975 m (expected 3.0 m) and at 4.975 m (expected 5.0 m).
  The full regression passes.

## DI-7 - Regression test polluted by a previous run's cache

* **Problem.** In the second regression run the gate was "visible before the rebuild".
* **Cause.** The patched root's private cache still held the cells written by the crashed first run.
  The test assumed a clean start.
* **Fix.** Every scene starts with `env.rebuild_sonar_octree()` before anything is spawned, and every
  scene change is followed by another rebuild. The simulator wrapper does the same: one rebuild right
  after the static scene is spawned.
* **Why the fix is principled.** The octree state is established explicitly instead of being inherited.
* **Result.** The regression passes from any cache state.

## DI-8 - Coverage bench: "blind" directions that were a placement artifact

* **Problem.** In the first coverage bench every direction with no forward component (pure lateral,
  pure vertical, lateral-vertical) looked blind. The same positions were detected in a plain azimuth
  sweep.
* **Cause.** `set_physics_state` moves an agent with a *sweep*. When the straight path from the previous
  test pose crossed the observer, the target stopped at contact, stuck to the observer's hull
  (0.46 m behind it). It then stayed stuck for the following placements. The sensors were fine.
* **Fix.** The bench places the target with a detour: the path is checked against the observer and
  re-routed through a waypoint 10 m out, the target comes in radially, and every case records the
  target's true position. Cases where the target is not where it was put are marked invalid.
* **Why the fix is principled.** The measurement is validated against ground truth instead of being
  trusted. The sensor model was never touched.
* **Result.** `scripts/sonar_bench.py coverage` covers 204 cases (6 axes, 12 edges, 8 diagonals x 2
  target yaws, centre distance 1-5 m). All 204 are valid and detected, and the observed sector pattern
  is always within the geometric prediction (must <= seen <= maybe). The worst-case diagonals are
  covered at every distance.

## DI-9 - With several sonar-equipped drones, some drones were invisible to every sonar

* **Problem.** In the first head-on run drone_1 never saw drone_0, while drone_0 saw drone_1. The
  single-observer benches had not exposed it: there only the observer's view was checked.
* **Cause.** This is pre-existing HoloOcean behaviour. Agent octrees are built with the *shared*
  collision query `Octree::params`. The first sonar that initialises adds every agent to that query's
  ignore list, because the world octree must not contain agents. Any agent octree built afterwards is
  swept with a query that ignores that very agent: the build fails and the agent is invisible to all
  sonars. With six sonars per drone, only the agents built by the very first sonar were visible.
* **Fix.** Fourth commit of the engine patch: agent octrees use a clean collision query. The
  actor-name filter already restricts hits to the agent itself.
* **Why the fix is principled.** Each query gets the parameters its purpose requires. Nothing about
  what a sonar can see changes, except removing an order-dependent failure.
* **Result.** Both drones of the head-on pair see each other from 8 m. Every drone sees every other
  in the 6-drone runs.

## DI-10 - Head-on standoff: cone uncertainty forbids a guaranteed sidestep

* **Problem.** The first head-on runs were safe (minimum distance 2.6-2.8 m) but the drones did not
  cross. One retreated while the other pushed, or both sidestepped too late and stalled abeam inside
  the warning band.
* **Cause.**
  * A FRONT-only echo leaves the bearing uncertain by about +/-50 deg. No lateral motion has a
    guaranteed opening against the whole region, so the warning filter can only open by moving
    backwards. That is correct for safety but gives no way through.
  * Abeam at about 2.8 m the conservative distance is inside the band, and again only opening motions
    are certified.
* **Fix.** A mission-level traffic rule (as COLREG rule 14), decided from sector patterns before the
  warning band:
  * a DYNAMIC echo closing in FRONT within 8 m makes the drone shift its mission target 2.5 m to its
    right;
  * if a neighbour occupies the right side, the shift is vertical: up when heading east-ish, down
    otherwise, which is opposite for two drones meeting head-on;
  * the shift is released only once the other drone is behind (REAR/UP/DOWN only) or beyond 5 m.
* **Why the fix is principled.** Safety still rests only on the certified filter and escape (P1).
  The rule addresses liveness (no standoff) with a symmetric, communication-free convention, like
  maritime traffic.
* **Result.** `p1_head_on`: both drones give way at 8 m and pass port to port at 4.8 m, without
  entering the warning band. `p1_vertical_escape`: drones flanked by a neighbour give way UP (one
  line) and DOWN (the other line), and the end drones give way to the right. Minimum distance
  3.1 m, 0 collisions.

## DI-11 - Single-capture false alarms from the intensity-noise tail

* **Problem.** In a 6-drone run, two single-capture UNKNOWN echoes (0.93 m UP, 2.22 m FRONT) put a
  drone into COLLISION_AVOIDANCE / SEPARATION_WARNING for one control step. No drone was there.
* **Cause.** The detection threshold was 0.25 with Rayleigh intensity noise sigma = 0.05. The
  per-bin exceedance probability is exp(-0.25^2 / (2 * 0.05^2)) = 3.7e-6. Per 234-bin profile that
  is about 9e-4, which means about ten false alarms over the roughly 13 000 captures of a 36 s run
  with 36 sonars. The Phase-1 empty-water test (60 captures) was too short to see them.
* **Fix.** Threshold 0.30, giving a per-bin tail of exp(-18) = 1.5e-8, i.e. about 0.05 expected
  false alarms per run. Vehicle and structure echoes have intensity 0.7-0.95 and Phase 1 measured
  100 % detection up to threshold 0.7.
* **Why the fix is principled.** The threshold is set from the noise model's tail probability and
  the measured detection margin, not tuned until a test passes. The conservative reaction to
  unconfirmed echoes is kept.
* **Result.** Re-measured in the following runs (no spurious avoidance events expected).

## DI-12 - Gate bars echo far outside the nominal cone at short range

* **Problem.** While crossing gate G06 a drone entered COLLISION_AVOIDANCE for 0.1-0.3 s several
  times, on UNKNOWN echoes at 0.6-1.0 m. The passage took 22 s instead of about 14 s.
* **Cause.** The echoes were the gate posts, but the predicted structure window did not contain
  them. At 0.63 m the FRONT sonar returned a post 78 deg off its axis, 18 deg outside the nominal
  60 deg cone. A strong reflector close to the transducer is detected outside the main lobe, as
  with a real wide beam, and the bar's octree leaves straddle the cone boundary. The probe (60 deg;
  62.5 deg at 3 m) used BlueROV2 targets at 1.3 m and beyond, where the effect is small.
* **Fix.** The structure window uses a cone widened by atan(0.25 m / r): 22 deg at 0.6 m, 5 deg at
  3 m, 2 deg at 7 m. The same near-field widening enters the sound distance bound (DI-13).
* **Why the fix is principled.** It changes the prediction of where the mapped structure can echo,
  not the decision thresholds. A wider window can only mask a drone near the bars, and the gate
  protocol already handles that (occupancy latch, DI-14).
* **Result.** No avoidance mode while crossing the gate in `gate_single` and `integrated_short`.

## DI-13 - The conservative distance could exceed the true distance

* **Problem.** A numeric check of the onboard distance bound, written for the formal suite (S0),
  found poses where the bound was above the true centre distance by up to 0.38 m. That is an
  unsound bound for a safety guard.
* **Cause.** The bound assumed L = r + R_IN from the sensor to the hull centre, which is true when
  the centre is inside the cone. When only a hull corner reaches into the cone while the centre is
  outside, the centre can be nearer than r + R_IN. This happens at cone edges, and for the
  cube-corner directions below 3 m because of the sensor parallax. Taking the minimum over the
  member sectors did not help near the corners, where no sector contains the centre.
* **Fix.**
  * Every member sector gives a bound that is always sound. The nearest in-cone hull point is at
    |p| >= sqrt(r'^2 + |m| r' + |m|^2) (law of cosines with the cone half-angle plus the near-field
    widening), and the centre is within R_OUT of that point.
  * The tight r + R_IN bound is used only where the sector pattern and the sound bound certify that
    the centre is inside that cone (region tables per distance band, parallax, 5 deg hull fuzz).
  * The target keeps the minimum over its sectors, so two objects merged into one target are both
    bounded.
* **Why the fix is principled.**
  * The bound is derived, then checked numerically on 10^5 random BlueROV2 poses, with a mutation
    (the uncertified bound) that the check must catch.
  * The P1 proof needs only the one-sided property d_hat <= d; the looseness (median 0.8 m,
    max 1.2 m) only makes the warning start earlier.
* **Result.**
  * S0: 0 violations, minimum margin 0.07 m.
  * Lateral formation and queue neighbours at 3.5 m keep the tight bound (about 3.0 m, outside the
    2.7 m warning exit).
  * All scenarios were re-run with the sound bound: P1 holds in all of them.

## DI-14 - The occupancy belief latched BUSY on noise and on the queue itself

* **Problem.** In `gate_single` the first queued drone had PRIORITY at 6.4 s but committed only at
  28.2 s, when its CR belief timed out (t_occ_max = 25 s).
* **Cause.** Two triggers:
  * during the approach, the corridor test ("FRONT echo nearer than the far end of the CR") also
    held for the centre drone already waiting at its queue point;
  * in another run, an echo 1.2 m beyond the far gate post, seen in 2 of 3 captures, was confirmed
    as a DYNAMIC target in the corridor. The exponential range noise is applied per octree leaf:
    the farthest of about 10^3 leaves lies about 0.05 ln N = 0.4 m beyond the surface. Structure
    echoes are therefore lengthened on the far side only.
* **Fix.**
  * The corridor is the range interval [r_near - R_OUT - eps_far, r_far + eps_near] of the CR seen
    from the drone's own FRONT sonar (gate map + own pose). Nearer drones are WAIT relations, not
    occupancy.
  * The structure window gets an asymmetric tolerance: 0.3 m near, 0.7 m far.
  * Gate decisions use only echoes that persist over two updates.
* **Why the fix is principled.**
  * The corridor is now exactly the set of ranges at which a hull centred in the CR can echo.
  * The far tolerance follows from the sonar noise model.
  * A drone inside the CR is seen for several captures before the bars mask it (formal M4d), so
    persistence costs no safety.
* **Result.**
  * First commit at 7.5 s.
  * Formal M3: the latch model holds for every queue view (n = 3, 4).
  * The variant without latch gives a counterexample.

## DI-15 - Single-capture echoes reached the safety guards

* **Problem.** While passing the gate, a drone entered SEPARATION_WARNING twice on REAR echoes at
  1.9-2.0 m that appeared in a single capture: the far corners of the gate frame grazing the cone.
* **Cause.** Only DYNAMIC echoes had a confirmation step. "Too extended" and the other UNKNOWN
  echoes went straight to the guards.
* **Fix.** Every unexplained echo needs M-of-N confirmation.
  * It becomes DYNAMIC or UNKNOWN only once an unexplained echo has been seen within 0.5 m in at
    least 2 of the last 3 captures.
  * Before that it is UNCONFIRMED: logged and shown on the dashboard, but not used by the guards.
  * The seabed-clutter pseudo-target is not an echo and is not delayed.
* **Why the fix is principled.** This is the standard M-of-N detection logic of sonar trackers. The
  added latency (one capture, 0.1 s) is inside the staleness budget tau_max = 0.25 s used by the P1
  derivation.
* **Result.** No avoidance mode is caused by single captures. The classification bench A-F is
  unchanged.

## DI-16 - BACKOFF at the queue from two drones merged into one target

* **Problem.** The centre queued drone repeatedly backed off and re-approached while the left drone
  was passing. The logged relation was BACKOFF, "FRONT+RIGHT at 2.87 m".
* **Cause.** The passing drone (FRONT) and the right queue neighbour (RIGHT) were at similar ranges.
  The target builder associated them into one FRONT+RIGHT target. The table read that as a single
  drone in the ambiguous zone: BACKOFF.
* **Fix.** A simpler table: WAIT if FRONT or LEFT is in the pattern; PRIORITY if only REAR and/or
  RIGHT; RANK if only UP and/or DOWN; no BACKOFF state.
  * Two drones at their queue points cannot wait for each other: they are abreast, so their
    relation is pure LEFT/RIGHT with a 12 deg margin.
  * A WAIT caused by an approaching or passing drone ends by itself.
* **Why the fix is principled.** A merged FRONT+RIGHT target now gets WAIT, which is safe for both
  readings. The rule is checked formally:
  * M1: never both PRIORITY when |delta| + 2 fuzz <= 30 deg (the bound is tight, shown by mutation);
  * M2: exactly one PRIORITY in abreast queues of 2 to 6 drones.
* **Result.** No BACKOFF. Entry order is left, centre, right in every gate run, and the static rank
  is never used.

## DI-17 - Gate queue geometry and the rendezvous beyond the gate

* **Problem.** Three problems with the first gate design:
  * the committed drone merged diagonally across the queue line, passing 2.4 m from the next queued
    drone (warning);
  * for wide queues the CR left the FRONT cone of the outer queue points;
  * after the gate the formation clock was still behind the gate, so passed drones would have
    turned back towards it.
* **Cause.** The queue parameters were fixed (s_queue = -5 m for any n), and the formation clock
  knew nothing about the gate.
* **Fix.**
  * `GateRule.queue_s(n)` is the smaller of two bounds:
    * the CR inside every queue sonar's FRONT cone: 49 deg, plus 6 deg heading tolerance, plus
      5 deg fuzz;
    * a 55 deg descent to the axis that keeps merge_clearance = 3.2 m from the next queue point.
  * The pass path: follow the own lane, descend to the axis 0.7 m before the CR, cross, then veer
    back to the own lane.
  * Rendezvous: as soon as every drone is in the approach zone, the formation reference jumps to a
    point 6 m beyond the gate and holds there for a planned time budget. The jump and budget are
    part of the mission plan; no communication is involved.
* **Why the fix is principled.** The geometry is derived from the sensor cone and the separation
  thresholds, and checked formally (M4a-e) for n = 2..6. The rendezvous is ordinary mission
  planning.
* **Result.** In `gate_single` three drones pass one at a time (left, centre, right), with maximum
  occupancy 1, P1 minimum 3.2-3.3 m, and the formation re-formed beyond the gate.

## DI-18 - Slow formation recovery after the jet

* **Problem.** In `formation_gust` the formation was lost at 24.7 s and recovered only at 46.1 s
  (21.4 s). A later variant declared itself recovered while a neighbour was still missing.
* **Cause.** Three causes:
  * The displaced neighbour was outside the 1.2 m range gate, so it was no longer associated as
    "expected". The head-on traffic rule then made a formation drone give way upwards inside its
    own formation, twice.
  * The catch-up margin was only v_slot_max - v_clock = 0.1 m/s, so 2 m of slot error took 20 s.
  * One echo could match two expected neighbours that were both behind, hiding a missing one.
* **Fix.**
  * Echoes within 3 m of an expected neighbour's distance, in compatible sectors, count as
    "possible neighbours". They never trigger the traffic rule, which also requires closing at
    >= 0.45 m/s.
  * In FORMATION_RECOVERY the speed cap is 0.5 m/s (= v_max_nominal), a catch-up margin of
    0.2 m/s.
  * Neighbour association is one-to-one, and a pattern pointing to the other side of the hull is
    incompatible.
* **Why the fix is principled.**
  * The margin is what the P3 ranking function needs: the mutation Fm1 shows that without a
    catch-up margin no decrease can be certified.
  * The traffic rule is a liveness aid and must not act on the drone's own formation.
* **Result.** In repeated development runs (one seed each), `formation_gust` recovers 6-7 s after
  the jet ends. The triangle, square and six-drone runs never lose the formation.

## DI-19 - The P1 demo never exercised the escape, then the escape chattered

* **Problem.** Three successive issues:
  * in the two-lines head-on scenario the traffic rule resolved every encounter, so the escape
    planner (the safety layer) never acted;
  * with a non-cooperative vehicle, the escape alternated between UP and DOWN every 0.1-0.3 s;
  * the rear drone was then chased backwards for 20 s by the scripted vehicle.
* **Cause.** Respectively:
  * the traffic rule is designed to remove the encounter before the warning band;
  * nearly tied candidates flipped with the vertical component of the mission velocity;
  * the scripted vehicle moved at constant velocity along the formation's centre line.
* **Fix.**
  * `p1_vertical_escape` is now a T formation crossed head-on, 1.2 m below, by a scripted vehicle
    outside the fleet, with the traffic rule switched off for this scenario only. The old scenario
    remains as `p1_two_lines`.
  * The escape planner keeps a previous direction that still opens every threat and scores within
    0.10 of the best.
  * The scripted vehicle dives away after crossing.
* **Why the fix is principled.**
  * The demo shows what the formal P1 argument relies on: the warning filter and the escape alone.
  * The hysteresis keeps only certified choices.
  * The intruder is labelled as outside the fleet and outside P1; its clearance is reported
    separately.
* **Result.** Both the boxed-in centre drone (REAR+UP, FWD+UP) and the rear drone escape upwards.
  P1 among the drones holds (minimum 2.5 m), the clearance to the intruder stays above 2.4 m, and
  the formation recovers 18.5 s after the encounter.

## DI-20 - Square drones never declared the formation recovered

* **Problem.** In the final `formation_square` run the referee saw a perfect formation (error below
  0.08 m), but all four drones stayed in FORMATION_RECOVERY for the whole run. Six-drone drones were
  in it most of the time.
* **Cause.** Two effects of DI-18's one-to-one association.
  * In a 2 x 2 box the side and the rear neighbour echo at almost the same range in adjacent sectors.
    The target builder merges them into one REAR+RIGHT target, which could confirm only one of them.
  * In the six-drone line the returns of neighbours 10.5 m away are intermittent near the end of the
    range, so `neighbors_ok` flickered.
* **Fix.**
  * Association per (target, sector): each sector of a merged target can confirm one neighbour.
    The direction compatibility of DI-18 is kept.
  * A neighbour counts as present for 1 s after its last match.
  * Only neighbours expected within 7.5 m are required.
* **Why the fix is principled.**
  * A merged target is two hulls seen in two sectors, so each sector carries its own evidence.
  * The memory matches the sonar's measured intermittency at long range.
  * Requiring the nearer neighbours is what the spacing correction actually uses.
* **Result.** Triangle and square: every drone in FORMATION_FOLLOW for 42.8 of 45 s (after the
  initial 2.2 s). New unit tests: merged target, intermittent far neighbour.

## DI-21 - A neighbour hidden in the gate's echo window was reported missing

* **Problem.** In `gate_single` the last drone through the gate never declared the formation
  recovered. The referee did declare it, at 73 s.
* **Cause.** At the rendezvous, 6 m beyond the gate, the neighbour 7 m away on the drone's left lies
  at the same range as the gate bars behind-left of it. Its echo (6.42 m) falls inside the structure
  window of that sector and is classified STRUCTURE. For a range-only sensor the neighbour is
  unobservable there, but the formation check counted it as missing.
* **Fix.** An expected neighbour whose predicted echo lies inside a mapped structure's window, in
  every sector where it is expected, is not required for `neighbors_ok`. It is still used when it is
  matched.
* **Why the fix is principled.** It separates "not seen" from "cannot be seen with this sensor and
  this map". Nothing changes for P1: a masked hull is a documented limitation, covered for the gate by
  the occupancy latch.
* **Result.** See the final `gate_single` and `integrated_short` runs in the README table.

## DI-22 - The formal observation domain still carried a v1 relation

* **Problem.** `formal/common.Obs.legal`, the set of observations over which the determinism and P3
  checks quantify, contained `passed -> at_queue`. In v2 `passed` means s > exit_s (beyond the gate)
  and `at_queue` means "within 0.35 m of the own queue point" (s < 0), so the two are never true
  together. Every reachable observation with `passed` was outside the domain.
* **Cause.** In v1 `at_queue` was the half-line "s beyond the commit window start", which `passed`
  implied. v2 redefined `at_queue` and nobody re-checked the domain. Nothing tested that the domain
  still contained the observations the perception produces.
* **Fix.**
  * The domain is now the conjunction of the observation invariants of
    `holo_fleet/ha/observation_invariants.py` (DI-26), the same predicate as the runtime check.
  * `formal/check_observations.py` checks that the domain is satisfiable and that each invariant
    excludes something. It also checks that every edge of every mode is enabled on some consistent
    observation (non-vacuity).
  * Mutation Om2 puts the v1 relation back: `pass_done` becomes unreachable (expected UNSAT).
* **Why the fix is principled.** The domain of a proof must contain every reachable observation.
  Writing it once and monitoring it at runtime makes any relation the code does not guarantee
  visible.
* **Result.**
  * Determinism (66/66) and P3 (11/11) hold over the corrected domain.
  * The guards are deterministic over every Boolean combination anyway (formal O5), so the v1
    conclusion was right. Its proof was vacuous for the exit edge.

## DI-23 - For five or more drones the outer queue points lay outside the approach zone

* **Problem.** Found while writing the invariants.
  * `at_queue -> gate_zone` should hold, but `gate_zone` required |l| <= corridor_half_width = 6.5 m.
  * The abreast queue points are at ((n - 1)/2 - r) x 3.5 m, i.e. ±7.0 m for n = 5 and ±8.75 m for
    n = 6.
* **Consequence.** A drone holding an outer queue point would have `at_queue & !gate_zone`. The
  automaton would then take a formation edge instead of yield or commit (formal Q1; mutation Om1
  shows the counterexample). The defect was latent: the gate scenarios use three drones.
* **Fix.** The lateral bound of the approach zone is max(corridor_half_width, |queue_l| + 1.0).
* **Why the fix is principled.** The approach zone is where the gate protocol applies, so it must
  contain the drone's own queue point. The 1.0 m margin covers queue_tol (0.35 m). Nothing changes
  for n <= 4.
* **Result.** I2 holds by construction. A unit test checks it for n = 3..6 over every queue point and
  its tolerance box.

## DI-24 - No current inside every assumption breaks the formation; against the motion the claimed drift bound is not met

* **Problem.** The closing brief asked for a P3 experiment with three conditions:
  * a current that stays inside the envelope (drift <= 0.6 m/s horizontal, <= 0.25 m/s vertical, all
    other assumptions valid);
  * the formation truly lost (true error > e_lost = 1.2 m);
  * the formation then recovered.
* **What was measured.** One run each, square formation, a Gaussian jet of radius 3 m, no tuning
  between probes (`results/v2/design_probes.json`).

  | probe | current | max true formation error | onboard | self-declared envelope violations |
  |---|---|---|---|---|
  | lateral jet on the left lane | 0.6 m/s, t = 12-24 s | 0.47 m | onboard error <= 0.94 m | 0 |
  | lateral jet reversing | +0.6 then -0.6 m/s, 6 s each | 0.44 m | <= 0.85 m | 0 |
  | head-on jet on the front-left drone | 0.6 m/s against the motion | 0.74 m | 1.40 m: FOLLOW -> RECOVERY -> FOLLOW | 0 |
  | head-on jet on the rear-left drone | 0.6 m/s against the motion | 1.83 m: lost, recovered in 8 s | 2.60 m | 1 (after 7 s, at 0.76 m true error) |

* **Analysis.**
  1. **Lateral and following currents up to 0.6 m/s are rejected.**
     * The DVL velocity loop rejects about 55 % of the current at once; the integrator rejects the
       rest within about 2.3 s.
     * With k_slot = 0.45 the transient slot error stays near 0.6 m.
     * A single displaced drone counts 0.75 x in the translation-invariant error.
  2. **Against the motion, the authority is the limit.**
     * Under a 0.59 m/s head drift, with the nominal authority saturated, the drone made 0.12 m/s over
       ground. That is about 0.71 m/s through the water, not the 0.96 m/s that the small-signal slope
       of 2.4 m/s per unit command predicts.
     * At the 0.30 m/s survey speed a drone therefore follows its slot only against head currents up
       to about 0.4 m/s (0.71 - 0.30).
     * Beyond that it saturates persistently. Its own EnvelopeMonitor declares ENVELOPE_VIOLATION
       after 4 s, and FAILSAFE holds position.
  3. **A formation loss requires persistent saturation.**
     * Persistent saturation violates the P1 assumption w_drift_max (unrejected drift <= 0.15 m/s) and
       P3's A3 (no FAILSAFE). Inside every assumption the formation is not lost.
     * With an unrejected drift <= 0.15 m/s, the slot loop bounds the steady error near
       0.15 / 0.45 = 0.33 m, plus the integrator transient.
  4. **Front-drone case.**
     * The front drone was pushed back towards its rear neighbour, and the warning filter moved that
       neighbour back too, so the lane moved as a block.
     * The onboard guard, which sees the absolute slot error of 1.4 m, went FOLLOW -> RECOVERY ->
       FOLLOW.
     * The true translation-invariant error stayed at 0.74 m.
* **Decision.**
  * No scenario is labelled "in envelope" for a loss it cannot produce. The P3 scenario is
    `formation_recovery_head_current`: the principled worst case at the claimed drift limit.
    * Direction: against the motion, the one with the smallest margin.
    * Intensity: the claimed 0.6 m/s.
    * Shape: a localized jet, since a uniform current only translates the formation.
    * Target: the rear drone, so that the warning filter does not move a neighbour with it.
    * Timing: 12 s, about five integrator time constants, with formation_gust's ramps.
  * Its outcome is reported together with the vehicle's own verdict.
  * No parameter was changed: authority, gains and envelope numbers are as before.
* **Consequence for the claims.**
  * The 0.6 m/s drift bound holds for lateral and following currents and for station keeping.
  * Against the motion, at survey speed, the effective bound is about 0.4 m/s.
  * This is now a documented limit (REPORT section 6). ASSUMPTIONS.md reports the vehicle's own view
    (E2) next to the referee's drift (E1).

## DI-25 - A right-angle crossing is not a head-on encounter

* **Problem.** The P1 close-encounter experiment had to meet four conditions:
  * fleet drones only, with no scripted vehicle;
  * the traffic rule and the whole safety layer on;
  * an inevitable conflict;
  * the drones inside the warning band, and collision avoidance only if it happens naturally.
* **Analysis.**
  * Head-on encounters are removed by the traffic rule before d_warning (`p1_head_on`,
    `p1_two_lines`).
  * A right-angle crossing at survey speed closes at 0.30 x sqrt(2) = 0.42 m/s. That is below the
    rule's 0.45 m/s threshold for a FRONT target; the noisy closing estimate occasionally exceeds it.
  * Even then, giving way to the right does not resolve a crossing: both drones shift right and the
    conflict point moves with them.
* **Probes** (`results/v2/design_probes.json`).
  * Two drones crossing at 90 deg, arriving together: both entered SEPARATION_WARNING twice. The
    onboard bound reached 1.92 m, the true minimum was 2.89 m, and there was no collision avoidance.
  * A third drone crossing through the 3.5 m gap of a two-drone line: repeated SEPARATION_WARNING,
    true minimum 2.82 m, no collision avoidance, not resolved within 40 s. Less readable, so not used.
* **Decision.** `p1_close_encounter` is the two-drone crossing.
  * Collision avoidance is not forced and no parameter was changed. The warning filter, which demands
    an opening of 0.2 m/s, stops the closing within about 0.2 m of the trigger.
  * The conservative bound triggers SEPARATION_WARNING at a true distance of about 2.9 m. For the
    FRONT+LEFT pattern the bound's looseness is 0.5-0.9 m.
* **Result.** README table and `figures/v2/p1/p1_close_encounter.png`.

## DI-26 - Observation consistency between perception and automaton

* **Problem.** The guards assume that their abstract observation was produced by the perception code.
  Nothing checked this, neither at runtime nor in the proofs' domain. DI-22 and DI-23 are two such
  inconsistencies.
* **Fix.**
  * **Invariants.** `holo_fleet/ha/observation_invariants.py` defines:
    * N0: well-formed values;
    * N1: ranges;
    * I1: has_prio -> at_queue;
    * I2: at_queue -> gate_zone;
    * I3: passed -> !gate_zone;
    * I4: t_ok > 0 -> (form_err < e_ok & neighbors_ok).

    Each invariant documents its meaning, why the code guarantees it, and which states it excludes.
    Latched beliefs (occ_busy, committed) are deliberately not constrained.
  * **Runtime check.** It runs in the controller between perception and automaton. A violation is
    logged as OBSERVATION_INCONSISTENT (drone, time, violated invariants, observation values). The
    observation then goes to the automaton with sense_ok = False, so the existing fault edge leads to
    FAILSAFE_HOLD_OR_RETREAT; no new mode is added.
  * **Reporting.** The counters appear in the run metrics, the printed summary and the dashboard.
  * **Formal checks** (`formal/check_observations.py`, 27 checks):
    * O1-O3: satisfiability, independence, non-vacuity;
    * O4: exactly one edge enabled;
    * O5: the monitored observation leads to FAILSAFE;
    * Q1: queued drones stay in the gate protocol;
    * mutations Om1-Om3.
* **Why the fix is principled.** The invariants are the perception code's own guarantees. They are
  written once and used both as the proof domain and as a runtime monitor. A violation is a perception
  defect, which is exactly what a sensing fault means.
* **Result.** 0 violations in every final run; the formal suite gives every check its expected
  verdict.
