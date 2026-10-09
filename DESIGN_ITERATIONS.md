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
  * Corrected in DI-27: the scalar bound is replaced by the control-feasible envelope. By that envelope this
    run is OUTSIDE, which is what the hit drone declared.

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

## DI-27 - The current envelope was a scalar bound; what the drone can do depends on the direction and on the requested speed

* **Problem.** The v2 baseline declared one isotropic bound, |w| <= 0.6 m/s (`current_drift_max`), and called
  "inside the envelope" every run whose current stayed below it. `formation_recovery_head_current` (DI-24)
  showed that this was false. A 0.6 m/s current against the motion was "inside", yet the hit drone could not
  hold its slot and declared ENVELOPE_VIOLATION itself.
* **What the code did** (checked before changing anything).
  * The onboard `env_ok` never looked at |w|. EnvelopeMonitor declared a violation after 4 s (leaky) of
    saturated horizontal command outside avoidance manoeuvres. `env_ok` False then takes the fault edge to
    FAILSAFE.
  * The value 0.6 was used only on the judging side:
    * the referee's `inside_envelope`;
    * the live dashboard label;
    * the head-current scenario;
    * the assumption script, the tests and the documents.
  * The formal models never used the raw current. P1 uses the residual drifts w_rel and w_drift (0.15 m/s);
    P3 uses the residual w_res (0.05 m/s).
  * The low level caps the norm of the normalised (surge, sway) command at AUTHORITY: 0.40 nominal, 0.50
    brake (also FAILSAFE), 0.60 escape. Its feed-forward uses the small-signal slope of 2.4 m/s per unit of
    command, and its current estimate is its own integral action expressed with that slope.
* **Calibration.** `probe/probe_head_current_authority.py` runs one 32 s session: seven BlueROV2s, no
  sonars, the deployed low level, a requested velocity of 0.30 m/s
  (`results/calibration/head_current_authority.json`).

  | current | ground speed [m/s] | saturated steps | onboard monitor | through-water speed [m/s] |
  |---|---|---|---|---|
  | head 0.30 | 0.300 | 0 % | ok | 0.60 |
  | head 0.35 | 0.301 | 0 % | ok | 0.65 |
  | head 0.40 | 0.297 | 13 % (0.2 s at most) | ok | 0.70 |
  | head 0.45 | 0.263 | 91 % | violation at 16 s | 0.71 |
  | head 0.50 | 0.211 | 99 % | violation | 0.71 |
  | head 0.60 | 0.102 | 100 % | violation | 0.70 |
  | lateral 0.60 | 0.301 (lateral residual 0.033) | 0 % | ok | 0.64 |

  * At the nominal authority the BlueROV2 makes 0.70-0.71 m/s through the water, not the 0.96 m/s that the
    slope of 2.4 would give.
  * At 0.30 m/s it holds head currents up to 0.40 m/s and loses ground from 0.45 m/s.
  * A lateral 0.60 m/s current is held without saturation, as in the earlier station-keeping calibration.
  * Per axis, the steady command for a through-water speed r is g(r) = |r|/2.4 + 0.203 r^2. This is fitted on
    these points only; the largest residual is 5.3 %, so the model tolerance is 6 %.
* **Fix: one shared definition** (`holo_fleet/control/current_envelope.py`).
  * **Definition.** A current w is inside the envelope for a requested velocity v when three conditions hold:
    * the steady command |(g(r_surge), g(r_sway))| for the through-water velocity r = v - w, on the drone's
      body axes, stays within the AUTHORITY of the mode;
    * |w_h| <= 0.6 m/s, the exercised range (renamed `current_validated_max`; it is not a controllability
      bound);
    * |w_z| <= 0.25 m/s.

    The vertical channel is separate and not binding.
  * **Derived limits.**
    * At 0.30 m/s: head current <= 0.41 m/s, lateral <= 0.60 m/s (physics alone allows 0.67 m/s).
    * At 0.50 m/s (recovery catch-up, gate passage): head <= 0.21 m/s, lateral <= 0.57 m/s.
    * A diagonal current of 0.59 m/s is inside when it follows the motion and outside when it opposes it.
  * **Onboard (EnvelopeMonitor).** The same authority test runs on the steady command of the implemented
    loop, |v_d - w_est| / 2.4, with saturation kept as direct evidence (the integrator freezes while
    saturated). The persistence rule is unchanged (4 s).
    * Logged every step: current estimate, requested velocity, head, lateral and vertical components,
      through-water speed, required command, authority, reason.
    * ENVELOPE_VIOLATION and ENVELOPE_OK events carry the same fields.
    * Replayed on the logs of the eleven final runs, the new monitor gives exactly the logged `env_ok`
      sequence on every drone, so the runs were re-evaluated, not regenerated.
  * **Judging** (`holo_fleet/referee/envelope.py`, called at the end of every run and by
    `scripts/run_all_demos.py` on logged runs). It applies the same test to the true current, with the plant
    curve, the logged requested velocity and heading, the authority of the mode and the same persistence.
    * Verdict INSIDE, LIMIT (beyond the authority only within the model tolerance) or OUTSIDE.
    * The vehicles' own declarations are reported next to it.
  * **Formal.** The ranking proof of P3 works with the residual after compensation, which exists only while
    the commanded velocities are deliverable. A raw current is therefore admissible for P3 when the recovery
    speed V = 0.5 m/s stays deliverable: the envelope evaluated at V, head current <= 0.21 m/s. New checks:
    * F5 (UNSAT): at V the catch-up margin of F1 remains;
    * Fm3 (SAT): with the envelope at the survey speed only, the margin can vanish;
    * Em1 (SAT): the old scalar bound admits head currents with which not even the survey speed is
      deliverable.
* **Why the fix is principled.**
  * The envelope is the controller's own feasibility condition: a norm bound on its commands, mapped through
    a calibrated plant curve.
  * One definition, with the same parameters, serves the runtime, the judging, the tests and the proofs.
  * No gain, authority or threshold was changed.
* **Result** (re-evaluation of the logged runs, `results/v2/ASSUMPTIONS.md`).
  * `formation_triangle`, `formation_square`, `formation_six`, `gate_single` and the P1 runs: INSIDE, with no
    violation declared.
  * `formation_recovery_head_current`: OUTSIDE (1.52 times the authority, head current 0.59 m/s). The hit
    drone declares it: the two views now agree, where before the run was "inside 0.6" yet in violation.
  * `formation_gust`: OUTSIDE (beyond the range and 2.06 times the authority); the drone declares it.
  * `integrated_short`: OUTSIDE at the margin. The old scalar bound had called this run inside.
    * Drone 0's diagonal gate passage at 0.5 m/s against the 0.30 m/s cross-current requires up to 1.12 times
      the authority. It was above authority plus tolerance for 4.9 s.
    * The drone delivered its maximum: ratio 1.00 on the achieved velocity, with a lag of up to 0.07 m/s.
    * Its command stayed at the edge without persistent saturation, so its own monitor did not declare it.
    * A monitor that also flagged such lags would put a committed drone in FAILSAFE during a gate passage.
      That was not done (no controller change); the case is documented as a limit.
  * Inside the corrected envelope no run loses its formation, consistent with DI-24.

## DI-28 - The gate protocol is a mutual-exclusion zone; precedence left first, then top first; a latent static-rank deadlock

* **Naming.** What the drones coordinate is a mutual-exclusion zone (the critical region of a narrow gate),
  so the three gate modes are now MUTEX_APPROACH, MUTEX_YIELD and MUTEX_PASS, the observation flag is
  `mutex_zone` and the rule module is `ha/mutex_rule.py`.  Mechanical rename, no behaviour change.
* **Problem 1: a latent deadlock.**  The v2 rule mapped a neighbour seen only in UP or DOWN (a vertically
  stacked neighbour) to RANK and resolved RANK with the static rank... of an unknown neighbour: the
  runtime passed `their_ranks = [-1, ...]`, so a drone that saw a stacked neighbour always waited.  Two
  drones stacked in the same queue column both waited, and every drone on their right waited for them:
  no drone could ever commit.  No v2 scenario had a stacked queue, so it never showed; formal check M2m
  (the v2 rule as a mutation) finds it: no leader in 5 of the 15 occupancies of the 2 x 2 stacked queue, and in
  2 of the 7 of the 3-drone stacked queue of the demo (whenever the stacked pair is still queued).
* **Fix: one visible precedence, from sector patterns.**
  * LEFT first: a neighbour with FRONT or LEFT in its pattern -> WAIT; with REAR and/or RIGHT -> PRIORITY.
  * then TOP first: a neighbour seen only in UP -> WAIT, only in DOWN -> PRIORITY (no horizontal sector).
  * static rank only as the last tie-break, for a pattern that cannot be ordered geometrically (UP and DOWN
    at once: one target associated across two drones).  The static rank is now the queue precedence
    order of the shared plan (`mission.queue_order`: left first, then top first), and a drone compares it
    with the plan's drones whose queue points lie within its bracket: it can never contradict the
    geometric order.  Used 0 times in every run.
* **Problem 2: queue geometry for stacked slots.**  A template with depth layers (two drones on the same
  lane, one above the other) needs stacked queue points.  `mission.queue_assignment` stacks them
  (`stack_spacing` 3.5 m, top first) in one column; the columns are then `stack_column_spacing` 5.0 m
  apart.  Why 5.0 m: with 3.5 m columns the diagonal pair (left-bottom / right-top) is seen in LEFT/RIGHT
  or in UP/DOWN depending on a few tenths of a metre of holding error (numeric check M2v over the tolerance
  box: ambiguous); from 4.5 m it is always horizontal, so "left first" decides it.  Mutation M2vm keeps
  the 3.5 m spacing and must show the ambiguity.
* **Problem 3 (probe run): a neighbour hidden by the gate frame.**  The first probe of the stacked queue
  passed in the order left-top, right, left-bottom: the diagonal neighbour's LEFT/RIGHT echo fell inside
  the gate's structure window (the posts lie inside the side cones of the queue points) and was classified
  STRUCTURE, so only its UP/DOWN echo remained and the vertical relation decided alone.  Safe by luck (both
  sides were hidden symmetrically).  Fix: the queue line of a stacked queue is moved back until no pair
  within the bracket is hidden (`mission.structure_masked_pairs`, the classifier's own window and
  tolerances, the measured holding error): check M4f, mutation M4fm.  For abreast queues the adjacent
  pairs are never hidden (M4f); the pair around ONE vacant point of a 3-drone queue is hidden at G06 and
  is declared out of scope rather than moving the line back: a second probe showed that a line 1.7 m
  further back puts the passing drone's exit beyond the range at which the outer queued drone detects a
  hull (about 8.5 m), and the occupancy latch then frees only after its 25 s timeout.
* **The pass path of a stacked drone** keeps its depth along its queue lane and descends on the diagonal
  to the merge point, so it never closes on the drone below it; beyond the critical region it goes to its
  own slot depth.  M4s checks merge_clearance on this 3-D path.
* **Progress / no deadlock (M2, M5).**  For every set of occupied queue points with at most one vacant point
  between neighbours (degraded formations, drones already through), exactly the first occupied point in
  the order has PRIORITY: Z3 over the bearing model for abreast queues of 2..6 (96 occupancies), numeric
  over the tolerance box for the stacked queues (15 + 7).  With the occupancy latch (M3) and the commit
  edge, every queued drone eventually commits, under the fairness assumptions "a committed drone
  completes its passage" and "every drone in the approach zone reaches its queue point or leaves".  Not
  covered: a drone that stops for good inside a queued drone's bracket, on its left (it is never seen to
  leave); scenario `lost_drone_mutex` keeps the stopped drone more than a bracket behind the queue.
* **Result.** (one seed each, `results/v2/demos/SUMMARY.md`)
  * `mutex_deadlock_resolution`: entry order left-top, left-bottom, right; occupancy <= 1; static rank 0;
    every drone back in FOLLOW beyond the gate.  A third probe found that, seen from the right drone, the
    stacked pair is one echo in LEFT at one range: the pair is now confirmed as a group (DI-29).
  * `line_parallel_mutex`: left to right, occupancy <= 1, static rank 0.
  * `lost_drone_mutex`: the leftmost drone present goes first; the stopped drone stays outside the bracket.
  * `gate_single`, `integrated_short`: unchanged order and occupancy, no latch timeout.
  * ASSUMPTIONS A1 now separates the echoes masked by the gate frame: 12 control steps in
    `mutex_deadlock_resolution` (a passing drone near the frame, a queued neighbour 3.7-5 m away), all
    explained by the classifier's structure window; true minimum distance 3.42 m.
  * Formal: P2 suite 27/27 as expected (M2 for 96 occupancies, M2s, M2v, M2m, M2vm, M4f, M4fm, M4s, M3 n = 2..4).

## DI-29 - Lost drones: wait for a missing neighbour, declare its slot vacant after t_rejoin, keep a degraded formation

* **Problem.** In v2 a missing neighbour (`neighbors_ok` false) sent a drone to FORMATION_RECOVERY and kept it
  there until the neighbour came back: a drone that never came back left the whole fleet "recovering" for
  ever (mutation Fm4 reproduces it), and the recovery flow (catch-up speed) was applied to drones whose
  own slot was fine.  Nothing distinguished a temporary loss from a permanent one, and nothing said where a
  lost drone should rejoin.
* **Fix: separate what a drone can fix from what it cannot** (no communication: only own sensors and the
  shared plan).
  * Own slot error > e_lost -> FORMATION_RECOVERY (exit when the own error is < e_ok for t_ok_hold; t_ok
    now counts the own error only, invariant I4 updated).
  * Own slot ok, an expected neighbour missing -> FORMATION_WAIT_REJOIN (new mode): keep the slot, follow
    the shared clock, time the absence.
  * Missing for t_rejoin -> the slot is declared VACANT (a latch) -> DEGRADED_FORMATION (new mode): the
    vacant slot is no longer required, the drone keeps its ORIGINAL slot, the hole stays (no
    reconfiguration).  The slot is re-included as soon as its drone is confirmed there again (1 s).
  * WAIT_REJOIN and DEGRADED use the follow flow and sit at the follow level of the hierarchy:
    FAILSAFE > COLLISION_AVOIDANCE > SEPARATION_WARNING > MUTEX > FORMATION_RECOVERY > formation keeping.
* **t_rejoin = neighbour_range_m / (v_recovery_max - v_nominal) = 7.5 / 0.2 = 37.5 s**: the time a lost drone
  still at the edge of the sensing range needs to close the gap with the catch-up margin.  Not tuned.
* **What the others do while waiting: nothing different.**  They keep their slots on the shared clock, at the
  survey speed.  Slowing down was rejected: without communication it cannot be applied consistently (a
  drone that does not expect the missing neighbour within its sensing range would not slow down and the
  formation would split).  The lost drone catches up with its 0.2 m/s margin.
* **The timers run only when they mean something**: while the drone itself holds its slot (FOLLOW, WAIT,
  DEGRADED; not while lost, avoiding or in a gate) and while the shared clock moves (a planned hold such
  as the rendezvous beyond a gate is not an absence).
* **Probe run 1: spurious "back".**  An echo of another neighbour (two neighbours merged in one target, a
  second echo 1.1 m from the expected range of the missing diagonal neighbour) fitted the wide range gate
  every 1-2 s, reset the timer and made a drone flicker between FOLLOW and WAIT_REJOIN.  Fix: a missing
  neighbour counts as back only when confirmed near its slot - three associations within 1 s, each within
  half the range gate.  A real neighbour gives ten per second; the spurious echo never qualified again.
* **Rejoin from behind.**  A recovering drone off its own lane does not head straight for its slot, which
  could cut across the lanes of the others: it lets the fleet pass (DROP_BACK: slower than the clock, never
  backwards, no lateral motion), moves to its own lane 2 m behind the rear-most slot (TO_LANE), then
  advances along its lane to the slot.  Shared plan and own navigation only.
* **Probe run 2: a dead drone is indistinguishable from a healthy one holding still.**  The faulty drone
  (thrusters zeroed in the simulator, no controller told) declares ENVELOPE_VIOLATION from persistent
  saturation and goes FAILSAFE; there, holding still and requesting almost nothing, it is neither saturated
  nor infeasible, so the violation cleared after 4 s, the drone tried to recover, saturated again: it
  cycled FAILSAFE / RECOVERY every 8 s.  Fix: while a violation is declared the FAILSAFE flow adds a small
  heave probe (+-0.3 m, period 8 s), and the violation clears only after 4 s in which the vehicle delivered
  the motion it requested - projection gain of the DVL velocity on the request over 4 s (half the probe
  period, so a lagging but healthy response still gives cos(lag)) at least 0.5.  A dead vehicle gives
  about 0 and stays in FAILSAFE; the temporarily failed one clears about 6 s after its thrusters return.
  The persistence accumulator is also emptied when a violation clears (otherwise a violation longer than
  t_enter + t_exit was re-declared at the next step).
* **Fault injection** is simulator-side only (`SimSpec.faults`: thruster commands zeroed between t_on and
  t_off, or for good), logged as FAULT_INJECTED / FAULT_CLEARED events, never visible to a controller.
  The envelope evaluation does not count the faulty drone's own declarations against the current envelope.
* **Referee (P3-deg).**  A drone farther than e_lost from its slot (robust median fit of the others) for
  t_rejoin of mission time is ABSENT; the formation of the present drones on their original slots is then
  judged with the P3 episode logic.  Ground truth, independent of the onboard timers.
* **Formal.**  F4 now covers the new edges (own slot lost -> RECOVERY from every formation-keeping mode,
  neighbour missing -> WAIT_REJOIN, recovered -> the mode selected by the neighbour knowledge).  F6: the
  missing timer is a ranking function (t_rejoin - tau decreases by dt per running step), and a BMC over
  376 steps shows that a drone waiting for a neighbour that never reappears leaves WAIT_REJOIN; Fm4: without
  the timeout it may wait for ever.  Determinism, completeness, observation consistency (O1-O5, Q1) cover
  the two new modes and the new variable `degraded` automatically.
* **Result.** (one seed each)
  * `lost_drone_rejoin`: fault from 8.1 to 15.1 s; ENVELOPE_VIOLATION at 13.3 s; FAILSAFE until the probe is
    delivered (ENVELOPE_OK at 21.5 s); TO_LANE then along its lane; FOLLOW at 44.4 s; the others
    FORMATION_WAIT_REJOIN from 12.7-13.3 s to 33.9-40.8 s; no vacancy; P3 recovered in 27.3 s.
  * `lost_drone_timeout`: missing from 14.1 / 18.7 / 21.1 s, vacant at 51.5 / 56.1 / 58.5 s (37.5 s each),
    DEGRADED_FORMATION with a formation error of the three of 0.06 m; referee: absent at 50.8 s, P3-deg PASS.
  * `formation_recovery_head_current`, `formation_gust`: as before, with WAIT_REJOIN for the neighbours of the
    hit drone instead of RECOVERY, and a FAILSAFE about 2 s longer (the probe).  Formation scenarios: every
    drone in FOLLOW for the whole run (no false "missing").
  * Probe run 3: in `lost_drone_rejoin` a closing-rate spike of a re-acquired echo triggered the head-on
    give-way while the drone was holding in FAILSAFE, and the 2 m offset displaced its target during the
    rejoin.  Mission-level rules are now suspended in FAILSAFE (a running give-way is cancelled).
  * Probe run 4: seen from the side, the two drones of a stacked pair give one echo at one range in one sector;
    one association could confirm only one of them, so the other was missing for ever (a false vacancy after
    t_rejoin).  Neighbours that a range-only sonar cannot separate (same expected sectors, ranges within the
    range gate) are now confirmed as a group.
  * Formal: P3 suite 22/22 (F4 for every formation-keeping mode, F6, F6r, Fm4); determinism 82/82 and
    observation consistency 31/31 with the two new modes and `degraded`.

## DI-30 - A fleet view instead of the onboard-vs-referee dashboard

* **Problem.** The v2 dashboard showed everything each drone knew (six sectors, ranges, classes, ages,
  distance bounds, current estimate...) next to the referee: right for debugging, unreadable for a
  supervisor, and it did not show the fleet as a fleet (slots, missing drones, the queue order).
* **Fix.** `holo_fleet/ui/fleet_view.py`, one picture of the whole fleet:
  * top view: common path, slots of the shared plan (a vacant slot marked), drones coloured by automaton
    state with a short trail, missing / assumed-failed markers, the gate with its critical region and the
    queue points numbered in precedence order; a side view (along-track vs depth) when the formation has
    depth layers;
  * fleet panel: P1 / P2 / P3 one line each, formation error, occupancy, queue progress (or deadlock risk:
    nobody has priority for more than 5 s), drones active / missing / assumed failed;
  * one row per drone (state, ok / missing / rejoining / assumed failed, nearest distance only in the
    warning band), readable events, and a timeline of the automaton states.
  * Positions are ground truth (for the viewer); states, missing and vacant slots, queue decisions are the
    drones' own logs.  The same class renders live and from the logs (`scripts/render_demo.py`).
  * The technical dashboard is kept only for the sonar-classification bench.
