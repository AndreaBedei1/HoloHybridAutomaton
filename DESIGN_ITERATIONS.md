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
