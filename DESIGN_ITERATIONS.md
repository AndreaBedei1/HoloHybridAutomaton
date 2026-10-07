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
