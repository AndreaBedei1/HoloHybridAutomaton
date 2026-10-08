# HoloOcean 2.3.0 sonar-octree patch

A minimal patch to HoloOcean 2.3.0 that lets every sonar see the scene that is really there:
runtime-spawned props, other agents, and nothing left over from previous scenes.  It is used by
this project through a separate HoloOcean root; the official installation is never modified.

| | |
|---|---|
| patch file | `patches/holoocean-2.3.0-sonar-octree-rebuild.patch` (12 files: engine C++ + Python client) |
| source checkout | `F:\Andrea\holoocean-octree-patch`, branch `sonar-octree-rebuild` on top of `pristine-2.3.0` (4 commits) |
| build / install | `patches/build_and_install_patched_holoocean.ps1` (UE 5.3.2, Win64 Development) |
| installed root | `F:\Andrea\holoocean_patched_root` (`HOLODECKPATH`), `PATCH_INFO.json` with source commit and exe SHA-256 |
| regression | `probe/octree_rebuild_regression.py` -> `results/v2/octree_patch/regression.json` (pytest: `tests/test_octree_patch.py`, marker `holoocean`) |
| status | not submitted upstream (an upstream PR is left to the author) |

## 1. Problem

The four problems below share one root: a sonar trusts a static octree that does not describe the
current scene.

1. **Runtime props are invisible.** Gates, boxes and other props spawned after `env.reset()` are
   missing from every sonar.  Example: the Marine Race Arena gates, 0 % detection at 3 m.
2. **Ghosts.** The octree cache is keyed only by map name and octree size.  A scene with props leaves
   cells on disk, and a later scene without them still echoes the old gate and box, at the old
   places.
3. **Engine crash after a reset.** A sonar initialised after a `reset()` that removed a
   runtime-spawned agent crashes the engine (`EXCEPTION_ACCESS_VIOLATION` in
   `AHolodeckBuoyantAgent::makeOctree`).  The Python client then waits forever.
4. **Order-dependent agent invisibility.** With several sonar-equipped agents, the octree of every
   agent built after the first sonar initialised is empty.  Those agents are invisible to all
   sonars.  Example: in a head-on pair one drone saw the other, the other did not.

A fifth, smaller defect was found by the regression: **axis-aligned faces lying exactly on an octree
cell boundary are lost.**  Example: a box face at x = 324 cm with 6 cm leaves.

## 2. Reproduction (unpatched 2.3.0)

`probe/octree_debug_box.py` and `probe/sonar_probe_lib.py` (sessions k1-k3,
`docs/v2/SONAR_PROBE.md`):

* **k1:** reset, spawn the arena gate G06 and a 0.5 m steel box 3 m ahead of a SinglebeamSonar.
  The gate and the box are never echoed; a BlueROV2 at the same distance is.
* **k3:** after a k2 run that spawned the props before the sonars, start a scene with nothing
  spawned.  The sonar echoes the old gate pillar at 1.875 / 2.875 m and the old box at
  2.025 / 3.025 m.
* **Crash:** spawn an agent at runtime, `reset()`, then initialise a sonar.
* **Invisible agent:** two BlueROV2 with six sonars each; one of them never appears in the other's
  profiles.

## 3. Cause (engine sources)

* `UHolodeckSonar::initOctree` builds the world octree with `ECC_WorldStatic` overlap tests the first
  time a sonar initialises.  That happens inside `reset()`, before any prop can be spawned.
  * `Octree::makeOctree` caches it under `Octrees/<map>/min<cm>_max<cm>/` (`roots.json` plus one JSON
    per 7.68 m cell).
  * Nothing ever invalidates the cache or the in-memory octree.
* Agents add themselves to the server's `AgentMap` (a `TMap` of raw pointers) and are never removed.
  * The sonar loop `static_cast`s every entry, so after a reset it dereferences a destroyed agent.
* Agent octrees are built with the *shared* query `Octree::params`.
  * The first sonar adds every agent to that query's ignore list, so the world octree contains no
    agent.
  * Any agent octree built afterwards is swept with a query that ignores that very agent, and comes
    out empty.
* The occupancy test treats a cell that only touches a surface as empty.
  * A face lying exactly on a cell boundary therefore leaves no surface leaf.

## 4. Solution (4 commits)

| commit | change | files |
|---|---|---|
| 16e2571 | `RebuildSonarOctree` command, `Octree::invalidateCache(region)`, `UHolodeckSonar::ResetOctree()`, client `env.rebuild_sonar_octree(region=None)` | `ClientCommands/{Public,Private}/RebuildSonarOctreeCommand.*`, `CommandFactory.*`, `General/{Public,Private}/Octree.*`, `HolodeckSonar.*`, `client/.../command.py`, `environments.py` |
| b7bcb31 | agents leave `AgentMap` in `EndPlay`; sonar loops use `Cast<>` + `IsValid()` | `HolodeckAgent.*`, `HolodeckSonar.cpp` |
| 1934564 | occupancy test with a 5 mm margin (`OccupancyMargin`) | `Octree.*` |
| a8d3070 | agent octrees built with a clean collision query (the actor-name filter already restricts hits to the agent) | `Octree.cpp` |

**Rebuild flow.**
1. The client calls `env.rebuild_sonar_octree()` or passes a region.
2. The command converts the region to engine units.
3. It deletes the cache of the current level and octree size: the whole folder, or only `roots.json`
   plus the cells that intersect the region.
4. It calls `ResetOctree()` on every sonar in the world, which clears the leaves and the octree and
   resets the tick counter.
5. On their next ticks the sonars rebuild from the scene that is there now.

`rebuild_sonar_octree()` returns the number of ticks to let pass before the new octree is used.
That is 8 + 2 x the slowest sonar's tick period + 1.

Caches of other levels or octree sizes are never touched.  The patch does not use
`delete_world_octrees`, which would wipe other projects' caches.

## 5. How it is used here

`holo_fleet/sim/holo_env.py` does the following:
1. Spawns the static scene (the arena gate).
2. Calls `env.rebuild_sonar_octree()` once.
3. Waits the returned number of ticks before the first control step.

`holo_fleet/sim/holoocean_setup.py` selects the patched root:
* `HOLODECKPATH` points to `F:\Andrea\holoocean_patched_root`;
* the patched client is installed in the project environment
  (`pip install -e F:\Andrea\holoocean-octree-patch\client --no-deps`).

If the patch is missing, it falls back with an explicit warning.  `holo_fleet/sim/engine_watchdog.py`
ends the Python process with a clear message if the engine dies instead of hanging.

## 6. Regression test

`python probe/octree_rebuild_regression.py`, about 1 minute, all checks must be true:

* **Hard case:** the sonar is created first, then a box, the arena gate and a late BlueROV2 are spawned.
  * All three are invisible before the rebuild.
  * After a full rebuild all three are visible, and the initial agent is still visible.
* **Region rebuild:** a second box appears after rebuilding only its region, and the first box stays.
* **Reset without spawning:** the ghost box is present before the rebuild and absent after it.
  The gate and the second box also leave no ghost.
* **Moved box:** it is seen at its new place, and the old place is silent.
* **No crash:** a sonar initialised after a reset that removed a runtime agent does not crash the
  engine.
* **Official cache:** the official installation's cache is untouched.

## 7. Benchmark

128 x 128 x 40 m environment box, 6 cm leaves, `results/v2/octree_patch/regression.json`:

| operation | ticks | wall time |
|---|---|---|
| initial rebuild | 15 | 0.21 s |
| full rebuild | 15 | 0.29 s |
| region rebuild | 15 | 0.42 s |
| rebuild with a late agent | 15 | 0.45 s |
| rebuild after reset | 15 | 0.21 s |

A rebuild is a one-off cost after a scene change.  Steady-state cost with six sonars per drone is in
`README.md` (performance) and `results/v2/sonar_bench/perf.json`.

## 8. Compatibility

* HoloOcean 2.3.0, Unreal Engine 5.3.2, Windows (Win64 Development build).  The C++ changes are
  platform-independent.
* No change to any sensor model, parameter or message format.
* The new client command is opt-in: scripts that never call it behave as before, except that agents
  are now visible to sonars regardless of initialisation order, and touching faces are kept.
* The patched engine and its private cache live in their own root.  `%LOCALAPPDATA%\holoocean` and
  the Marine Race Arena's frozen benchmark keep using the official engine.

## 9. Rollback

* **Project.** Unset `HOLODECKPATH` (or delete `F:\Andrea\holoocean_patched_root`) and reinstall the
  official client with `pip install holoocean==2.3.0`.  The official installation was never modified.
  Without the patch, runtime props are invisible to the sonars, so gate scenarios lose their
  structure echoes; the wrapper warns about this.
* **Patch checkout.** `git -C F:\Andrea\holoocean-octree-patch checkout pristine-2.3.0`.

## 10. Applying the patch to a fresh HoloOcean 2.3.0 checkout

```bash
git apply --check patches/holoocean-2.3.0-sonar-octree-rebuild.patch
git apply patches/holoocean-2.3.0-sonar-octree-rebuild.patch
```

```powershell
powershell -ExecutionPolicy Bypass -File patches/build_and_install_patched_holoocean.ps1
```
