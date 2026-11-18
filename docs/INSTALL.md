# Installation

Windows 10/11, HoloOcean 2.3.0 with the sonar octree patch, Python 3.9 or later in a conda environment.

## 1. Python environment

HoloOcean's client is installed in a conda environment (`ocean` below); the project environment is a clone of it.

```bash
conda create -n holo_fleet_ha --clone ocean -y
conda activate holo_fleet_ha
pip install -e .
```

`pip install -e .` installs `holo_fleet` and its dependencies (numpy, scipy, matplotlib, opencv-python, z3-solver,
imageio, psutil).

## 2. Patched HoloOcean engine

Unpatched HoloOcean 2.3.0 sonars do not see props spawned at run time (the gates) and can see ghosts of earlier scenes.
The project therefore runs on a patched engine installed in its own root; the official installation is not modified.
The cause, the fix and its regression test are described in [`HOLOOCEAN_OCTREE_PATCH.md`](HOLOOCEAN_OCTREE_PATCH.md).

```bash
pip install -e F:\Andrea\holoocean-octree-patch\client --no-deps                            # patched client
powershell -ExecutionPolicy Bypass -File patches/build_and_install_patched_holoocean.ps1   # patched engine (UE 5.3.2)
```

The patch file is `patches/holoocean-2.3.0-sonar-octree-rebuild.patch`. The engine root defaults to
`F:\Andrea\holoocean_patched_root`; set `HOLO_FLEET_HOLOOCEAN_ROOT` to use another one. The campaign runner refuses
to run without the patched root.

## 3. Gate arena

The gate scenarios import gate G06 of the Marine Race Arena from `~/Desktop/HoloDroneCompetition`; set
`MARINE_RACE_ARENA_ROOT` to use another location.

## 4. Several engines at once (optional)

Each concurrent HoloOcean instance needs a private engine root with its own writable files (octree cache, saved
state); the large engine and content directories are shared by junctions:

```powershell
powershell -ExecutionPolicy Bypass -Command "& ./scripts/prepare_engine_roots.ps1 -Slots 0,1,2,3"
```

Use `-Command "& ..."`: with `-File` the list `0,1,2,3` arrives as one string. The script refuses to overwrite an
existing slot root.

Then set `HOLO_FLEET_ENGINE_SLOTS` (1 to 4). `python scripts/verify_engine_isolation.py` checks that two instances do
not share writable state.

## 5. Check the installation

```bash
python -m pytest -q -m "not slow and not holoocean"   # fast tests, no engine
python scripts/run_demo.py --scenario gate_single     # opens HoloOcean and the Fleet View
```
