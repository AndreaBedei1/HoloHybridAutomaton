"""Which HoloOcean installation the project runs on, and how the sonar octree is kept consistent.

HoloOcean 2.3.0 sonars see the static scene through an octree that the engine builds once and
caches on disk per level and octree size (DESIGN_ITERATIONS.md DI-1/DI-2).  The project runs on a
*patched* HoloOcean root (docs/HOLOOCEAN_OCTREE_PATCH.md) whose engine understands the
``RebuildSonarOctree`` command and whose client exposes ``env.rebuild_sonar_octree()``:

* the patched root lives in its own folder (default ``F:\\Andrea\\holoocean_patched_root``,
  override with ``HOLO_FLEET_HOLOOCEAN_ROOT``); its world content is a junction to the official
  package, its octree cache is private, and the official installation (used by the Marine Race
  Arena's frozen benchmark) is never modified;
* every scenario rebuilds the sonar octree right after the static scene (arena gates, props)
  has been spawned, so the octree always describes the current scene: runtime props are visible
  and a cache written by another scene can never produce ghost echoes.

Without the patch the project falls back to the Phase-1 procedure: static scene first, sonars
attached afterwards, cache folder deleted when the scene signature changes.  The fallback is
kept only so that the code still runs on an unpatched machine; it is reported in every run.
"""

from __future__ import annotations

import hashlib
import json
import os
import shutil
from pathlib import Path
from typing import Iterable, Optional, Sequence

DEFAULT_PATCHED_ROOT = Path(r"F:\Andrea\holoocean_patched_root")
OCTREE_MIN = 0.06            # leaf size [m] (arena bars are 0.18 m thick)
OCTREE_MAX = 5.0             # mid-level size [m] -> engine folder "min6_max768"
# One environment box for every arena-area scenario: it bounds the octree root (fast to rebuild)
# and contains the Horseshoe Bay arena and the open-water test areas around it.
ENV_MIN = [-64.0, -64.0, -32.0]
ENV_MAX = [64.0, 64.0, 8.0]


def patched_root() -> Optional[Path]:
    root = Path(os.environ.get("HOLO_FLEET_HOLOOCEAN_ROOT", str(DEFAULT_PATCHED_ROOT)))
    return root if (root / "PATCH_INFO.json").exists() else None


def use_patched_holoocean() -> dict:
    """Point the HoloOcean client at the patched root (call before ``import holoocean``)."""
    root = patched_root()
    if root is None:
        return {"patched": False, "root": None}
    os.environ["HOLODECKPATH"] = str(root)
    info = json.loads((root / "PATCH_INFO.json").read_text(encoding="utf-8-sig"))
    return {"patched": True, "root": str(root), **info}


def client_supports_rebuild() -> bool:
    from holoocean.environments import HoloOceanEnvironment

    return hasattr(HoloOceanEnvironment, "rebuild_sonar_octree")


def octree_cache_dir(world: str = "OpenWater", package: str = "Ocean") -> Path:
    """Folder where the engine of the active root caches the octree for our octree size."""
    import holoocean.util as hu

    base = Path(hu.get_holoocean_path()) / "worlds" / package / "Windows" / "Holodeck" / "Octrees" / world
    cm_min = int(round(OCTREE_MIN * 100))
    cm_max = cm_min
    while cm_max < OCTREE_MAX * 100:
        cm_max *= 2
    return base / f"min{cm_min}_max{cm_max}"


def scene_signature(bars: Iterable, props: Sequence = (), env_box: Sequence = (ENV_MIN, ENV_MAX)) -> str:
    """Hash of everything static the octree depends on (used only by the unpatched fallback)."""
    h = hashlib.sha256()
    for b in bars:
        h.update(repr((tuple(getattr(b, "position", ())), tuple(getattr(b, "rotation_rpy_deg", ())),
                       tuple(getattr(b, "dimensions_m", ())))).encode())
    for p in props:
        h.update(repr(p).encode())
    h.update(repr((tuple(env_box[0]), tuple(env_box[1]), OCTREE_MIN, OCTREE_MAX)).encode())
    return h.hexdigest()[:16]


def fallback_prepare_cache(signature: str) -> bool:
    """Unpatched engines only: wipe the private cache folder when the static scene changed."""
    d = octree_cache_dir()
    marker = d / "holo_fleet_scene.json"
    if d.exists():
        try:
            same = json.loads(marker.read_text(encoding="utf-8")).get("signature") == signature
        except (OSError, ValueError):
            same = False
        if same:
            return False
        shutil.rmtree(d)
    return True


def fallback_mark_cache(signature: str) -> None:
    d = octree_cache_dir()
    if d.exists():
        (d / "holo_fleet_scene.json").write_text(json.dumps({"signature": signature}), encoding="utf-8")
