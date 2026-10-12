"""The HoloOcean sonar-octree patch is installed and passes its regression test.

The regression (probe/octree_rebuild_regression.py) needs the patched engine and takes about a
minute: it is marked ``holoocean`` and ``slow``.
"""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from holo_fleet.sim.holoocean_setup import patched_root  # noqa: E402

PATCH_FILE = ROOT / "patches" / "holoocean-2.3.0-sonar-octree-rebuild.patch"


def test_patch_file_touches_only_the_expected_files():
    text = PATCH_FILE.read_text(encoding="utf-8")
    files = sorted({line.split(" b/")[1] for line in text.splitlines() if line.startswith("diff --git")})
    assert files == sorted([
        "client/src/holoocean/command.py",
        "client/src/holoocean/environments.py",
        "engine/Source/Holodeck/ClientCommands/Private/CommandFactory.cpp",
        "engine/Source/Holodeck/ClientCommands/Private/RebuildSonarOctreeCommand.cpp",
        "engine/Source/Holodeck/ClientCommands/Public/CommandFactory.h",
        "engine/Source/Holodeck/ClientCommands/Public/RebuildSonarOctreeCommand.h",
        "engine/Source/Holodeck/General/Private/Octree.cpp",
        "engine/Source/Holodeck/General/Public/Octree.h",
        "engine/Source/Holodeck/HolodeckCore/Private/HolodeckAgent.cpp",
        "engine/Source/Holodeck/HolodeckCore/Private/HolodeckSonar.cpp",
        "engine/Source/Holodeck/HolodeckCore/Public/HolodeckAgent.h",
        "engine/Source/Holodeck/HolodeckCore/Public/HolodeckSonar.h",
    ])


@pytest.mark.skipif(patched_root() is None, reason="patched HoloOcean root not installed")
def test_patched_client_exposes_rebuild_api():
    from holoocean.environments import HoloOceanEnvironment

    assert hasattr(HoloOceanEnvironment, "rebuild_sonar_octree")


@pytest.mark.holoocean
@pytest.mark.slow
@pytest.mark.skipif(patched_root() is None, reason="patched HoloOcean root not installed")
def test_octree_rebuild_regression():
    proc = subprocess.run([sys.executable, str(ROOT / "probe" / "octree_rebuild_regression.py")],
                          capture_output=True, text=True, timeout=1200)
    res = json.loads((ROOT / "results" / "v2" / "octree_patch" / "regression.json").read_text(encoding="utf-8"))
    assert proc.returncode == 0, proc.stdout[-3000:] + proc.stderr[-3000:]
    assert res["all_passed"], {k: v for k, v in res["checks"].items() if not v}
    # the hazard the patch removes is real: before the rebuild, a reset leaves a ghost box behind
    assert res["ghosts_after_reset_before_rebuild"]["box"]
