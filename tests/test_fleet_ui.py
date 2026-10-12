from types import SimpleNamespace

import cv2
import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.runner import run
from holo_fleet.sim.scenarios import formation_triangle
from holo_fleet.ui import fleet_camera as fc
from holo_fleet.ui.fleet_camera import FleetCamera, camera_pose, framing, viewport_rotation
from holo_fleet.ui.fleet_view import FleetView
from holo_fleet.ui.live import DemoUI


def _view(n=3):
    sc = formation_triangle(DEFAULT)
    names = [f"drone_{k}" for k in range(n)]
    return FleetView(sc, DEFAULT, names, selected=0)


def test_fleet_view_selects_map_icons_and_sidebar_rows():
    view = _view()
    d1 = view.px(view.P[1])
    assert view.select_at(*d1) == 1
    assert view.sel == 1
    d2 = view.px(view.P[2])
    assert view.select_at(*d2) == 2
    assert view.sel == 2
    assert view.select_at(1100, 380) == 0
    assert view.sel == 0
    assert view.select_at(1100, 500) is None
    assert view.sel == 0


def test_initial_selection_can_be_changed_by_click():
    sc = formation_triangle(DEFAULT)
    view = FleetView(sc, DEFAULT, sc.sim.names, selected=2)
    assert view.sel == 2
    assert view.select_at(*view.px(view.P[1])) == 1
    assert view.sel == 1


def test_window_is_created_with_the_first_frame_not_before_holoocean_starts(monkeypatch):
    """Created in the constructor, nothing would pump the window while the engine starts (Windows: "not
    responding"); it is created when the first frame is shown, as a plain fixed-size window."""
    calls = []
    monkeypatch.setattr("holo_fleet.ui.live.cv2.namedWindow", lambda *a: calls.append(("create", a[1])))
    monkeypatch.setattr("holo_fleet.ui.live.cv2.setMouseCallback", lambda *a: calls.append(("callback",)))
    ui = DemoUI(formation_triangle(DEFAULT), DEFAULT, ["drone_0", "drone_1", "drone_2"])
    assert calls == []
    ui._ensure_window()
    ui._ensure_window()
    assert calls == [("create", cv2.WINDOW_AUTOSIZE), ("callback",)]


def test_demo_ui_close_is_idempotent(monkeypatch):
    calls = []
    monkeypatch.setattr("holo_fleet.ui.live.cv2.namedWindow", lambda *a: calls.append("create"))
    monkeypatch.setattr("holo_fleet.ui.live.cv2.setMouseCallback", lambda *a: calls.append("callback"))
    monkeypatch.setattr("holo_fleet.ui.live.cv2.destroyWindow", lambda *a: calls.append("destroy"))
    monkeypatch.setattr("holo_fleet.ui.live.cv2.waitKey", lambda *a: calls.append("pump") or -1)
    ui = DemoUI(formation_triangle(DEFAULT), DEFAULT, ["drone_0", "drone_1", "drone_2"])
    ui._ensure_window()
    ui.close()
    ui.close()
    assert calls.count("destroy") == 1
    assert calls.count("pump") == 1
    assert not ui.show


def test_closed_opencv_window_disables_live_display(monkeypatch):
    monkeypatch.setattr("holo_fleet.ui.live.cv2.namedWindow", lambda *a: None)
    monkeypatch.setattr("holo_fleet.ui.live.cv2.setMouseCallback", lambda *a: None)
    monkeypatch.setattr("holo_fleet.ui.live.cv2.getWindowProperty", lambda *a: 0.0)
    ui = DemoUI(formation_triangle(DEFAULT), DEFAULT, ["drone_0", "drone_1", "drone_2"])
    ui._ensure_window()
    ui._check_window_open()
    assert not ui.show


def test_runner_closes_ui_when_run_raises(tmp_path, monkeypatch):
    closed = {"ui": 0, "sim": 0}

    class FakeSim:
        def __init__(self, spec, cfg, headless):
            self.names, self.t, self.setup_report, self.headless = list(spec.names), 0.0, {}, headless

        def start(self):
            pass

        def frames(self):
            return {name: None for name in self.names}

        def step(self, *_args):
            raise RuntimeError("simulated tick failure")

        def perf(self):
            return {}

        def close(self):
            closed["sim"] += 1

    class FakeController:
        def __init__(self, *_args):
            self.last_record, self.events = {}, []

        def step(self, *_args):
            return None

    class FakeUI:
        def __call__(self, **_kwargs):
            pass

        def close(self):
            closed["ui"] += 1

    monkeypatch.setattr("holo_fleet.runner.HoloFleetSim", FakeSim)
    monkeypatch.setattr("holo_fleet.runner.DroneController", FakeController)
    monkeypatch.setattr("holo_fleet.runner.Referee", lambda *a, **k: SimpleNamespace())
    with pytest.raises(RuntimeError, match="simulated tick failure"):
        run("formation_triangle", tmp_path, duration=0.1, ui=FakeUI())
    assert closed == {"ui": 1, "sim": 1}


def test_viewport_rotation_uses_the_measured_holoocean_convention():
    """move_viewport's rotation is the look direction with the sign inverted (measured on the engine):
    [0, 0, 10] looks straight down, [30, 0, 0] along -x, [0, 30, 0] along -y."""
    np.testing.assert_allclose(viewport_rotation([1.0, 0.0, 0.0]), [-1.0, 0.0, 0.0])    # along +x
    np.testing.assert_allclose(viewport_rotation([0.0, 0.0, -5.0]), [0.0, 0.0, 1.0])    # straight down
    np.testing.assert_allclose(viewport_rotation([0.0, 2.0, 0.0]), [0.0, -1.0, 0.0])    # along +y


def test_camera_looks_at_the_fleet_from_behind_below_the_surface():
    line6 = [[x, y, -5.0] for x, y in zip([0.0] * 6, [-8.75, -5.25, -1.75, 1.75, 5.25, 8.75])]
    centre, dist = framing(line6, heading=0.0)
    loc, rot = camera_pose(centre, dist, heading=0.0)
    assert loc[0] < centre[0] and abs(loc[1] - centre[1]) < 1e-6                  # behind, on the axis
    assert loc[2] <= -fc.SURFACE_CLEARANCE_M and loc[2] > centre[2]               # above the fleet, below the surface
    look = -rot                                                                    # measured convention
    assert look[0] > 0 and abs(look[1]) < 1e-9 and look[2] < 0                     # ahead (+x), tilted down
    np.testing.assert_allclose(look, (centre - loc) / np.linalg.norm(centre - loc))  # at the framed centre
    half = np.max(np.abs(np.asarray(line6)[:, 1] - centre[1])) + fc.MARGIN_M
    assert dist >= half / np.tan(np.radians(fc.HFOV_DEG / 2.0)) - 1e-9             # the whole line is in view
    assert fc.MIN_DISTANCE_M <= dist <= fc.MAX_DISTANCE_M                          # never lost in the fog


def test_camera_does_not_follow_drone_0_and_ignores_a_drone_lost_far_away():
    fleet = np.array([[0, 0, -5], [0, 3.5, -5], [-3.5, 0, -5], [-3.5, 3.5, -5]], float)
    a = framing(fleet, 0.0)
    b = framing(fleet[::-1], 0.0)
    np.testing.assert_allclose(a[0], b[0])
    assert a[1] == pytest.approx(b[1])
    lost = fleet.copy()
    lost[3] = [-30.0, 25.0, -5.0]
    c = framing(lost, 0.0)
    assert np.linalg.norm(c[0] - framing(fleet[:3], 0.0)[0]) < 1e-9               # framed without it


def test_camera_includes_a_near_gate_and_moves_smoothly():
    fleet = np.array([[0, 0, -5], [0, 3.5, -5], [0, -3.5, -5]], float)
    cam = FleetCamera()
    loc0, rot0 = cam.update(fleet, 0.0, gates=[[40.0, 0.0, -5.0]])                 # gate far: not framed
    assert not cam.with_gate
    near = fleet + [30.0, 0.0, 0.0]
    cam.update(near, 0.2, gates=[[40.0, 0.0, -5.0]])
    assert cam.with_gate                                                           # gate within 12 m: framed
    steps = []
    prev = cam.update(near, 0.2, gates=[[40.0, 0.0, -5.0]])[0]
    for _ in range(30):
        loc = cam.update(near, 0.2, gates=[[40.0, 0.0, -5.0]])[0]
        steps.append(float(np.linalg.norm(loc - prev)))
        prev = loc
    assert max(steps) < 3.0 and steps[-1] < steps[0]                               # converges, no teleport
    cam.update(near + [23.5, 0.0, 0.0], 0.2, gates=[[40.0, 0.0, -5.0]])            # 13.5 m past it: kept
    assert cam.with_gate
    cam.update(near + [26.0, 0.0, 0.0], 0.2, gates=[[40.0, 0.0, -5.0]])            # 16 m: released
    assert not cam.with_gate


def test_client_waits_for_the_engine_semaphores_instead_of_failing(monkeypatch):
    """HoloOcean's engine releases the loading semaphore before creating the server semaphores: the
    client's OpenSemaphore may run first (ERROR_FILE_NOT_FOUND); it is retried, other errors are not."""
    pywintypes = pytest.importorskip("pywintypes")
    from holoocean import holooceanclient

    from holo_fleet.sim import holoocean_setup as hs

    calls = {"n": 0}

    def flaky(self):
        calls["n"] += 1
        if calls["n"] < 3:
            raise pywintypes.error(2, "OpenSemaphore", "not found")
        return "connected"

    monkeypatch.setattr(holooceanclient.HoloOceanClient, "__windows_init__", flaky, raising=True)
    hs.wait_for_engine_semaphores(timeout_s=5.0)
    hs.wait_for_engine_semaphores(timeout_s=5.0)                                   # idempotent
    assert holooceanclient.HoloOceanClient.__windows_init__(object()) == "connected" and calls["n"] == 3

    def denied(self):
        raise pywintypes.error(5, "OpenSemaphore", "access denied")

    monkeypatch.setattr(holooceanclient.HoloOceanClient, "__windows_init__", denied, raising=True)
    hs.wait_for_engine_semaphores(timeout_s=5.0)
    with pytest.raises(pywintypes.error):
        holooceanclient.HoloOceanClient.__windows_init__(object())
