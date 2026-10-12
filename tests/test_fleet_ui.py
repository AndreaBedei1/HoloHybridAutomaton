from types import SimpleNamespace

import numpy as np
import pytest

from holo_fleet.config import DEFAULT
from holo_fleet.runner import run
from holo_fleet.sim.scenarios import formation_triangle
from holo_fleet.ui.fleet_camera import FleetCameraSmoother, fleet_camera_pose
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


def test_demo_ui_close_is_idempotent(monkeypatch):
    calls = []
    monkeypatch.setattr("holo_fleet.ui.live.cv2.namedWindow", lambda *a: calls.append("create"))
    monkeypatch.setattr("holo_fleet.ui.live.cv2.setMouseCallback", lambda *a: calls.append("callback"))
    monkeypatch.setattr("holo_fleet.ui.live.cv2.destroyWindow", lambda *a: calls.append("destroy"))
    monkeypatch.setattr("holo_fleet.ui.live.cv2.waitKey", lambda *a: calls.append("pump") or -1)
    ui = DemoUI(formation_triangle(DEFAULT), DEFAULT, ["drone_0", "drone_1", "drone_2"])
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


def test_fleet_camera_distance_covers_wider_fleet_and_near_gate_intruder():
    direction = [1.0, 0.0]
    compact = fleet_camera_pose([[0, 0, -5], [1, 0, -5]], direction)
    line6 = fleet_camera_pose([[x, 0, -5] for x in (-8, -5, -2, 2, 5, 8)], direction)
    lost = fleet_camera_pose([[0, 0, -5], [1, 0, -5], [0, 10, -5]], direction)
    assert line6[2] > compact[2]
    assert lost[2] > compact[2]

    with_gate = fleet_camera_pose([[0, 0, -5], [1, 0, -5]], direction, gate=[12, 0, -5])
    without_gate = fleet_camera_pose([[0, 0, -5], [1, 0, -5]], direction)
    assert with_gate[2] > without_gate[2]

    close_intruder = fleet_camera_pose([[0, 0, -5], [1, 0, -5]], direction,
                                       intruders=[[0, 10, -5]])
    far_intruder = fleet_camera_pose([[0, 0, -5], [1, 0, -5]], direction,
                                     intruders=[[0, 100, -5]])
    np.testing.assert_allclose(far_intruder[0], without_gate[0])
    assert close_intruder[2] > without_gate[2]


def test_camera_smoothing_limits_small_update_jump_and_slows_zoom_in():
    smoother = FleetCameraSmoother()
    first = smoother.update((np.array([0.0, 0.0, -5.0]), np.array([0.0, -20.0, 0.0]), 12.0), 0.2)
    second = smoother.update((np.array([1.0, 0.0, -5.0]), np.array([0.0, -20.0, 0.0]), 13.0), 0.2)
    assert np.linalg.norm(second[0] - first[0]) < 2.0
    third = smoother.update((np.array([0.0, 0.0, -5.0]), np.array([0.0, -20.0, 0.0]), 8.0), 0.2)
    assert third[2] > 8.0


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
