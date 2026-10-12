"""Guard against a dead HoloOcean engine.

While a sonar exists the HoloOcean client waits for the engine without timeout (octree
construction can take long).  If the engine crashes in that state the Python process would
block forever.  The watchdog polls the engine process and terminates this process with a clear
message (exit code 3) instead.
"""

from __future__ import annotations

import os
import sys
import threading
import time
from typing import Callable, Optional


class EngineWatchdog:
    def __init__(self, env, on_death: Optional[Callable[[str], None]] = None, period_s: float = 2.0):
        self.proc = getattr(env, "_world_process", None)
        self.on_death = on_death
        self.period_s = period_s
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, name="holoocean-watchdog", daemon=True)

    def start(self) -> "EngineWatchdog":
        if self.proc is not None:
            self._thread.start()
        return self

    def stop(self) -> None:
        self._stop.set()

    def _run(self) -> None:
        while not self._stop.wait(self.period_s):
            code = self.proc.poll()
            if code is not None and not self._stop.is_set():
                msg = f"HoloOcean engine exited unexpectedly (code {code}); see <root>/Windows/Holodeck/Saved/Logs"
                print(f"[watchdog] {msg}", file=sys.stderr, flush=True)
                if self.on_death is not None:
                    try:
                        self.on_death(msg)
                    except Exception:  # noqa: BLE001 - best effort before exiting
                        pass
                time.sleep(0.2)
                os._exit(3)
