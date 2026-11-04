"""OS-held project engine slots. Crashes release locks, but never hide a live orphan engine."""

from __future__ import annotations

import json
import os
from pathlib import Path
import tempfile

import psutil

SLOTS = Path(__file__).resolve().parents[2] / "results/.holo_slots"


def worker_engines(pid, created):
    """Read launch evidence; a PID alone never authorizes process changes."""
    records = []
    if created is None:
        return records
    for path in SLOTS.glob("slot*.json"):
        try:
            record = json.loads(path.read_text(encoding="utf-8"))
            engine = record.get("engine")
            if (record.get("pid") == pid and record.get("created") == created and isinstance(engine, dict)
                    and all(k in engine for k in ("pid", "ppid", "created", "uuid"))):
                records.append(engine)
        except (OSError, ValueError):
            pass  # a concurrent metadata write is not ownership evidence; retry on the next watch tick
    return records


def same_process(record):
    try:
        return psutil.Process(record["pid"]).create_time() == record["created"]
    except psutil.NoSuchProcess:
        return False
    except (psutil.AccessDenied, KeyError):
        return True  # uncertainty is not permission to reuse a slot


def owned_engine(record):
    """Return only the exact engine we launched, never a recycled PID or a foreign child."""
    try:
        p = psutil.Process(record["pid"])
        if (p.create_time() == record["created"] and p.ppid() == record["ppid"]
                and p.name().lower() == "holodeck.exe"
                and "--HolodeckUUID=" + record["uuid"] in p.cmdline()):
            return p
    except (psutil.Error, KeyError):
        pass
    return None


class EngineLease:
    def __init__(self, directory=None, capacity=None):
        self.directory = Path(SLOTS if directory is None else directory)
        self.capacity = int(os.environ.get("HOLO_FLEET_ENGINE_SLOTS", "1")) if capacity is None else capacity
        if self.capacity not in (1, 2):
            raise ValueError("engine capacity must be 1 or 2 (two only after isolation validation)")
        self.file = None
        self.slot = None
        self.metadata = None
        self.record = {"pid": os.getpid(), "created": psutil.Process().create_time(), "uuid": None, "engine": None}

    @staticmethod
    def _lock(file, acquire):
        file.seek(0)
        if os.name == "nt":
            import msvcrt
            msvcrt.locking(file.fileno(), msvcrt.LK_NBLCK if acquire else msvcrt.LK_UNLCK, 1)
        else:
            import fcntl
            fcntl.flock(file, fcntl.LOCK_EX | fcntl.LOCK_NB if acquire else fcntl.LOCK_UN)

    def acquire(self):
        self.directory.mkdir(parents=True, exist_ok=True)
        for slot in range(self.capacity):
            path = self.directory / f"slot{slot}.lock"
            file = open(path, "a+b")
            try:
                self._lock(file, True)
            except OSError:
                file.close()
                continue
            try:
                metadata = path.with_suffix(".json")
                if path.stat().st_size > 1 and not metadata.exists():
                    raise ValueError("legacy slot metadata must be inspected before migration")
                text = metadata.read_text(encoding="utf-8") if metadata.exists() else ""
                old = json.loads(text) if text.strip() else {}
                if not isinstance(old, dict):
                    raise ValueError("slot metadata is not an object")
                if old and (same_process(old) or (old.get("engine") and same_process(old["engine"]))):
                    self._lock(file, False)
                    file.close()
                    continue
                self.file, self.slot, self.metadata = file, slot, metadata
                self.update()
                return self
            except BaseException as exc:
                self._lock(file, False)
                file.close()
                self.file = None
                if isinstance(exc, (ValueError, UnicodeError)):
                    raise RuntimeError(f"unreadable engine slot {path}; preserve and inspect it") from exc
                raise
        raise RuntimeError("all project engine slots are busy; no engine started")

    def update(self, **fields):
        self.record.update(fields)
        self._persist(self.record)

    def _persist(self, record):
        # Atomic replacement retains the previous identity if the worker dies during a write.
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=self.directory,
                                         prefix=f"slot{self.slot}-", suffix=".tmp", delete=False) as file:
            temporary = Path(file.name)
            json.dump(record, file)
            file.flush()
            os.fsync(file.fileno())
        try:
            os.replace(temporary, self.metadata)
        finally:
            temporary.unlink(missing_ok=True)

    def release(self):
        if self.file is None:
            return
        # Preserve a live engine's identity even if its owner exits abnormally.
        if not self.record.get("engine") or not same_process(self.record["engine"]):
            self._persist({})
        self._lock(self.file, False)
        self.file.close()
        self.file = None
