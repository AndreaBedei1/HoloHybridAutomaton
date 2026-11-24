"""Re-check legacy/INDEX.json: every moved item is at its new path with the recorded content hashes.

    python scripts/verify_legacy_index.py            # exit code 0 and "OK" when the index is complete and every hash matches

The hashes are taken over the content with CRLF normalised to LF (the form stored in git), so a Windows checkout with
core.autocrlf and a Linux checkout give the same values.  ``sha256`` is the digest of those bytes and ``git_blob`` the git
object id of the same bytes (``git hash-object``), so an item can also be compared with the history it was moved from.
Checked: the new path exists; the hashes and the size match; the original path no longer holds the item (except where
``successor_at_original_path`` is true: a reduced active successor of the same name stays in scripts/); every file under
legacy/ other than the index is listed exactly once.
"""

from __future__ import annotations

import hashlib
import json
import sys
from pathlib import Path
from typing import Dict, List

ROOT = Path(__file__).resolve().parents[1]
INDEX = ROOT / "legacy" / "INDEX.json"


def normalised(path: Path) -> bytes:
    return path.read_bytes().replace(b"\r\n", b"\n")


def digests(data: bytes) -> Dict[str, object]:
    blob = hashlib.sha1(b"blob %d\0" % len(data) + data).hexdigest()
    return {"sha256": hashlib.sha256(data).hexdigest(), "git_blob": blob, "bytes": len(data)}


def verify(root: Path = ROOT) -> List[str]:
    """Problems found (an empty list: the index is complete and consistent)."""
    index_path = root / "legacy" / "INDEX.json"
    doc = json.loads(index_path.read_text(encoding="utf-8"))
    problems: List[str] = []
    listed = set()
    for item in doc["items"]:
        new, old = item["new_path"], item["original_path"]
        if new in listed:
            problems.append(f"{new}: listed twice")
        listed.add(new)
        if not new.startswith("legacy/"):
            problems.append(f"{new}: not under legacy/")
        path = root / new
        if not path.is_file():
            problems.append(f"{new}: missing")
            continue
        got = digests(normalised(path))
        for key in ("sha256", "git_blob", "bytes"):
            if got[key] != item[key]:
                problems.append(f"{new}: {key} {got[key]} != recorded {item[key]}")
        if (root / old).exists() and not item.get("successor_at_original_path"):
            problems.append(f"{old}: still present at the original path")
        if not item.get("what"):
            problems.append(f"{new}: no 'what'")
        if not (item.get("related_tags") or item.get("related_commits")):
            problems.append(f"{new}: related to no tag and no commit")
    on_disk = {p.relative_to(root).as_posix() for p in (root / "legacy").rglob("*") if p.is_file()} - {"legacy/INDEX.json"}
    for extra in sorted(on_disk - listed):
        problems.append(f"{extra}: under legacy/ but not in the index")
    return problems


def main() -> int:
    problems = verify()
    doc = json.loads(INDEX.read_text(encoding="utf-8"))
    if problems:
        print("\n".join(problems))
        print(f"{len(problems)} problem(s)")
        return 1
    print(f"OK: {len(doc['items'])} items, every sha256 / git_blob / size matches")
    return 0


if __name__ == "__main__":
    sys.exit(main())
