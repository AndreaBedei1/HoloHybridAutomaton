"""Internal consistency of the parameter provenance registry config/parameter_sources.json."""

import json
import re
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REG = json.loads((ROOT / "config" / "parameter_sources.json").read_text(encoding="utf-8"))
ROWS = REG["parameters"]


def test_the_counts_match_the_rows():
    c = Counter(r["verdict"] for r in ROWS)
    assert REG["counts"]["rows"] == len(ROWS)
    assert all(REG["counts"][k] == c.get(k, 0) for k in REG["verdict_classes"])
    assert set(c) <= set(REG["verdict_classes"])


def test_every_parameter_is_listed_once_with_its_fields():
    assert len({r["parameter"] for r in ROWS}) == len(ROWS)
    for r in ROWS:
        assert r["parameter"] and r["section"] and "value" in r and isinstance(r["citation_keys"], list)


def test_every_citation_key_is_in_the_bibliography():
    bib = set(re.findall(r"@\w+\{([^,\s]+),", (ROOT / "references.bib").read_text(encoding="utf-8")))
    assert {k for r in ROWS for k in r["citation_keys"]} <= bib


def test_every_cited_repository_file_exists():
    text = json.dumps(ROWS)
    paths = set(re.findall(r"(?<![\w/])((?:results|calibration|formal|holo_fleet)/[\w/.\-]+\.(?:json|py))", text))
    assert paths and not [p for p in paths if not (ROOT / p).exists()]
