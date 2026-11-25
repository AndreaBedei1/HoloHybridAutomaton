"""legacy/INDEX.json: every item moved to legacy/ is listed with its original path, its description, the tag / commit it relates to
and its content hashes, and nothing under legacy/ is unlisted (scripts/verify_legacy_index.py)."""

import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def _verifier():
    spec = importlib.util.spec_from_file_location("verify_legacy_index", ROOT / "scripts" / "verify_legacy_index.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_every_moved_item_is_indexed_with_matching_hashes():
    assert _verifier().verify(ROOT) == []


def test_the_index_names_the_campaigns_and_the_commit_they_relate_to():
    doc = json.loads((ROOT / "legacy" / "INDEX.json").read_text(encoding="utf-8"))
    covered = {c for i in doc["items"] for c in i["covers"]}
    assert {"campaign_448", "validation_164", "validation_34", "gate_safety_targeted", "b2_negative_bias",
            "criteria_v9_reclassification"} <= covered
    assert doc["last_main_commit"] == "a9ffa34" and doc["moved_from_commit"] == "d5999c2"
    assert {"b2-freeze-v4", "b2-freeze-v11", "b2-v11-validation"} <= set(doc["preexisting_tags"])
    for item in doc["items"]:
        assert set(item["related_tags"]) <= set(doc["preexisting_tags"]), item["new_path"]


def test_a_changed_byte_is_detected(tmp_path):
    v = _verifier()
    index = json.loads((ROOT / "legacy" / "INDEX.json").read_text(encoding="utf-8"))
    (tmp_path / "legacy").mkdir()
    item = dict(index["items"][0], new_path="legacy/x.json", original_path="results/x.json")
    (tmp_path / "legacy" / "x.json").write_text("{}\n", encoding="utf-8")
    (tmp_path / "legacy" / "INDEX.json").write_text(json.dumps({"items": [item]}), encoding="utf-8")
    problems = v.verify(tmp_path)
    assert any("sha256" in p for p in problems) and any("git_blob" in p for p in problems) and any("bytes" in p for p in problems)
    (tmp_path / "legacy" / "extra.txt").write_text("unlisted", encoding="utf-8")
    assert any("not in the index" in p for p in v.verify(tmp_path))
