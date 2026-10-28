# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0
"""Keep completed capabilities independent of the retired wrapper package."""

from pathlib import Path
import runpy

ROOT = Path(__file__).resolve().parents[2]
audit = runpy.run_path(str(ROOT / "scripts/audit_cdr_boundaries.py"))["audit"]


def test_completed_stack_has_no_retired_message_imports():
    report = audit(ROOT)
    assert report["status"] == "clear", report["legacy_imports"]
    assert not (ROOT / "dimos/msgs").exists()


def test_guard_checks_helpers_and_native_runtime_examples(tmp_path):
    example = tmp_path / "examples/demo.py"
    runtime = tmp_path / "native/python/driver/runtime.py"
    example.parent.mkdir(parents=True)
    runtime.parent.mkdir(parents=True)
    example.write_text("from dimos.msgs.helpers import resolve_type\n")
    runtime.write_text("from dimos.msgs.sensor_msgs import Image\n")
    report = audit(tmp_path)
    assert report["status"] == "unfinished"
    assert {item["file"] for item in report["legacy_imports"]} == {
        "examples/demo.py",
        "native/python/driver/runtime.py",
    }
