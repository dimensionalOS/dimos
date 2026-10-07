# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path
from types import SimpleNamespace
from typing import Any

import pytest

from dimos.hosted import doctor
from dimos.hosted.doctor import run
from dimos.hosted.taggers import gpu
from dimos.hosted.tags import as_tags, auto_tags, format_tags, missing, taggers


def test_auto_tags_merges_and_skips_a_failing_tagger() -> None:
    def broken() -> set[str]:
        raise RuntimeError("no sysfs")

    assert auto_tags([lambda: {"a"}, broken, lambda: None, lambda: {"gpu": "rtx", "b": ""}]) == {
        "a": "",
        "gpu": "rtx",
        "b": "",
    }


def test_tag_requirements_match_keys_or_exact_values() -> None:
    tags = as_tags(["go2", "gpu=rtx", "site=athens"])
    assert tags == {"go2": "", "gpu": "rtx", "site": "athens"}
    assert missing(tags, ["go2", "gpu", "site=athens"]) == []
    assert missing(tags, ["site=sf", "jetson"]) == ["jetson", "site=sf"]
    assert missing(frozenset({"go2"}), ["go2"]) == []
    assert format_tags(tags) == "go2,gpu=rtx,site=athens"


def test_taggers_are_discovered_from_the_package() -> None:
    assert {fn.__module__.rsplit(".", 1)[1] for fn in taggers()} >= {"jetson", "gpu", "system"}


@pytest.mark.parametrize("node", ["/dev/nvidia0", "/dev/nvgpu", "/dev/nvhost-gpu"])
def test_gpu_tagger_reads_device_nodes(node: str, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(Path, "exists", lambda self: str(self) == node)
    monkeypatch.setattr(gpu.shutil, "which", lambda _: None)
    assert gpu.tags() == {"gpu": ""}
    monkeypatch.setattr(Path, "exists", lambda self: False)
    assert gpu.tags() == {}


def _doctor(check: Any, fix: Any = None) -> SimpleNamespace:
    doctor = SimpleNamespace(description="d", check=check)
    if fix is not None:
        doctor.fix = fix
    return doctor


def test_doctor_runner_outcomes() -> None:
    state = {"ok": False}

    def boom() -> bool:
        raise OSError("gone")

    results = run(
        fix=True,
        modules=[  # type: ignore[list-item]
            _doctor(lambda: True),
            _doctor(lambda: False),
            _doctor(lambda: state["ok"], fix=lambda: state.update(ok=True)),
            _doctor(boom),
        ],
    )
    assert [(r.ok, r.fix_note) for r in results] == [
        (True, None),
        (False, "no automatic fix"),
        (True, "fixed"),
        (False, "no automatic fix"),
    ]
    assert results[3].error == "OSError: gone"


def test_doctor_runner_without_fix_only_checks() -> None:
    fixed = []
    (result,) = run(modules=[_doctor(lambda: False, fix=lambda: fixed.append(1))])  # type: ignore[list-item]
    assert not result.ok and result.fix_note is None and not fixed


def test_doctors_are_discovered_from_the_package() -> None:
    names = {m.__name__.rsplit(".", 1)[1] for m in doctor.doctors()}
    assert names >= {"identity", "config", "revision", "service_running", "service_installed"}
    assert all(isinstance(m.description, str) and callable(m.check) for m in doctor.doctors())


def test_a_failing_warning_is_reported_not_fatal() -> None:
    doctor = _doctor(lambda: False)
    doctor.warning = True
    (result,) = run(modules=[doctor])  # type: ignore[list-item]
    assert not result.ok and result.warning
