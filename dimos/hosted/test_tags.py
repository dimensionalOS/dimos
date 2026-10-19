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

import pytest

from dimos.hosted import tags
from dimos.hosted.tags import as_tags, auto_tags, format_tags, missing


def test_auto_tags_merges_and_skips_a_failing_tagger() -> None:
    def broken() -> dict[str, str]:
        raise RuntimeError("no sysfs")

    taggers = [lambda: {"a": ""}, broken, lambda: {"gpu": "rtx", "b": ""}]
    assert auto_tags(taggers) == {"a": "", "gpu": "rtx", "b": ""}


def test_tag_requirements_match_keys_or_exact_values() -> None:
    tags = as_tags(["lab", "gpu=rtx", "site=athens"])
    assert tags == {"lab": "", "gpu": "rtx", "site": "athens"}
    assert missing(tags, ["lab", "gpu", "site=athens"]) == []
    assert missing(tags, ["site=sf", "jetson"]) == ["jetson", "site=sf"]
    assert format_tags(tags) == "gpu=rtx,lab,site=athens"


def test_every_tagger_answers_on_this_machine() -> None:
    assert "arch" in auto_tags()


@pytest.mark.parametrize("node", ["/dev/nvidia0", "/dev/nvgpu", "/dev/nvhost-gpu"])
def test_gpu_tagger_reads_device_nodes(node: str, monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(Path, "exists", lambda self: str(self) == node)
    monkeypatch.setattr(tags, "nvidia_smi_gpus", lambda: [])
    assert tags.gpu_tags() == {"gpu": "", "gpu_count": "1"}
    monkeypatch.setattr(Path, "exists", lambda self: False)
    assert tags.gpu_tags() == {}
