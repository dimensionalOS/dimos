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

from dimos.evals.active_learning import HANDPICK_MEMORY, load_active_learning


def test_named_pack_is_handpick_memory() -> None:
    text = load_active_learning("handpick-memory")
    assert "walkable path" in text
    assert "the store is open" in text
    assert "fences" in text
    assert "roller shutter" in text
    assert "Before the final answer" not in text
    assert text == HANDPICK_MEMORY.strip() + "\n"


def test_prompt_file_is_loaded(tmp_path: Path) -> None:
    path = tmp_path / "notes.md"
    path.write_text("inspect the named door\n", encoding="utf-8")
    assert load_active_learning(str(path)) == "inspect the named door\n"


def test_unknown_name_is_rejected() -> None:
    with pytest.raises(ValueError, match="unknown active_learning"):
        load_active_learning("not-a-pack")


def test_empty_spec_is_rejected() -> None:
    with pytest.raises(ValueError, match="empty"):
        load_active_learning("  ")
