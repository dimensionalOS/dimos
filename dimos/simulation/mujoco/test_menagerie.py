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
import sys
from types import SimpleNamespace

from pytest import MonkeyPatch

from dimos.simulation.mujoco import menagerie


def test_ensure_menagerie_clones_again_after_an_interrupted_clone(
    tmp_path: Path, monkeypatch: MonkeyPatch
) -> None:
    path = tmp_path / "mujoco_menagerie"
    (path / ".git").mkdir(parents=True)  # what a clone killed part-way leaves behind
    found_on_disk: list[bool] = []

    def clone() -> None:
        found_on_disk.append(path.exists())
        (path / ".git").mkdir(parents=True)
        (path / ".git" / "index").touch()

    monkeypatch.setattr(menagerie, "menagerie_path", lambda: path)
    # Stands in for mujoco_playground, a 4 s import whose helper only clones a missing directory.
    mjx_env = SimpleNamespace(ensure_menagerie_exists=clone)
    monkeypatch.setitem(sys.modules, "mujoco_playground._src", SimpleNamespace(mjx_env=mjx_env))

    menagerie.ensure_menagerie()
    menagerie.ensure_menagerie()  # the finished clone is left alone

    assert found_on_disk == [False]
