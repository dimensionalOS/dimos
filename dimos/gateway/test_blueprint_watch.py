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

import asyncio
import json
from pathlib import Path
from typing import Any

from dimos.gateway.blueprint_watch import BlueprintWatch, diff, relevant, snapshot


def test_only_sources_and_package_metadata_count() -> None:
    assert relevant("all_blueprints.py")
    assert relevant("acme_arm-1.0.dist-info")
    assert relevant("_acme.pth")
    assert not relevant("README.md")
    assert diff(["go2", "spot"], ["spot", "g1", "acme/arm"]) == (["acme/arm", "g1"], ["go2"])


def test_a_snapshot_sees_a_new_blueprint_file(tmp_path: Path) -> None:
    robot = tmp_path / "dimos" / "robot"
    (robot / "__pycache__").mkdir(parents=True)
    (robot / "__pycache__" / "x.cpython-312.pyc").write_text("")
    before = snapshot(tmp_path)
    (robot / "README.md").write_text("docs")
    assert snapshot(tmp_path) == before
    (robot / "new_robot.py").write_text("blueprint = 1\n")
    assert snapshot(tmp_path) != before


# a fake checkout whose listing is a JSON file: adding a blueprint file and a name re-lists and sends the difference
async def test_a_registry_change_sends_the_difference(tmp_path: Path) -> None:
    robot = tmp_path / "dimos" / "robot"
    robot.mkdir(parents=True)
    listing = tmp_path / "listing.json"
    listing.write_text(json.dumps([{"name": "unitree-go2", "kind": "builtin"}]))

    async def list_blueprints() -> list[dict[str, Any]]:
        return json.loads(listing.read_text())

    heard: list[tuple[list[str], list[str]]] = []
    touched: list[bool] = []
    watch = BlueprintWatch(
        tmp_path,
        list_blueprints,
        lambda _listed, added, removed: heard.append((added, removed)),
        lambda: touched.append(True),
        poll=0.05,
        settle=0.05,
        min_gap=0,
    )
    task = asyncio.create_task(watch.run())
    try:
        await asyncio.sleep(0.2)
        assert watch.names == ["unitree-go2"]
        # a source change that doesn't change the list: re-checked, no event
        (robot / "helper.py").write_text("x = 1\n")
        await asyncio.sleep(0.4)
        assert touched and heard == []
        listing.write_text(
            json.dumps(
                [{"name": "unitree-go2", "kind": "builtin"}, {"name": "gamma", "kind": "builtin"}]
            )
        )
        (robot / "gamma.py").write_text("gamma = 1\n")
        for _ in range(40):
            if heard:
                break
            await asyncio.sleep(0.05)
        assert heard == [(["gamma"], [])]
        assert [entry["name"] for entry in watch.listed or []] == ["unitree-go2", "gamma"]
    finally:
        task.cancel()
