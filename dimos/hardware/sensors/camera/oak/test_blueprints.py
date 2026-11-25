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

from collections.abc import Callable, Iterator
from typing import Any

import pytest

from dimos.core.module import Module
from dimos.hardware.sensors.camera.oak.blueprints import OakMountTf
from dimos.hardware.sensors.camera.oak.camera import OakCamera


@pytest.fixture
def make() -> Iterator[Callable[..., Any]]:
    modules: list[Module] = []

    def build(cls: type[Module], **config: Any) -> Any:
        module = cls(**config)
        modules.append(module)
        return module

    yield build
    for module in modules:
        module.stop()


@pytest.mark.parametrize(
    ("camera", "mount", "link"),
    [
        ({}, {}, "camera_link"),
        ({"frame_id_prefix": "arm"}, {"frame_id_prefix": "arm"}, "arm/camera_link"),
        (
            {"instance_name": "wrist/oakcamera"},
            {"instance_name": "wrist/oakmounttf"},
            "wrist/camera_link",
        ),
        ({"frame_id": "oak_link"}, {"frame_id": "oak_link"}, "oak_link"),
    ],
)
def test_the_mount_holds_the_camera_link_by_the_name_the_camera_gives_it(
    make: Callable[..., Any], camera: dict[str, str], mount: dict[str, str], link: str
) -> None:
    (edge,) = make(OakMountTf, **mount).transforms()

    assert (edge.frame_id, edge.child_frame_id) == ("world", link)
    assert make(OakCamera, **camera).frame_id == link
