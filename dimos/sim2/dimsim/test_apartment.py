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


from collections.abc import Iterator

import pytest

from dimos.core.hardware_files import hardware_file_topics
from dimos.core.hardware_module import HardwareModule
from dimos.core.transport_factory import make_transport
from dimos.msgs.files.FileInfo import FileInfo
from dimos.robot.unitree.go2.dimsim import Go2KinematicBase
from dimos.sim2.dimsim.apartment import DimSimApartment
from dimos.sim2.dimsim.backend import DimSimBackend
from dimos.sim2.sim import compose_usds, fetched_usd
from dimos.sim2.world_observations import WorldMirror

pytest.importorskip("pxr")

# The apartment's frame the Go2 mounts on, and so its namespace.
_GO2 = DimSimApartment.mounts.go2.frame

# The apartment's models are Git LFS data, which the scene fetches as it loads.
pytestmark = pytest.mark.self_hosted


@pytest.fixture(scope="module")
def apartment() -> Iterator[tuple[DimSimBackend, WorldMirror]]:
    scene = DimSimApartment()
    go2 = Go2KinematicBase(attached_to=_GO2, instance_name=f"{_GO2}/Go2KinematicBase")
    try:
        world, robot = (
            compose_usds([_fetched(scene), _fetched(go2)]),
            go2.simulated_robot(),
        )
    finally:
        scene.stop()
        go2.stop()
    backend = DimSimBackend(world, [robot], timestep=0.02)
    mirror = WorldMirror()
    mirror.apply(backend.frame())
    yield backend, mirror
    backend.close()


def _fetched(module: HardwareModule) -> tuple[str, str | None]:
    """``module``'s USD fetched over its hardware_file streams (wired as a blueprint wires them), and its mount."""
    request_topic, response_topic = hardware_file_topics(module.namespace)
    module.hardware_file_request.transport = make_transport(request_topic, FileInfo)
    module.hardware_file_response.transport = make_transport(response_topic, FileInfo)
    module.provide_hardware_files()
    return fetched_usd(module.namespace), module.attached_to()


def test_the_go2_stands_on_its_spot(apartment: tuple[DimSimBackend, WorldMirror]) -> None:
    backend, mirror = apartment
    for _ in range(50):
        mirror.apply(backend.advance()[1])
    go2 = mirror.entity(_GO2)
    assert go2 is not None and go2.pose is not None
    # The spot on the rug, plus the height the Go2's USD stands its base at.
    assert go2.pose[:3].tolist() == pytest.approx([3.0, 2.0, 0.12 + 0.445], abs=0.01)
