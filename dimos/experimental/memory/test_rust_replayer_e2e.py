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
import subprocess
from threading import Event
import time
import uuid

import numpy as np
import pytest

from dimos.core.global_config import global_config
from dimos.core.stream import Out
from dimos.core.transport import LCMTransport
from dimos.experimental.memory.rust_replayer import (
    RustReplayer,
    RustSqliteReplayStoreConfig,
)
from dimos.memory.codecs.lcm import LcmCodec
from dimos.memory.codecs.lz4 import Lz4Codec
from dimos.memory.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.Image import Image, ImageFormat
from dimos.msgs.sensor_msgs.Imu import Imu

pytestmark = pytest.mark.self_hosted_large


class InteropRustReplayer(RustReplayer):
    imu: Out[Imu]
    color_image: Out[Image]


@pytest.fixture(scope="module")
def rust_replayer_executable() -> Path:
    subprocess.run(
        ["cargo", "build", "-p", "dimos-memory-replayer"],
        check=True,
    )
    executable = Path("target/debug/dimos-memory-replayer").resolve()
    assert executable.is_file()
    return executable


def test_sqlite_replay_publishes_decoded_lcm_and_accepts_controls(
    tmp_path: Path,
    rust_replayer_executable: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    monkeypatch.setattr(global_config, "transport", "lcm")
    artifact = tmp_path / "recording.db"
    expected_imu = Imu(
        ts=10.0,
        frame_id="imu_link",
        angular_velocity=Vector3(1.0, 2.0, 3.0),
    )
    expected_image = Image(
        data=np.full((16, 16, 3), [20, 80, 140], dtype=np.uint8),
        format=ImageFormat.RGB,
        frame_id="camera",
        ts=10.1,
    )
    with SqliteStore(path=str(artifact)) as store:
        store.stream("imu", Imu, codec=Lz4Codec(LcmCodec(Imu))).append(expected_imu, ts=10.0)
        store.stream("color_image", Image).append(expected_image, ts=10.1)

    suffix = uuid.uuid4().hex[:8]
    lcm_url = "udpm://239.255.76.67:7667?ttl=0"
    imu_transport: LCMTransport[Imu] = LCMTransport(f"/replay_imu_{suffix}", Imu, url=lcm_url)
    image_transport: LCMTransport[Image] = LCMTransport(
        f"/replay_image_{suffix}", Image, url=lcm_url
    )
    received_imu: list[Imu] = []
    received_images: list[Image] = []
    complete = Event()

    def on_imu(message: Imu) -> None:
        received_imu.append(message)
        if received_images:
            complete.set()

    def on_image(message: Image) -> None:
        received_images.append(message)
        if received_imu:
            complete.set()

    imu_unsubscribe = imu_transport.subscribe(on_imu)
    image_unsubscribe = image_transport.subscribe(on_image)
    replayer = InteropRustReplayer(
        executable=str(rust_replayer_executable),
        build_command=None,
        store=RustSqliteReplayStoreConfig(path=str(artifact)),
        speed=100.0,
        shutdown_timeout=10.0,
    )
    replayer.imu.transport = imu_transport
    replayer.color_image.transport = image_transport
    try:
        replayer.start()
        assert complete.wait(10.0)
        replayer.pause()
        replayer.set_speed(2.0)
        replayer.resume()

        assert [message.lcm_encode() for message in received_imu] == [expected_imu.lcm_encode()]
        decoded = received_images[0]
        assert decoded.frame_id == expected_image.frame_id
        assert decoded.format is ImageFormat.RGB
        assert decoded.data.shape == expected_image.data.shape
        assert np.mean(np.abs(decoded.data.astype(float) - expected_image.data.astype(float))) < 5
    finally:
        replayer.stop()
        imu_unsubscribe()
        image_unsubscribe()
        imu_transport.stop()
        image_transport.stop()
        time.sleep(0.1)
