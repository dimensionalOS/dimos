# Copyright 2025 Dimensional Inc.
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

"""Replay recorded YAML logs into a simple dashboard + rerun viewer."""

from pathlib import Path
import sys
import threading
import time

from reactivex.disposable import Disposable
import rerun.blueprint as rrb

from dimos.core import Module, Out, pLCMTransport, pSHMTransport
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.dashboard.module import Dashboard
from dimos.msgs.nav_msgs import Odometry
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage


class DataReplay(Module):
    color_image: Out[Image] = None  # type: ignore[assignment]
    lidar: Out[LidarMessage] = None  # type: ignore[assignment]
    odom: Out[Odometry] = None  # type: ignore[assignment]

    def __init__(
        self,
        *,
        replay_paths: dict[str, str] | None = None,
        interval_sec: float = 0.05,
        loop: bool = True,
        **kwargs,
    ) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        self.replay_paths = replay_paths or {}
        self.interval_sec = interval_sec
        self.loop = loop
        self._stop_event = threading.Event()
        self._threads: list[threading.Thread] = []

    def _iter_messages(self, path: str):
        import yaml

        file_path = Path(path)
        if not file_path.exists():
            print(f"""[DataReplay] file {path} does not exist""", file=sys.stderr)
            return

        with file_path.open("r", encoding="utf-8") as f:
            for line_number, line in enumerate(f):
                if not line.strip():
                    continue
                try:
                    parsed = yaml.unsafe_load(line) or []
                except Exception as error:
                    print(f"""warning: line:{line_number} could not be parsed: {error}""")
                    continue

                if isinstance(parsed, list):
                    yield from parsed
                else:
                    yield parsed

    def _publish_stream(self, output_name: str, path: str) -> None:
        import rerun as rr

        rr.init("rerun_main", spawn=False, strict=True)
        print("""[DataReplay] _publish_stream started!""")
        # Resolve the output by attribute name (e.g., "color_image" or "lidar").
        output: Out = getattr(self, output_name)
        while not self._stop_event.is_set():
            any_sent = False
            for i, msg in enumerate(self._iter_messages(path)):
                if self._stop_event.is_set():
                    break
                if output and output.transport:
                    if i % 20 == 0:
                        print(f"[DataReplay] publishing {output_name} message {i}")
                    rr.log(f"/{output_name}", msg.to_rerun(), strict=True)
                    output.publish(msg)  # type: ignore[no-untyped-call]
                # time.sleep(self.interval_sec)
                any_sent = True
            if not self.loop or not any_sent:
                break

    @rpc
    def start(self) -> None:
        super().start()
        import rerun as rr

        # needs to be init-ed once per thread/process
        rr.init("rerun_main", spawn=False, strict=True)
        rr.log("logs", rr.TextLog("this entry has loglevel TRACE", level=rr.TextLogLevel.TRACE))
        try:
            for output_name, path in self.replay_paths.items():
                thread = threading.Thread(
                    target=self._publish_stream,
                    args=(output_name, path),
                    name=f"{output_name}-replay",
                    daemon=True,
                )
                self._threads.append(thread)
                thread.start()
                print("[DataReplay] waiting on _publish_stream to start...")
                time.sleep(4)

            self._disposables.add(Disposable(self._stop_event.set))
            for thread in self._threads:
                self._disposables.add(Disposable(thread.join))
        except Exception as error:
            print(f"""[DataReplay] error = {error}""")


# NOTE: this data was recorded with `from dimos.dashboard.support.utils import record_message`
replay_paths = {
    "color_image": "./dimos/dashboard/support/color_image.ignore.yaml",
    "lidar": "./dimos/dashboard/support/lidar.ignore.yaml",
    # "odom": "./dimos/dashboard/support/odom.ignore.yaml",
}
blueprint = (
    autoconnect(
        DataReplay.blueprint(
            replay_paths=replay_paths,
            interval_sec=0.05,
            loop=True,
        ),
        Dashboard.blueprint(
            auto_open=True,
            terminal_commands={
                "agent-spy": "htop",
                "lcm-spy": "dimos lcmspy",
                # "skill-spy": "dimos skillspy",
            },
        ),
    )
    .transports(
        {
            ("color_image", Image): pSHMTransport("/replay/color_image"),
            ("lidar", LidarMessage): pLCMTransport("/replay/lidar"),
        }
    )
    .global_config(n_dask_workers=1)
)


def main() -> None:
    coordinator = blueprint.build()
    print("Data replay running. Press Ctrl+C to stop.")
    coordinator.loop()


if __name__ == "__main__":
    main()
