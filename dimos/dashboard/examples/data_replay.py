"""Replay recorded YAML logs into a simple dashboard + rerun viewer."""

import threading
import time
from pathlib import Path

from reactivex.disposable import Disposable

from dimos.core import Module, Out, pSHMTransport, pLCMTransport
from dimos.core.blueprints import autoconnect
from dimos.core.core import rpc
from dimos.dashboard.module import Dashboard
from dimos.dashboard.rerun import layouts, RerunHook
from dimos.msgs.sensor_msgs import Image
from dimos.robot.unitree_webrtc.type.lidar import LidarMessage


class DataReplay(Module):
    color_image: Out[Image] = None  # type: ignore[assignment]
    lidar: Out[LidarMessage] = None  # type: ignore[assignment]

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
            return

        with file_path.open("r", encoding="utf-8") as f:
            for line in f:
                if not line.strip():
                    continue
                try:
                    parsed = yaml.safe_load(line) or []
                except Exception:
                    continue
                if isinstance(parsed, list):
                    for item in parsed:
                        yield item
                else:
                    yield parsed

    def _publish_stream(self, output_name: str, path: str) -> None:
        # Resolve the output by attribute name (e.g., "color_image" or "lidar").
        output: Out = getattr(self, output_name)
        while not self._stop_event.is_set():
            any_sent = False
            for msg in self._iter_messages(path):
                if self._stop_event.is_set():
                    break
                if output and output.transport:
                    output.publish(msg)  # type: ignore[no-untyped-call]
                time.sleep(self.interval_sec)
                any_sent = True
            if not self.loop or not any_sent:
                break

    @rpc
    def start(self) -> None:
        super().start()

        for output_name, path in self.replay_paths.items():
            thread = threading.Thread(
                target=self._publish_stream,
                args=(output_name, path),
                name=f"{output_name}-replay",
                daemon=True,
            )
            self._threads.append(thread)
            thread.start()

        self._disposables.add(Disposable(self._stop_event.set))
        for thread in self._threads:
            self._disposables.add(Disposable(thread.join))


layout = layouts.AllTabs(collapse_panels=False)
replay_paths = {
    "color_image": "/Users/jeffhykin/repos/dimos/dimos/dashboard/rerun/color_image.yaml",
    "lidar": "/Users/jeffhykin/repos/dimos/dimos/dashboard/rerun/lidar.yaml",
    "odom": "/Users/jeffhykin/repos/dimos/dimos/dashboard/rerun/odom.yaml",
}
blueprint = (
    autoconnect(
        DataReplay.blueprint(
            replay_paths=replay_paths,
            interval_sec=0.05,
            loop=True,
        ),
        Dashboard().blueprint(
            layout=layout,
            auto_open=True,
            terminal_commands={
                "agent-spy": "htop",
                "lcm-spy": "dimos lcmspy",
                # "skill-spy": "dimos skillspy",
            },
        ),
        RerunHook(
            "color_image",
            Image,
            target_entity=layout.entities.spatial2d,
        ).blueprint(),
        RerunHook(
            "lidar",
            LidarMessage,
            target_entity=layout.entities.spatial2d,
        ).blueprint(),
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
