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
from dimos.msgs.nav_msgs import Odometry

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
            return

        with file_path.open("r", encoding="utf-8") as f:
            for line_number, line in enumerate(f):
                print(f'''_iter_messages parsing line''')
                if not line.strip():
                    continue
                try:
                    parsed = yaml.unsafe_load(line) or []
                except Exception as error:
                    print(f'''warning: line:{line_number} could not be parsed: {error}''')
                    continue
                
                if isinstance(parsed, list):
                    for item in parsed:
                        yield item
                else:
                    yield parsed

    @rpc
    def start(self) -> None:
        super().start()
        try:
            print(f'''[DataReplay] starting {len(self.replay_paths)} threads''')
            outputs = tuple(getattr(self, output_name) for output_name in self.replay_paths.keys())
            print(f'''[DataReplay] outputs = {outputs}''')
            iterer = iter(self._publish_stream("color_image", "/Users/jeffhykin/repos/dimos/dimos/dashboard/rerun/color_image.yaml"))
            print(f'''[DataReplay] iterer = {iterer}''')
            print(f'''[DataReplay] next(iterer) = {next(iterer)}''')
            for msgs in zip(self._iter_messages(path) for output_name, path in self.replay_paths.items()):
                for output, message in zip(outputs, msgs):
                    if output and output.transport:
                        output.publish(message)  # type: ignore[no-untyped-call]
        except Exception as error:
            print(f'''[DataReplay] Error in .start replay: {error}''')
            
        self._disposables.add(Disposable(self._stop_event.set))

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
