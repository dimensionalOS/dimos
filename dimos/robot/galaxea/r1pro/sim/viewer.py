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

"""Browser controls and visualization for the R1 Pro sim2 demonstration."""

from io import BytesIO
import threading
import time
from typing import Any

from dimos_lcm.sim_msgs import WorldState
import numpy as np
import viser
from viser.extras import ViserUrdf
import yourdfpy  # type: ignore[import-untyped]

from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import ModuleConfig
from dimos.core.shared_config import ConfigSenders, SharedConfigModule
from dimos.core.stream import In, Out
from dimos.manipulation.visualization.viser.config import ViserVisualizationConfig
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.robot.galaxea.r1pro.sim.config import HOME, UPPER_JOINTS
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1ProSimViewerConfig(ModuleConfig):
    port: int = ViserVisualizationConfig().port


class R1ProSimViewer(SharedConfigModule):
    config: R1ProSimViewerConfig
    observations: In[WorldState]
    cmd_vel: Out[Twist]
    upper_command: Out[JointState]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._xml = ""
        self._lock = threading.RLock()
        self._server: viser.ViserServer | None = None
        self._robot: ViserUrdf | None = None
        self._root: viser.FrameHandle | None = None
        self._done = threading.Event()
        self._thread: threading.Thread | None = None
        self._velocity = (0.0, 0.0, 0.0)
        self._driver: int | None = None
        self._drive_until = 0.0
        self._joint_names: tuple[str, ...] = ()
        self._next_ns = 0
        self._sliders: dict[str, viser.GuiSliderHandle[float]] = {}

    def config_receive(self, config_senders: ConfigSenders) -> None:
        models = [
            config["robot_models"]["r1pro"]
            for _, config in config_senders
            if "r1pro" in config.get("robot_models", {})
        ]
        if len(models) != 1:
            raise ValueError("R1 Pro viewer requires one robot model")
        self._xml = models[0].xml

    @rpc
    def start(self) -> None:
        server = viser.ViserServer(
            host=global_config.listen_host, port=self.config.port, label="R1 Pro · sim2"
        )
        self._server = server
        server.scene.set_up_direction("+z")
        server.scene.add_grid(
            "/floor",
            width=20,
            height=20,
            plane="xy",
            cell_size=0.25,
            section_size=1,
            plane_opacity=1,
            plane_color=(42, 48, 58),
        )
        self._root = server.scene.add_frame("/robot", show_axes=False)
        self._robot = ViserUrdf(
            server, yourdfpy.URDF.load(BytesIO(self._xml.encode())), root_node_name="/robot"
        )
        server.gui.add_markdown(
            "## R1 Pro\nEach drive button runs for up to 3 seconds. **Stop** brakes sooner. Closing the controlling browser also stops driving. Joint sliders move the torso and arms."
        )
        with server.gui.add_folder("Drive"):
            for label, velocity in [
                ("Forward", (0.3, 0, 0)),
                ("Back", (-0.3, 0, 0)),
                ("Left", (0, 0.3, 0)),
                ("Right", (0, -0.3, 0)),
                ("Turn left", (0, 0, 0.4)),
                ("Turn right", (0, 0, -0.4)),
                ("Stop", (0, 0, 0)),
            ]:
                button = server.gui.add_button(label, color="red" if label == "Stop" else None)
                button.on_click(self._drive_handler(velocity))
        self._joint_names = tuple(self._robot.get_actuated_joint_names())
        limits = self._robot.get_actuated_joint_limits()
        for label, names in [
            ("Torso", UPPER_JOINTS[:4]),
            ("Left arm", UPPER_JOINTS[4:11]),
            ("Right arm", UPPER_JOINTS[11:]),
        ]:
            with server.gui.add_folder(label):
                for name in names:
                    lower, upper = limits[name]
                    assert lower is not None and upper is not None
                    slider = server.gui.add_slider(
                        name.split("/")[-1], min=lower, max=upper, step=0.01, initial_value=0.0
                    )
                    slider.on_update(self._joint_handler(name))
                    self._sliders[name] = slider
        server.gui.add_button("Home arms and torso").on_click(self._home)
        server.on_client_connect(self._connect)
        server.on_client_disconnect(self._disconnect)
        super().start()
        self._thread = threading.Thread(
            target=self._heartbeat, name="r1pro-demo-drive", daemon=True
        )
        self._thread.start()
        logger.info("R1 Pro simulation controls", url=self.url())

    def _drive_handler(self, velocity: tuple[float, float, float]) -> Any:
        def handle(event: viser.GuiEvent[Any]) -> None:
            with self._lock:
                self._velocity = velocity
                self._driver = event.client_id if any(velocity) else None
                self._drive_until = time.monotonic() + 3.0
            if not any(velocity):
                self.cmd_vel.publish(Twist())

        return handle

    def _joint_handler(self, name: str) -> Any:
        def handle(event: viser.GuiEvent[viser.GuiSliderHandle[float]]) -> None:
            self.upper_command.publish(JointState(name=[name], position=[event.target.value]))

        return handle

    def _home(self, event: viser.GuiEvent[Any]) -> None:
        del event
        for name, value in zip(UPPER_JOINTS, HOME, strict=True):
            self._sliders[name].value = value
        self.upper_command.publish(JointState(name=list(UPPER_JOINTS), position=list(HOME)))

    def _connect(self, client: viser.ClientHandle) -> None:
        client.camera.position = np.array((2.5, -2.5, 1.8))
        client.camera.look_at = np.array((0, 0, 0.9))

    def _disconnect(self, client: viser.ClientHandle) -> None:
        with self._lock:
            if self._driver == client.client_id:
                self._velocity = (0.0, 0.0, 0.0)
                self._driver = None
                self.cmd_vel.publish(Twist())

    def _heartbeat(self) -> None:
        while not self._done.wait(0.1):
            with self._lock:
                if self._driver is None:
                    continue
                vx, vy, wz = self._velocity
                if time.monotonic() >= self._drive_until:
                    vx = vy = wz = 0.0
                    self._driver = None
            self.cmd_vel.publish(Twist(linear=Vector3(vx, vy, 0), angular=Vector3(0, 0, wz)))

    async def handle_observations(self, sample: WorldState) -> None:
        if (
            self._robot is None
            or self._root is None
            or self._server is None
            or sample.header.time_ns < self._next_ns
        ):
            return
        self._next_ns = sample.header.time_ns + 33_000_000
        body = next(b for b in sample.bodies if b.body_id == "r1pro/base_link")
        p, q = body.pose.position, body.pose.orientation
        with self._server.atomic():
            self._root.position = np.array((p.x, p.y, p.z))
            self._root.wxyz = np.array((q.w, q.x, q.y, q.z))
            positions = dict(zip(sample.joint_truth.name, sample.joint_truth.position, strict=True))
            self._robot.update_cfg(np.array([positions[name] for name in self._joint_names]))

    @rpc
    def url(self) -> str:
        if self._server is None:
            raise RuntimeError("viewer has not started")
        return f"http://{global_config.listen_host}:{self._server.get_port()}"

    @rpc
    def stop(self) -> None:
        self._done.set()
        if self._thread is not None:
            self._thread.join(timeout=2)
        if self._server is not None:
            self._server.stop()
        super().stop()
