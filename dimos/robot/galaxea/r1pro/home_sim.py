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

"""Desktop house demo lifecycle: scene, ACT packing, navigation and delivery."""

from concurrent.futures import CancelledError
from contextlib import suppress
from pathlib import Path
import threading
import time
from typing import Any
from uuid import uuid4
import xml.etree.ElementTree as ET

import numpy as np
from pydantic import Field
from threadpoolctl import threadpool_limits  # type: ignore[import-untyped]

from dimos.constants import DIMOS_PROJECT_ROOT, RECORDINGS_DIR
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.robot.galaxea.r1pro.grasping_task import GraspingTask
from dimos.robot.galaxea.r1pro.home_spec import HomeControlSpec, HomeSceneSpec, PackingPolicySpec
from dimos.robot.galaxea.r1pro.learning import R1PRO_PICK_PLACE_JOINTS
from dimos.robot.galaxea.r1pro.navigation_cloud import save_environment_cloud
from dimos.robot.galaxea.r1pro.navigation_sim import (
    NAV_TASK,
    R1ProNavigationSim,
    R1ProNavigationSimConfig,
)
from dimos.robot.galaxea.r1pro.packing_run import PackingRunConfig, run_packing_sequence
from dimos.robot.galaxea.r1pro.packing_sim import (
    PACKING_BODIES,
    PACKING_SOURCES,
    prepare_packing_scene,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class R1ProHomeSimConfig(R1ProNavigationSimConfig):
    scene_package: Path = DIMOS_PROJECT_ROOT / "dimos/data/scene_packages/hssd_102344115"
    output: Path = Field(default_factory=lambda: RECORDINGS_DIR / "r1pro-home-sim" / uuid4().hex)
    seed: int = 5000
    jitter: float = Field(default=0.003, ge=0, le=0.01)


class R1ProHomeSim(R1ProNavigationSim):
    """Build the scene at startup; listing a blueprint never compiles MuJoCo."""

    config: R1ProHomeSimConfig

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._preparation_lock = threading.Lock()
        self._session: dict[str, Any] | None = None
        self._thread_limits: threadpool_limits | None = None

    @rpc
    def build(self) -> None:
        self.prepare_home_scene()
        super().build()

    @rpc
    def prepare_home_scene(self) -> dict[str, Any]:
        """Prepare once and share the exact scene, home and limits with control."""
        with self._preparation_lock:
            if self._session is not None:
                return self._session
            package = (
                Path(global_config.scene_package or self.config.scene_package)
                .expanduser()
                .resolve()
            )
            if not package.exists():
                raise FileNotFoundError(
                    f"House scene package is missing: {package}. "
                    "Set --scene-package to your hssd_102344115 scene package."
                )
            self._thread_limits = threadpool_limits(limits=1, user_api="blas")
            output = self.config.output.expanduser().resolve()
            output.mkdir(parents=True, exist_ok=True)
            if (output / "result.json").exists():
                raise FileExistsError(
                    f"Demo results already exist in {output}; choose a new --output"
                )
            scene = prepare_packing_scene(output / "scene.xml", scene_package=package)
            tree = ET.parse(scene)
            rng = np.random.default_rng(self.config.seed)
            for name, xy in zip(PACKING_BODIES, PACKING_SOURCES, strict=True):
                body = tree.find(f'.//body[@name="{name}"]')
                assert body is not None
                x, y = np.asarray(xy) + rng.uniform(-self.config.jitter, self.config.jitter, 2)
                body.set("pos", f"{x} {y} 0.771")
            tree.write(scene, encoding="unicode")
            cloud = output / "navigation-cloud.npy"
            save_environment_cloud(scene, cloud)
            with GraspingTask(scene, images=False) as task:
                self.config.reset_joint_positions = task.home.tolist()
                limits = [task.model.joint(name).range.tolist() for name in R1PRO_PICK_PLACE_JOINTS]
            self.config.address = scene
            self._session = dict(
                output=str(output),
                scene=str(scene),
                cloud=str(cloud),
                seed=self.config.seed,
                limits=limits,
            )
            logger.info("House scene ready", output=str(output))
            return self._session

    @rpc
    def home_session(self) -> dict[str, Any]:
        """Return this run's asset and evidence paths after build."""
        if self._session is None:
            raise RuntimeError("House scene has not been built")
        return self._session

    @rpc
    def stop(self) -> None:
        try:
            super().stop()
        finally:
            if self._thread_limits is not None:
                self._thread_limits.restore_original_limits()
                self._thread_limits = None


class R1ProHomeDemoConfig(ModuleConfig):
    auto_run: bool = True
    pick_timeout: float = Field(default=30.0, gt=0, le=120)


class R1ProHomeDemo(Module):
    """Automatically run the demonstration and leave its native viewer open."""

    config: R1ProHomeDemoConfig
    _sim: HomeSceneSpec
    _control: HomeControlSpec
    _policy: PackingPolicySpec

    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        self._cancel = threading.Event()
        self._thread: threading.Thread | None = None
        self._state = "idle"
        self._result: dict[str, Any] = {}

    @rpc
    def start(self) -> None:
        super().start()
        if self.config.auto_run:
            self._thread = threading.Thread(target=self._run, name="r1pro-home-demo", daemon=True)
            self._thread.start()

    def _pause(self, seconds: float) -> None:
        if self._cancel.wait(seconds):
            raise CancelledError("House demonstration stopped")

    def _run(self) -> None:
        self._state = "starting"
        try:
            session = self._sim.home_session()
            deadline = time.monotonic() + 120
            while (
                not self._sim.is_simulation_running() or NAV_TASK not in self._control.list_tasks()
            ):
                if time.monotonic() > deadline:
                    raise RuntimeError("House simulation and control did not become ready")
                self._pause(0.1)
            self._state = "running"
            logger.info(
                "Packing five bottles, then delivering the tray to the laptop",
                output=session["output"],
            )
            run_packing_sequence(
                self._control,
                self._policy,
                self._sim,
                PackingRunConfig(
                    artifact=Path(self._policy.rollout_status()["artifact"]),
                    output=Path(session["output"]),
                    seed=session["seed"],
                    seconds=self.config.pick_timeout,
                ),
                self._result,
                navigation_cloud=Path(session["cloud"]),
                pause=self._pause,
            )
            self._state = "completed" if self._result["success"] else "failed"
            logger.info("House demonstration finished", state=self._state, output=session["output"])
        except CancelledError:
            self._state = "stopped"
        except Exception as error:
            self._state = "failed"
            self._result.update(success=False, error=str(error))
            logger.exception("House demonstration failed", error=str(error))
        finally:
            with suppress(Exception):
                self._policy.stop_rollout()
            with suppress(Exception):
                self._control.task_invoke(NAV_TASK, "cancel", {})
            with suppress(Exception):
                self._sim.stop_navigation_base()

    @rpc
    def demo_status(self) -> dict[str, Any]:
        """Read progress and the result location without transferring all samples."""
        return {
            "state": self._state,
            "success": self._result.get("success", False),
            "picks_completed": sum(
                bool(pick.get("success")) for pick in self._result.get("picks", [])
            ),
            "error": self._result.get("error"),
            **self._sim.home_session(),
        }

    @rpc
    def stop(self) -> None:
        self._cancel.set()
        if self._thread is not None:
            self._thread.join(timeout=3)
        super().stop()
