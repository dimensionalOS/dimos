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

"""Official one-episode Habitat/VLN-CE environment owned by the container."""

import gzip
import json
import os
import tempfile

import habitat
from habitat.utils.visualizations import maps as habitat_maps
from habitat_extensions import VLNCEDatasetV1, get_extended_config
import numpy as np
import quaternion

from .motion import PlanarMotionError, integrate_planar, record_accepted_motion

TASK_CONFIG = "/opt/VLN-CE/habitat_extensions/config/vlnce_task.yaml"


class EpisodeEnvironmentError(RuntimeError):
    """The official environment could not honor the pinned episode contract."""


class OfficialEpisodeEnvironment:
    """Thin motion adapter around Habitat's official task and measure registry."""

    def __init__(self, private_case, episode, dataset_path, scenes_dir, work_dir):
        self.private_case = private_case
        self.episode = episode
        self.work_dir = str(work_dir)
        if not os.path.isdir(self.work_dir):
            os.makedirs(self.work_dir)
        gt_path = os.path.join(self.work_dir, "episode-gt.json.gz")
        _write_private_gt(gt_path, episode)
        config = _official_config(private_case, dataset_path, scenes_dir, gt_path)
        dataset = VLNCEDatasetV1(config.DATASET)
        if len(dataset.episodes) != 1:
            raise EpisodeEnvironmentError("official dataset filtering did not select one episode")
        selected = dataset.episodes[0]
        if selected.episode_id != private_case["episode_id"]:
            raise EpisodeEnvironmentError("official dataset selected a foreign episode")
        if selected.instruction.instruction_text != private_case["instruction"]:
            raise EpisodeEnvironmentError("official dataset instruction changed during loading")

        self._env = habitat.Env(config=config, dataset=dataset)
        self._observations = self._env.reset()
        self._trajectory = [self._env.sim.get_agent_state().position.tolist()]
        self._submitted = False
        self._closed = False
        self._validate_observations(self._observations)

    @property
    def observations(self):
        return self._observations

    @property
    def trajectory(self):
        return list(self._trajectory)

    @property
    def pose(self):
        return self._env.sim.get_agent_state()

    def static_occupancy(self, resolution=0.05):
        """Return the complete current-floor navmesh projection with no overlays."""

        state = self._env.sim.get_agent_state()
        grid = habitat_maps.get_topdown_map(
            self._env.sim.pathfinder,
            height=float(state.position[1]),
            draw_border=False,
            meters_per_pixel=resolution,
        )
        lower, upper = self._env.sim.pathfinder.get_bounds()
        return {
            "resolution": float(resolution),
            "width": int(grid.shape[1]),
            "height": int(grid.shape[0]),
            "origin_x": float(lower[0]),
            "origin_z": float(lower[2]),
            "upper_x": float(upper[0]),
            "upper_z": float(upper[2]),
            "traversability": np.ascontiguousarray(grid == 1, dtype=np.uint8),
        }

    def apply_planar(self, linear_x, linear_y, angular_z, period_seconds):
        """Integrate one collision-aware velocity period and update official measures."""

        if self._submitted:
            raise EpisodeEnvironmentError("motion is unavailable after route submission")
        state = self._env.sim.get_agent_state()
        start = np.array(state.position, dtype=np.float64)
        try:
            requested, rotation_xyzw = integrate_planar(
                start,
                [state.rotation.x, state.rotation.y, state.rotation.z, state.rotation.w],
                linear_x,
                linear_y,
                angular_z,
                period_seconds,
            )
        except PlanarMotionError as error:
            raise EpisodeEnvironmentError(str(error))
        accepted = np.array(self._env.sim.pathfinder.try_step(start, requested))
        rotation = quaternion.quaternion(
            rotation_xyzw[3],
            rotation_xyzw[0],
            rotation_xyzw[1],
            rotation_xyzw[2],
        )
        observations = self._env.sim.get_observations_at(
            accepted.tolist(),
            rotation,
            keep_agent_at_new_pose=True,
        )
        if observations is None:
            raise EpisodeEnvironmentError("Habitat rejected a collision-filtered pose")
        action = {"action": "DIMOS_PLANAR"}
        self._env.task._is_episode_active = self._env.task._check_episode_is_active(
            observations=observations,
            action=action,
            episode=self._env.current_episode,
        )
        self._env.task.measurements.update_measures(
            episode=self._env.current_episode,
            action=action,
            task=self._env.task,
        )
        self._env._update_step_stats()
        self._observations = observations
        self._validate_observations(observations)
        try:
            collided = record_accepted_motion(self._trajectory, requested, accepted)
        except PlanarMotionError as error:
            raise EpisodeEnvironmentError(str(error))
        return {
            "collided": collided,
            "requested_position": requested.tolist(),
            "accepted_position": accepted.tolist(),
        }

    def submit_route(self):
        """Apply the official irreversible VLN-CE STOP action exactly once."""

        if self._submitted:
            raise EpisodeEnvironmentError("route has already been submitted")
        self._observations = self._env.step("STOP")
        self._submitted = True
        return self.metrics()

    def metrics(self):
        """Return values produced by Habitat/VLN-CE's configured measures."""

        return self._env.get_metrics()

    def close(self):
        if not self._closed:
            self._env.close()
            self._closed = True

    def _validate_observations(self, observations):
        if observations["rgb"].shape != (224, 224, 3):
            raise EpisodeEnvironmentError("official RGB observation shape changed")
        if observations["depth"].shape != (256, 256, 1):
            raise EpisodeEnvironmentError("official depth observation shape changed")


def _official_config(private_case, dataset_path, scenes_dir, gt_path):
    config = get_extended_config(TASK_CONFIG)
    config.defrost()
    config.DATASET.DATA_PATH = str(dataset_path)
    config.DATASET.SCENES_DIR = str(scenes_dir)
    config.DATASET.SPLIT = private_case["split"]
    config.DATASET.CONTENT_SCENES = [private_case["scene_id"].split("/")[-2]]
    config.DATASET.EPISODES_ALLOWED = [private_case["episode_id"]]
    config.TASK.NDTW.GT_PATH = str(gt_path)
    config.TASK.NDTW.SPLIT = private_case["split"]
    config.TASK.SENSORS = ["INSTRUCTION_SENSOR"]
    config.ENVIRONMENT.MAX_EPISODE_STEPS = 0
    config.ENVIRONMENT.MAX_EPISODE_SECONDS = 0
    config.SIMULATOR.AGENT_0.HEIGHT = 1.5
    config.SIMULATOR.AGENT_0.RADIUS = 0.1
    config.SIMULATOR.RGB_SENSOR.POSITION = [0.0, 1.25, 0.0]
    config.SIMULATOR.DEPTH_SENSOR.POSITION = [0.0, 1.25, 0.0]
    config.freeze()
    return config


def _write_private_gt(path, episode):
    directory = os.path.dirname(path)
    descriptor, temporary = tempfile.mkstemp(prefix=".episode-gt-", dir=directory)
    os.close(descriptor)
    try:
        with gzip.open(temporary, "wt", encoding="utf-8") as handle:
            json.dump(
                {str(episode["episode_id"]): {"locations": episode["reference_path"]}},
                handle,
                sort_keys=True,
                separators=(",", ":"),
            )
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
