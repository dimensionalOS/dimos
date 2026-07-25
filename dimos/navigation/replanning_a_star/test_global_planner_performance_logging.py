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

import json

import numpy as np
import pytest
from pytest_mock import MockerFixture

from dimos.core.global_config import GlobalConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner


def _planner(
    mocker: MockerFixture, performance_logging_enabled: bool
) -> tuple[GlobalPlanner, Path]:
    planner = GlobalPlanner(
        GlobalConfig(),
        constrained_path_smoothing_enabled=True,
        path_smoothing_performance_logging_enabled=performance_logging_enabled,
    )
    planner._current_odom = PoseStamped(position=[0.0, 0.0, 0.0])
    planner._current_goal = PoseStamped(position=[1.0, 0.0, 0.0])
    mocker.patch.object(planner, "cancel_goal")
    mocker.patch.object(planner, "_find_safe_goal", return_value=Vector3(1.0, 0.0, 0.0))
    raw_path = Path(
        poses=[
            PoseStamped(position=[0.0, 0.0, 0.0]),
            PoseStamped(position=[0.5, 0.0, 0.0]),
            PoseStamped(position=[1.0, 0.0, 0.0]),
        ]
    )
    costmap = OccupancyGrid(np.zeros((20, 20), dtype=np.int8), resolution=0.1)
    mocker.patch.object(planner, "_find_wide_path", return_value=(raw_path, costmap))
    mocker.patch.object(planner.path, "on_next")
    mocker.patch.object(planner._local_planner, "start_planning")
    return planner, raw_path


@pytest.mark.parametrize("enabled", [False, True])
def test_smoothing_performance_logging_switch(mocker: MockerFixture, enabled: bool) -> None:
    planner, result_path = _planner(mocker, enabled)

    def smooth(*args):
        timing = args[4]
        if timing is not None:
            timing["optimizer_total_ms"] = 1.0
        return result_path

    smoothing = mocker.patch(
        "dimos.navigation.replanning_a_star.global_planner.constrained_smooth_resample_path",
        side_effect=smooth,
    )
    log_info = mocker.patch("dimos.navigation.replanning_a_star.global_planner.logger.info")

    planner._plan_path()

    timing = smoothing.call_args.args[4]
    if enabled:
        assert timing is not None
        assert timing["optimizer_total_ms"] == 1.0
        log_info.assert_called_once()
        assert log_info.call_args.args == ("Path smoothing performance.",)
        payload = json.loads(log_info.call_args.kwargs["smoothing_timing"])
        assert payload["optimizer_total_ms"] == 1.0
        assert payload["path_publish_ms"] >= 0.0
        assert payload["local_planner_handoff_ms"] >= 0.0
    else:
        assert timing is None
        log_info.assert_not_called()


def test_disabled_performance_logging_skips_planner_timers(mocker: MockerFixture) -> None:
    planner, result_path = _planner(mocker, False)
    mocker.patch(
        "dimos.navigation.replanning_a_star.global_planner.constrained_smooth_resample_path",
        return_value=result_path,
    )
    clock = mocker.patch("dimos.navigation.replanning_a_star.global_planner.time.perf_counter")

    planner._plan_path()

    clock.assert_not_called()
