import json
from threading import RLock
from unittest.mock import Mock, patch

import numpy as np
import pytest

from dimos.mapping.occupancy.path_resampling import ConstrainedPathSmoothingConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.OccupancyGrid import OccupancyGrid
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.replanning_a_star.global_planner import GlobalPlanner


def _planner(performance_logging_enabled: bool) -> tuple[GlobalPlanner, Path]:
    planner = GlobalPlanner.__new__(GlobalPlanner)
    planner._lock = RLock()
    planner._current_odom = PoseStamped(position=[0.0, 0.0, 0.0])
    planner._current_goal = PoseStamped(position=[1.0, 0.0, 0.0])
    planner.cancel_goal = Mock()
    planner._find_safe_goal = Mock(return_value=Vector3(1.0, 0.0, 0.0))
    raw_path = Path(
        poses=[
            PoseStamped(position=[0.0, 0.0, 0.0]),
            PoseStamped(position=[0.5, 0.0, 0.0]),
            PoseStamped(position=[1.0, 0.0, 0.0]),
        ]
    )
    costmap = OccupancyGrid(np.zeros((20, 20), dtype=np.int8), resolution=0.1)
    planner._find_wide_path = Mock(return_value=(raw_path, costmap))
    planner._publish_raw_path = False
    planner.raw_path = Mock()
    planner._constrained_path_smoothing_enabled = True
    planner._path_smoothing_performance_logging_enabled = performance_logging_enabled
    planner._path_smoothing_config = ConstrainedPathSmoothingConfig()
    planner.path = Mock()
    planner._local_planner = Mock()
    return planner, raw_path


@pytest.mark.parametrize("enabled", [False, True])
def test_smoothing_performance_logging_switch(enabled: bool) -> None:
    planner, result_path = _planner(enabled)

    def smooth(*args):
        timing = args[4]
        if timing is not None:
            timing["optimizer_total_ms"] = 1.0
        return result_path

    with (
        patch(
            "dimos.navigation.replanning_a_star.global_planner.constrained_smooth_resample_path",
            side_effect=smooth,
        ) as smoothing,
        patch("dimos.navigation.replanning_a_star.global_planner.logger.info") as log_info,
    ):
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


def test_disabled_performance_logging_skips_planner_timers() -> None:
    planner, result_path = _planner(False)

    with (
        patch(
            "dimos.navigation.replanning_a_star.global_planner.constrained_smooth_resample_path",
            return_value=result_path,
        ),
        patch("dimos.navigation.replanning_a_star.global_planner.time.perf_counter") as clock,
    ):
        planner._plan_path()

    clock.assert_not_called()
