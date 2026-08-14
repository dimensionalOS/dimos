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

"""Path-stream middleware between ``MLSPlannerNative`` and ``DanHolonomicTC``.

``MLSPlannerNative`` re-roots and re-emits a path on every lidar frame: the
stream is sparse, unsmoothed, and unthrottled.

- ``lock_replan``: commit-window throttle (forward a path only at commit
  moments, suppress in between so the follower keeps a stable lookahead).
- ``resample_spacing_m`` / ``smoothing_window``: smooth + uniform resample
  the committed path (``smooth_resample_path``).

:class:`_ReplanGate` tracks the committed path with a :class:`PathDistancer`
and forwards a fresh planner path only when the robot has advanced
``lock_replan`` metres along the committed path, when a clicked goal first
lands, on cold start, or on a stop. Between commits it suppresses replans so
``DanHolonomicTC`` keeps a stable lookahead. Each committed path is smoothed
and uniformly resampled inside :meth:`_ReplanGate._commit` before it is stored
and forwarded.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np
from numpy.typing import NDArray
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.mapping.occupancy.path_resampling import smooth_resample_path
from dimos.msgs.geometry_msgs.PointStamped import PointStamped
from dimos.msgs.geometry_msgs.Pose import Pose
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.nav_msgs.Path import Path
from dimos.navigation.dannav.geometry.path_distancer import PathDistancer
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class DanLocalPlannerConfig(ModuleConfig):
    """Configuration for :class:`DanLocalPlanner`.

    Defaults make every feature off so the module is a pass-through until
    configured.
    """

    lock_replan: float = 0.0  # commit-window length in m; 0 disables gating
    goal_commit_tolerance_m: float = 0.3  # match a path end to the clicked goal
    resample_spacing_m: float = 0.0  # resample spacing in m; 0 disables smoothing
    smoothing_window: int = 100  # moving-average window


class _ReplanGate:
    """Transport-free core deciding which planner paths reach the follower.

    Owns the gate state and the commit decision, so it is unit-testable directly
    the way ``_HolonomicPathFollower`` is. It forwards a planner path only at
    *commit* moments and suppresses replans in between, so ``DanHolonomicTC``
    follows the last committed path over a guaranteed-stable lookahead instead of
    being yanked onto a freshly re-rooted chord every lidar frame.

    The committed path is tracked with a :class:`PathDistancer`; progress is
    arc-length along it from the pose at commit time. :meth:`on_odom` and
    :meth:`on_goal` record the robot pose and the armed click goal that the
    commit triggers consume. Smoothing runs in :meth:`_commit` before the path
    is stored.
    """

    def __init__(self, config: DanLocalPlannerConfig) -> None:
        self._cfg = config
        # Latest robot xy from odometry; arc-length progress is measured against
        # the committed path with it.
        self._robot_xy: NDArray[np.float64] | None = None
        # xy of the most recent finite click, armed until a path ends at it
        # (fresh-click commit) or a cancel disarms it.
        self._armed_goal: NDArray[np.float64] | None = None
        # PathDistancer over the path DanHolonomicTC is currently following, and
        # the robot's arc-length on it at commit time (~0 since MLS roots the
        # path at the robot). None means nothing committed (cold start / stop).
        self._committed: PathDistancer | None = None
        self._anchor_progress_m: float = 0.0

    def on_odom(self, odom: PoseStamped) -> None:
        self._robot_xy = np.array([odom.position.x, odom.position.y], dtype=float)

    def on_goal(self, goal: PointStamped) -> None:
        """Arm or disarm the fresh-click commit.

        A finite ``PointStamped`` arms it and stores the goal xy; a NaN point is
        the cancel from ``MovementManager._cancel_goal`` and disarms it (and
        drops the committed path). ``DanLocalPlanner`` does not touch
        ``stop_movement``.
        """
        if math.isnan(goal.x) or math.isnan(goal.y):
            self._armed_goal = None
            self._drop_committed()
            return
        self._armed_goal = np.array([goal.x, goal.y], dtype=float)

    def on_planner_path(self, path: Path) -> Path | None:
        """Return the path to forward, or ``None`` to suppress it.

        Commit triggers, first match wins:

        1. Empty planner path -> forward immediately (a stop) and drop the
           committed path. Safety override, always first.
        2. No committed path yet (cold start, or just after a stop/cancel) ->
           commit this path.
        3. Fresh click: a finite goal arrived since the last commit and this
           path ends within ``goal_commit_tolerance_m`` of it -> commit and
           disarm. Rejects a stale in-flight replan for the previous goal.
        4. Lock released: the robot has advanced ``>= lock_replan`` along the
           committed path since the last commit -> commit and re-anchor.
        5. Otherwise -> suppress (return ``None``, publish nothing).
        """
        if len(path.poses) == 0:  # 1. stop override
            self._drop_committed()
            return path
        if self._committed is None:  # 2. cold start
            return self._commit(path)
        if self._armed_goal is not None and _ends_near(
            path, self._armed_goal, self._cfg.goal_commit_tolerance_m
        ):  # 3. clicked -> agree
            self._armed_goal = None
            return self._commit(path)
        if self._lock_released():  # 4. window elapsed
            return self._commit(path)
        return None  # 5. replan held

    def _commit(self, path: Path) -> Path:
        """Adopt ``path`` as the committed path and return it for publishing.

        Smoothing reshapes ``path`` here before it is stored, so progress is
        measured against the same polyline the tracker follows.
        """
        smoothed = self._smooth(path)
        self._committed = PathDistancer(smoothed)
        self._anchor_progress_m = self._progress_on_committed()
        return smoothed

    def _smooth(self, path: Path) -> Path:
        """Smooth and uniformly resample a committed path.

        Restores ``GlobalPlanner``'s ``smooth_resample_path``: upsample,
        moving-average smooth, resample at a fixed spacing, endpoints fixed. The
        holonomic tracker was tuned against that dense, rounded path; the raw
        string-pulled MLS path has piecewise-constant tangents that step at chord
        junctions and swing ``PathDistancer.yaw_at_progress``.

        ``resample_spacing_m <= 0`` disables it (forward the raw path, unchanged
        object). The MLS path already ends at the goal, so its last pose is the
        natural ``goal_pose`` (``smooth_resample_path`` keeps endpoints fixed).
        """
        if self._cfg.resample_spacing_m <= 0.0 or len(path.poses) == 0:
            return path
        last = path.poses[-1]
        goal_pose = Pose(last.position, last.orientation)
        return smooth_resample_path(
            path, goal_pose, self._cfg.resample_spacing_m, self._cfg.smoothing_window
        )

    def _drop_committed(self) -> None:
        self._committed = None
        self._anchor_progress_m = 0.0

    def _lock_released(self) -> bool:
        """True when the robot has advanced ``>= lock_replan`` since the commit.

        ``lock_replan <= 0`` disables gating, so the lock is always released and
        every non-empty path commits (current/pass-through behavior).
        """
        if self._cfg.lock_replan <= 0.0:
            return True
        progress = self._progress_on_committed() - self._anchor_progress_m
        return progress >= self._cfg.lock_replan

    def _progress_on_committed(self) -> float:
        """Robot arc-length along the committed path, or 0 if it can't be measured."""
        if self._committed is None or self._robot_xy is None:
            return 0.0
        return self._committed.project(self._robot_xy).s_along_path_m


def _ends_near(path: Path, goal_xy: NDArray[np.float64], tolerance_m: float) -> bool:
    """True when ``path``'s last pose is within ``tolerance_m`` of ``goal_xy``."""
    end = path.poses[-1]
    return float(math.hypot(end.x - goal_xy[0], end.y - goal_xy[1])) <= tolerance_m


class DanLocalPlanner(Module):
    """Gate and shape ``MLSPlannerNative``'s path stream for ``DanHolonomicTC``.

    Sits between the planner output (remapped to ``planner_path``) and the
    follower's ``path`` input. Forwards the paths :class:`_ReplanGate` commits
    unchanged to ``DanHolonomicTC``.
    """

    config: DanLocalPlannerConfig

    planner_path: In[Path]  # from MLSPlannerNative.path (remapped)
    odom: In[PoseStamped]  # from GO2Connection.odom
    goal: In[PointStamped]  # from MovementManager.goal; the click/cancel signal

    path: Out[Path]  # to DanHolonomicTC.path

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._gate = _ReplanGate(self.config)

    @rpc
    def start(self) -> None:
        super().start()
        # Subscribe odom and goal before paths so the gate has the latest robot
        # pose and armed goal when a planner path arrives.
        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        self.register_disposable(Disposable(self.goal.subscribe(self._on_goal)))
        self.register_disposable(Disposable(self.planner_path.subscribe(self._on_planner_path)))

    def _on_odom(self, msg: PoseStamped) -> None:
        self._gate.on_odom(msg)

    def _on_goal(self, msg: PointStamped) -> None:
        self._gate.on_goal(msg)

    def _on_planner_path(self, msg: Path) -> None:
        forwarded = self._gate.on_planner_path(msg)
        if forwarded is not None:
            self.path.publish(forwarded)
