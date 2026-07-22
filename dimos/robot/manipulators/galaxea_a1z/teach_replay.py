# Copyright 2025-2026 Dimensional Inc.
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

"""Compile saved Memory2 A1Z episodes into safe coordinator trajectories."""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import numpy as np
from numpy.typing import NDArray

from dimos.learning.dataprep.core import Episode, EpisodeExtractor, extract_episodes
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.trajectory_msgs.JointTrajectory import JointTrajectory
from dimos.msgs.trajectory_msgs.TrajectoryPoint import TrajectoryPoint

A1Z_JOINT_NAMES = (
    "arm/joint1",
    "arm/joint2",
    "arm/joint3",
    "arm/joint4",
    "arm/joint5",
    "arm/joint6",
    "arm/gripper",
)

# The first six bounds are the vendor SDK's commandable soft limits. The
# gripper position is represented in meters throughout DimOS.
_POSITION_LOWER = np.array([-2.094, 0.0, -3.142, -1.484, -1.484, -2.007, 0.0])
_POSITION_UPPER = np.array([2.094, 3.142, 0.0, 1.484, 1.484, 2.007, 0.1])

# Teach-replay caps. Faster demonstrations are automatically time-scaled
# rather than clipped or rejected. Arm values are sized so natural hand
# teaching (peaks ~3.5 rad/s measured on hardware) replays at 1.0x while
# staying under the SDK's 4.0 rad/s streaming cap and its 7-20 rad/s
# per-joint velocity watchdogs. The gripper is not hand-driven: the teach
# CLI commands it through the SDK's own gripper controller, whose measured
# toggle motion reaches ~0.28 m/s and ~9 m/s^2 - the caps cover that with
# margin, since re-streaming what the SDK itself commanded is safe by
# construction (its torque limit bounds the physical motion either way).
_REPLAY_VELOCITY_MAX = np.array([3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 0.4])
_REPLAY_ACCELERATION_MAX = np.array([25.0, 25.0, 25.0, 25.0, 25.0, 25.0, 15.0])
_APPROACH_VELOCITY_MAX = np.array([0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.04])


@dataclass(frozen=True)
class RecordedEpisode:
    """Measured samples and metadata loaded from one saved Memory2 episode."""

    episode: Episode
    episode_index: int
    timestamps: NDArray[np.float64]
    positions: NDArray[np.float64]


@dataclass(frozen=True)
class PreparedEpisode:
    """Smoothed, uniformly sampled positions ready for trajectory execution."""

    recorded: RecordedEpisode
    timestamps: NDArray[np.float64]
    positions: NDArray[np.float64]
    velocities: NDArray[np.float64]
    requested_speed: float
    effective_speed: float

    @property
    def duration(self) -> float:
        return float(self.timestamps[-1])


def load_recorded_episode(db_path: Path, episode_index: int = -1) -> RecordedEpisode:
    """Load one successfully saved episode and reorder every sample by joint name."""
    store = SqliteStore(path=db_path, must_exist=True)
    try:
        episodes = [
            episode
            for episode in extract_episodes(store, EpisodeExtractor(status_stream="status"))
            if episode.success
        ]
        if not episodes:
            raise ValueError(f"No saved episodes found in {db_path}")

        resolved_index = episode_index if episode_index >= 0 else len(episodes) + episode_index
        if resolved_index < 0 or resolved_index >= len(episodes):
            raise IndexError(
                f"Episode index {episode_index} is out of range; {db_path} contains "
                f"{len(episodes)} saved episode(s), indexed 0..{len(episodes) - 1}"
            )
        episode = episodes[resolved_index]

        observations = store.stream("coordinator_joint_state", JointState).time_range(
            episode.start_ts, episode.end_ts
        )
        timestamps: list[float] = []
        positions: list[list[float]] = []
        for observation in observations:
            msg = observation.data
            if len(msg.name) != len(msg.position):
                raise ValueError(
                    "Recorded JointState has different name/position lengths at "
                    f"t={observation.ts:.6f}: {len(msg.name)} names, "
                    f"{len(msg.position)} positions"
                )
            by_name = dict(zip(msg.name, msg.position, strict=True))
            missing = [name for name in A1Z_JOINT_NAMES if name not in by_name]
            if missing:
                raise ValueError(
                    f"Recorded JointState at t={observation.ts:.6f} is missing {missing}"
                )
            timestamps.append(observation.ts)
            positions.append([float(by_name[name]) for name in A1Z_JOINT_NAMES])
    finally:
        store.stop()

    if len(timestamps) < 3:
        raise ValueError(
            f"Episode {resolved_index} contains only {len(timestamps)} joint-state sample(s); "
            "record at least 0.1 seconds"
        )

    ts = np.asarray(timestamps, dtype=np.float64)
    q = np.asarray(positions, dtype=np.float64)
    ts -= ts[0]
    _validate_recorded_samples(ts, q)
    return RecordedEpisode(
        episode=episode,
        episode_index=resolved_index,
        timestamps=ts,
        positions=q,
    )


def prepare_episode(
    recorded: RecordedEpisode,
    *,
    speed: float = 1.0,
    sample_rate_hz: float = 100.0,
    smoothing_window_s: float = 0.08,
) -> PreparedEpisode:
    """Smooth, resample, and automatically time-scale a recorded episode."""
    if speed <= 0:
        raise ValueError(f"speed must be positive, got {speed}")
    if sample_rate_hz <= 0:
        raise ValueError(f"sample_rate_hz must be positive, got {sample_rate_hz}")
    if smoothing_window_s < 0:
        raise ValueError(f"smoothing_window_s cannot be negative, got {smoothing_window_s}")

    source_ts = recorded.timestamps
    _validate_recorded_samples(source_ts, recorded.positions)
    source_uniform_ts = _uniform_times(float(source_ts[-1]), sample_rate_hz)
    source_uniform_q = _interpolate_positions(source_ts, recorded.positions, source_uniform_ts)
    # Smooth on the uniform grid, not the raw samples: the recorder's sample
    # spacing is irregular (10-100 ms under load), and differentiating
    # linear interpolation across those gaps manufactures acceleration
    # spikes an order of magnitude above the real motion, which would both
    # trip the safety time-scaler and put velocity ripple into the replayed
    # command stream. Two moving-average passes (triangular kernel) give a
    # continuous velocity profile, unlike a single pass whose derivative
    # still has corners.
    source_uniform_q = _smooth_uniform(
        source_uniform_q, window=round(smoothing_window_s * sample_rate_hz), passes=2
    )

    source_velocity = np.gradient(source_uniform_q, source_uniform_ts, axis=0)
    source_acceleration = np.gradient(source_velocity, source_uniform_ts, axis=0)
    safe_speed = _safe_playback_factor(source_velocity, source_acceleration)
    effective_speed = min(speed, safe_speed)
    if not np.isfinite(effective_speed) or effective_speed <= 0:
        raise ValueError("Could not derive a safe playback speed from the recorded episode")

    playback_duration = float(source_uniform_ts[-1] / effective_speed)
    playback_ts = _uniform_times(playback_duration, sample_rate_hz)
    source_query = np.minimum(playback_ts * effective_speed, source_uniform_ts[-1])
    playback_q = _interpolate_positions(source_uniform_ts, source_uniform_q, source_query)
    playback_velocity = np.gradient(playback_q, playback_ts, axis=0)
    playback_velocity[0] = 0.0
    playback_velocity[-1] = 0.0

    _validate_positions(playback_q, context="Prepared trajectory")
    return PreparedEpisode(
        recorded=recorded,
        timestamps=playback_ts,
        positions=playback_q,
        velocities=playback_velocity,
        requested_speed=speed,
        effective_speed=effective_speed,
    )


def build_execution_trajectory(
    current_positions: dict[str, float],
    prepared: PreparedEpisode,
    *,
    sample_rate_hz: float = 100.0,
    settle_s: float = 0.35,
    final_hold_s: float = 0.35,
) -> JointTrajectory:
    """Prepend a minimum-jerk approach and append a final hold."""
    missing = [name for name in A1Z_JOINT_NAMES if name not in current_positions]
    if missing:
        raise ValueError(f"Current robot state is missing {missing}")
    current = np.asarray([current_positions[name] for name in A1Z_JOINT_NAMES], dtype=float)
    _validate_positions(current[np.newaxis, :], context="Current robot state")

    target = prepared.positions[0]
    delta = np.abs(target - current)
    # Minimum jerk has a peak normalized velocity of 1.875 / duration.
    approach_duration = max(1.0, float(np.max(1.875 * delta / _APPROACH_VELOCITY_MAX)))
    approach_ts = _uniform_times(approach_duration, sample_rate_hz)
    u = approach_ts / approach_duration
    blend = 10.0 * u**3 - 15.0 * u**4 + 6.0 * u**5
    blend_velocity = (30.0 * u**2 - 60.0 * u**3 + 30.0 * u**4) / approach_duration
    approach_q = current + blend[:, np.newaxis] * (target - current)
    approach_velocity = blend_velocity[:, np.newaxis] * (target - current)

    points = [
        TrajectoryPoint(
            time_from_start=float(ts),
            positions=q.tolist(),
            velocities=dq.tolist(),
        )
        for ts, q, dq in zip(approach_ts, approach_q, approach_velocity, strict=True)
    ]

    replay_offset = approach_duration + settle_s
    points.extend(
        TrajectoryPoint(
            time_from_start=float(replay_offset + ts),
            positions=q.tolist(),
            velocities=dq.tolist(),
        )
        for ts, q, dq in zip(
            prepared.timestamps,
            prepared.positions,
            prepared.velocities,
            strict=True,
        )
    )
    points.append(
        TrajectoryPoint(
            time_from_start=float(replay_offset + prepared.duration + final_hold_s),
            positions=prepared.positions[-1].tolist(),
            velocities=[0.0] * len(A1Z_JOINT_NAMES),
        )
    )
    return JointTrajectory(points=points, joint_names=list(A1Z_JOINT_NAMES))


def _validate_recorded_samples(
    timestamps: NDArray[np.float64],
    positions: NDArray[np.float64],
) -> None:
    if timestamps.ndim != 1 or positions.shape != (len(timestamps), len(A1Z_JOINT_NAMES)):
        raise ValueError(
            f"Unexpected episode shape: timestamps={timestamps.shape}, positions={positions.shape}"
        )
    if not np.all(np.isfinite(timestamps)) or not np.all(np.isfinite(positions)):
        raise ValueError("Recorded episode contains NaN or infinite values")
    deltas = np.diff(timestamps)
    if np.any(deltas <= 0):
        index = int(np.flatnonzero(deltas <= 0)[0])
        raise ValueError(
            "Recorded joint-state timestamps are not strictly increasing at samples "
            f"{index} and {index + 1}"
        )
    if timestamps[-1] < 0.1:
        raise ValueError(f"Recorded episode is only {timestamps[-1]:.3f}s; record at least 0.1s")
    _validate_positions(positions, context="Recorded episode")


def _validate_positions(positions: NDArray[np.float64], *, context: str) -> None:
    invalid = np.argwhere(
        (positions < _POSITION_LOWER[np.newaxis, :]) | (positions > _POSITION_UPPER[np.newaxis, :])
    )
    if invalid.size == 0:
        return
    sample_index, joint_index = (int(value) for value in invalid[0])
    value = positions[sample_index, joint_index]
    raise ValueError(
        f"{context} leaves the commandable range at sample {sample_index}: "
        f"{A1Z_JOINT_NAMES[joint_index]}={value:.4f}, allowed "
        f"[{_POSITION_LOWER[joint_index]:.4f}, {_POSITION_UPPER[joint_index]:.4f}]. "
        "No values were clipped; re-teach the episode inside the vendor command limits."
    )


def _smooth_uniform(
    positions: NDArray[np.float64],
    *,
    window: int,
    passes: int = 1,
) -> NDArray[np.float64]:
    """Zero-phase moving average over uniformly sampled positions."""
    window = min(window, len(positions))
    if window % 2 == 0:
        window -= 1
    if window <= 1:
        return positions.copy()

    radius = window // 2
    kernel = np.ones(window, dtype=np.float64) / window
    smoothed = positions
    for _ in range(passes):
        padded = np.pad(smoothed, ((radius, radius), (0, 0)), mode="edge")
        smoothed = np.column_stack(
            [
                np.convolve(padded[:, joint], kernel, mode="valid")
                for joint in range(positions.shape[1])
            ]
        )
    return smoothed


def _uniform_times(duration: float, rate_hz: float) -> NDArray[np.float64]:
    count = max(2, int(np.ceil(duration * rate_hz)) + 1)
    return np.linspace(0.0, duration, count, dtype=np.float64)


def _interpolate_positions(
    source_ts: NDArray[np.float64],
    source_q: NDArray[np.float64],
    target_ts: NDArray[np.float64],
) -> NDArray[np.float64]:
    return np.column_stack(
        [np.interp(target_ts, source_ts, source_q[:, joint]) for joint in range(source_q.shape[1])]
    )


def _safe_playback_factor(
    velocity: NDArray[np.float64],
    acceleration: NDArray[np.float64],
) -> float:
    max_velocity = np.max(np.abs(velocity), axis=0)
    max_acceleration = np.max(np.abs(acceleration), axis=0)
    velocity_factor = np.divide(
        _REPLAY_VELOCITY_MAX,
        max_velocity,
        out=np.full_like(max_velocity, np.inf),
        where=max_velocity > 1e-9,
    )
    acceleration_factor = np.sqrt(
        np.divide(
            _REPLAY_ACCELERATION_MAX,
            max_acceleration,
            out=np.full_like(max_acceleration, np.inf),
            where=max_acceleration > 1e-9,
        )
    )
    # Leave a small numerical margin for the second interpolation pass.
    return float(0.98 * min(np.min(velocity_factor), np.min(acceleration_factor)))
