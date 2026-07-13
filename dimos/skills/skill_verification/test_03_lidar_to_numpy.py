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

"""Wishlist item 3 — convert lidar to numpy signals (LidarPointCloudClient).

Headless: spin a synthetic scanner around a room, feed the real client
callbacks, and check the accumulated (N, 3) numpy snapshot grows and holds the
room. Renders the accumulation curve + final cloud. See manifest.yaml.
"""

from __future__ import annotations

import numpy as np

from dimos.mapping.pointclouds.live import LidarPointCloudClient
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

SKILL_ID = "03-lidar-to-numpy"


def _room_environment() -> np.ndarray:
    """A 6x4 m room outline plus a pillar, as a static point cloud."""
    rng = np.random.default_rng(3)
    W, H = 6.0, 4.0
    n_wall = 3200
    xs = rng.uniform(-W / 2, W / 2, n_wall)
    ys = rng.uniform(-H / 2, H / 2, n_wall)
    side = rng.integers(0, 4, n_wall)
    wx = np.where(side < 2, np.where(side == 0, -W / 2, W / 2), xs)
    wy = np.where(side < 2, ys, np.where(side == 2, -H / 2, H / 2))
    wz = rng.uniform(0.0, 2.4, n_wall)
    room = np.column_stack([wx, wy, wz])
    a = rng.uniform(0, 2 * np.pi, 350)
    pillar = np.column_stack(
        [1.2 + 0.18 * np.cos(a), -0.6 + 0.18 * np.sin(a), rng.uniform(0, 2.0, 350)]
    )
    return np.vstack([room, pillar])


def test_lidar_snapshot_accumulates_numpy_cloud(skill_output) -> None:
    """snapshot() returns a growing (N, 3) float array over a scanner revolution."""
    import matplotlib.pyplot as plt

    env = _room_environment()
    client = LidarPointCloudClient()
    rng = np.random.default_rng(3)

    n_msgs = 48
    counts = [0]
    for i in range(n_msgs):
        ang = 2 * np.pi * i / n_msgs
        bearing = np.arctan2(env[:, 1], env[:, 0])
        d = np.angle(np.exp(1j * (bearing - ang)))
        sector = np.abs(d) < np.deg2rad(38)
        pts = env[sector] + rng.normal(0, 0.008, (sector.sum(), 3))
        client._on_lidar(PointCloud2.from_numpy(pts.astype(float), frame_id="world", timestamp=float(i)))
        counts.append(len(client.snapshot()))

    snap = client.snapshot()

    # --- machine-checkable invariants -------------------------------------
    assert snap.ndim == 2 and snap.shape[1] == 3, f"expected (N, 3), got {snap.shape}"
    assert np.issubdtype(snap.dtype, np.floating)
    assert client.message_count == n_msgs
    assert counts[-1] > counts[1], "snapshot did not grow as messages arrived"
    # The accumulated cloud should span roughly the full 6x4 m room.
    span_x = snap[:, 0].max() - snap[:, 0].min()
    span_y = snap[:, 1].max() - snap[:, 1].min()
    assert span_x > 5.0 and span_y > 3.0, f"cloud too small: {span_x:.1f} x {span_y:.1f} m"

    # --- review artifact --------------------------------------------------
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(11.6, 4.2), gridspec_kw={"width_ratios": [1.15, 1]})
    ax1.fill_between(range(len(counts)), counts, color="#2a78d6", alpha=0.14, step="post")
    ax1.step(range(len(counts)), counts, where="post", color="#2a78d6", linewidth=2.0)
    ax1.annotate(f"{counts[-1]:,} pts", (len(counts) - 1, counts[-1]), xytext=(-8, -16),
                 textcoords="offset points", ha="right", fontsize=10, fontweight="bold",
                 family="monospace")
    ax1.set_title("snapshot() size while a scanner spins one revolution")
    ax1.set_xlabel("/lidar messages received")
    ax1.set_ylabel("points in snapshot()")
    ax1.set_xlim(0, len(counts) - 1)
    ax1.set_ylim(0, counts[-1] * 1.12)

    ax2.scatter(snap[:, 0], snap[:, 1], s=2.0, c=snap[:, 2], cmap="Blues_r", alpha=0.8, linewidths=0)
    ax2.scatter([0], [0], marker="^", s=70, color="#131a24", zorder=5)
    ax2.set_title("accumulated (N, 3) cloud, top-down (z as shade)")
    ax2.set_xlabel("x (m)")
    ax2.set_ylabel("y (m)")
    ax2.set_aspect("equal", adjustable="box")

    fig.tight_layout(pad=1.4)
    fig.savefig(skill_output.produced("lidar_accumulation.png"), dpi=115)
    plt.close(fig)
