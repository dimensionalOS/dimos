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

"""Let the agent write Python against the live memory recording.

The robot's Recorder is writing a SQLite store as it drives; this skill opens
that same db read-side and execs agent-authored code against it. Variables
persist between calls, so the agent can build a map in one call and query it in
the next.
"""

from __future__ import annotations

import contextlib
import io
import traceback
from typing import Any, ClassVar, Protocol

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.spec.utils import Spec
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RecordingSpec(Spec, Protocol):
    """The Recorder RPC this skill needs: where is the live db."""

    @rpc
    def recording_path(self) -> str: ...


class MemoryQuerySkillConfig(ModuleConfig):
    # Truncation guard: a careless print of a full cloud is ~29k points.
    max_output_chars: int = PointCloud2.ENCODE_SOFT_CAP


class MemoryQuerySkill(Module):
    # Agent code runs synchronously and can take seconds (fusing a map), so it
    # gets its own worker rather than stalling whatever it shares one with.
    dedicated_worker: ClassVar[bool] = True

    config: MemoryQuerySkillConfig
    _recorder: RecordingSpec

    _ns: dict[str, Any] | None = None

    @rpc
    def start(self) -> None:
        super().start()

    @rpc
    def stop(self) -> None:
        self._close_store()
        super().stop()

    def _close_store(self) -> None:
        store = (self._ns or {}).get("store")
        if store is not None:
            with contextlib.suppress(Exception):
                store.stop()
        self._ns = None

    def _open_store(self) -> Any:
        from dimos.memory.store.sqlite import SqliteStore

        path = self._recorder.recording_path()
        store = SqliteStore(path=path, must_exist=True)
        store.start()
        logger.info("MemoryQuerySkill reading %s", path)
        return store

    def _namespace(self) -> dict[str, Any]:
        """The persistent exec namespace, seeded on first use."""
        if self._ns is not None:
            return self._ns

        import numpy as np

        from dimos.mapping.voxels.module import VoxelMapTransformer
        from dimos.memory.transform import downsample
        from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2

        def reopen() -> Any:
            """Re-open the store — call this to pick up streams added since."""
            self._close_store()
            return self._namespace()["store"]

        self._ns = {
            "store": self._open_store(),
            "PointCloud2": PointCloud2,
            "VoxelMapTransformer": VoxelMapTransformer,
            "downsample": downsample,
            "np": np,
            "reopen": reopen,
        }
        return self._ns

    @skill
    def run_python(self, code: str) -> str:
        """Run Python against the robot's live sensor recording and return what it prints.

        Everything the robot records — lidar pointclouds, camera frames, odometry —
        goes into a memory store you query here. Use this whenever a question needs
        actual sensor data rather than a guess. End your code with print().

        Names already in scope (do not re-import): ``store``, ``PointCloud2``,
        ``VoxelMapTransformer``, ``downsample``, ``np``, ``reopen``. Anything you
        assign persists into your next call, so build a map once and reuse it.

        Start here — what is recorded, and how much::

            print(store.list_streams())     # e.g. ['lidar', 'color_image', 'odom', 'tf']
            print(store.summary())          # per stream: item count, duration, rate, size

        A stream is lazy: filters chain, nothing loads until a terminal
        (.last() .first() .to_list() .count()). Each returns an Observation with
        .data (the payload), .ts (epoch seconds), .pose (robot pose), .tags::

            lidar = store.streams.lidar
            cloud = lidar.last().data                  # newest sweep, a PointCloud2
            print(cloud.agent_encode())                # <- your main readout

        agent_encode() is how you read a pointcloud: a compact summary of what the
        lidar measured. Its format is described once, in
        ``PointCloud2.AGENT_ENCODE_LEGEND`` — print that first, then read the
        summary against it. If the answer to your question isn't in there, compute
        it from the raw points (below).

        Time windows. ``range_time``/``from_time``/``to_time`` are seconds from the
        start of the recording; ``at``/``to_timestamp`` take absolute epoch seconds::

            now = lidar.last().ts
            past   = lidar.at(now - 10.0, tolerance=0.5).last().data   # sweep 10s ago
            recent = lidar.from_timestamp(now - 30.0)                  # last 30 seconds
            window = lidar.range_time(20, 30)                          # 20s..30s in
            print(window.count(), "frames")

        Fuse many sweeps into one map (emit_every=0 = accumulate, yield once at the
        end). carve_columns=False keeps it purely additive, which is what you want
        when comparing two maps::

            fused = lidar.transform(
                VoxelMapTransformer(voxel_size=0.05, device="CPU:0", emit_every=0)
            ).last().data
            print(fused.agent_encode())

        Big windows are slow — thin the stream first with ``.transform(downsample(5))``
        (every 5th frame) or raise voxel_size.

        Where the robot has been, and what is near a place::

            here = store.streams.odom.last().data
            print(here.position)
            print(lidar.near(here, radius=2.0).count(), "sweeps taken within 2m")

        Work on the raw points for anything agent_encode() omits — pts is (N, 3)
        float32 world-frame xyz::

            pts = cloud.points_f32()
            ground = pts[pts[:, 2] < 0.15]
            print("ground z spread", round(float(ground[:, 2].ptp()), 3), "m")

        Other cloud helpers: ``cloud.filter_by_height(min_height=, max_height=)``,
        ``cloud.voxel_downsample(0.05)``, ``len(cloud)``, ``cloud_a + cloud_b``.

        Comparing two moments — quantize to voxels and set-difference the keys::

            def keys(c, v=0.05):
                idx = np.floor(c.points_f32() / np.float32(v)).astype(np.int64)
                return np.unique(idx @ np.array([1 << 42, 1 << 21, 1]))
            added = np.setdiff1d(keys(map_now), keys(map_then))
            print(len(added), "new voxels")

        Output is truncated, so print summaries and counts, never whole arrays.
        Errors come back as a traceback — read it and retry.

        Args:
            code: Python source to execute. Print what you want to see.
        """
        try:
            ns = self._namespace()
        except Exception as exc:  # store missing, recorder not up yet
            return f"memory store unavailable: {type(exc).__name__}: {exc}"

        buf = io.StringIO()
        try:
            with contextlib.redirect_stdout(buf):
                exec(code, ns)
        except Exception:
            return self._cap(buf.getvalue() + traceback.format_exc())

        out = buf.getvalue()
        return self._cap(out) if out.strip() else "(no output — end your code with print())"

    def _cap(self, text: str) -> str:
        limit = self.config.max_output_chars
        if len(text) <= limit:
            return text
        return text[:limit] + f"\n... [truncated, {len(text)} chars total]"


memory_query_skill = MemoryQuerySkill.blueprint
