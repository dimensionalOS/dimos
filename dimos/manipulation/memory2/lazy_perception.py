"""LazyPerceptionModule — agent-callable open-vocab perception.

Three @skill methods, each a one-line composition of memory2 query
primitives. See ``spec.py`` for the architecture docstring.
"""

from __future__ import annotations

from collections.abc import Callable
import time
from typing import Any

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.stream import Out
from dimos.manipulation.memory2.spec import LazyPerceptionModuleConfig
from dimos.memory2.module import MemoryModule
from dimos.models.embedding.base import EmbeddingModel
from dimos.models.vl.base import VlModel
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.perception.detection.type.detection3d.object import Object as DetObject
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def _relative_time(ts: float) -> str:
    """Format a past timestamp as a human-readable age."""
    delta = max(0.0, time.time() - ts)
    if delta < 60:
        return f"{int(delta)}s ago"
    if delta < 3600:
        return f"{int(delta // 60)}min ago"
    return f"{int(delta // 3600)}h ago"


class LazyPerceptionModule(MemoryModule):
    """Lazy memory2-native open-vocab object detector.

    Three skills, each a one-line memory2 composition. Stateless:
    every call is an independent query → VLM (find_objects only)
    → 3D → publish.
    """

    config: LazyPerceptionModuleConfig

    objects: Out[list[DetObject]]

    _vlm: VlModel | None = None
    _clip: EmbeddingModel | None = None

    @rpc
    def start(self) -> None:
        super().start()
        self._vlm = self.register_disposable(self.config.vlm_provider())
        self._vlm.start()
        self._clip = self.register_disposable(self.config.embedding_model())
        self._clip.start()

    # ------------------------------------------------------------------ skills

    @skill
    def find_objects(self, prompt: str) -> str:
        """Find objects matching ``prompt``. Returns most recent confident
        match with timestamp.

        Open-vocab: ``prompt`` can be any natural language. Comma-separated
        prompts are split and processed per class because Moondream's
        ``query_detections`` labels every result with the literal query
        string.
        """
        prompt_list = [p.strip() for p in prompt.split(",") if p.strip()]
        if not prompt_list:
            self.objects.publish([])
            return "No prompts provided."

        all_objects: list[DetObject] = []
        most_recent_ts: float | None = None
        for single in prompt_list:
            objs, ts = self._find_and_project(
                single,
                build_query=lambda stream, vec: stream.search(vec),
            )
            all_objects.extend(objs)
            if ts is not None and (most_recent_ts is None or ts > most_recent_ts):
                most_recent_ts = ts

        self.objects.publish(all_objects)
        if not all_objects:
            return f"No confident '{prompt}' match in memory."

        age = _relative_time(most_recent_ts) if most_recent_ts is not None else "unknown age"
        lines = [self._fmt_object_line(o) for o in all_objects]
        return f"Found {len(all_objects)} object(s) matching '{prompt}' (seen {age}):\n" + "\n".join(lines)

    @skill
    def find_objects_near(
        self,
        prompt: str,
        x: float,
        y: float,
        z: float,
        radius: float = 1.0,
    ) -> str:
        """Find objects matching ``prompt`` in frames recorded when the
        camera was within ``radius`` meters of ``(x, y, z)``.

        ``.near()`` filters by the camera's pose at record time, NOT by
        the detected object's position. Note: applied as a Python
        post-filter after vector search; R*Tree pre-gating of the
        vector index is a memory2 follow-up.
        """
        prompt_list = [p.strip() for p in prompt.split(",") if p.strip()]
        if not prompt_list:
            self.objects.publish([])
            return "No prompts provided."

        pose = (x, y, z)
        all_objects: list[DetObject] = []
        most_recent_ts: float | None = None
        for single in prompt_list:
            objs, ts = self._find_and_project(
                single,
                build_query=lambda stream, vec: stream.near(pose, radius).search(vec),
            )
            all_objects.extend(objs)
            if ts is not None and (most_recent_ts is None or ts > most_recent_ts):
                most_recent_ts = ts

        self.objects.publish(all_objects)
        if not all_objects:
            return f"No confident '{prompt}' match near ({x:.2f}, {y:.2f}, {z:.2f}) within {radius}m."

        age = _relative_time(most_recent_ts) if most_recent_ts is not None else "unknown age"
        lines = [self._fmt_object_line(o) for o in all_objects]
        return (
            f"Found {len(all_objects)} object(s) matching '{prompt}' "
            f"near ({x:.2f}, {y:.2f}, {z:.2f}) within {radius}m (seen {age}):\n"
            + "\n".join(lines)
        )

    @skill
    def recall(self, name: str) -> str:
        """Where did I last see something matching ``name``?

        Cheaper than ``find_objects``: no VLM, no 3D projection. Returns
        the camera pose at the most recent confident semantic match plus
        timestamp. Works across process restarts because memory2's SQLite
        store is the persistence layer.
        """
        if self._clip is None:
            return f"No memory of '{name}'."
        try:
            vec = self._clip.embed_text(name)
            obs = (
                self.store.streams.color_image_embedded
                    .search(vec)
                    .filter(lambda o: (o.similarity or 0) >= self.config.min_similarity)
                    .order_by("ts", desc=True)
                    .first()
            )
        except (AttributeError, LookupError):
            return f"No memory of '{name}'."
        except Exception as e:
            logger.warning("recall(%r) failed: %s", name, e)
            return f"No memory of '{name}'."

        age = _relative_time(obs.ts)
        if obs.pose:
            x, y, z = obs.pose[0], obs.pose[1], obs.pose[2]
            return f"Last saw '{name}' with camera near ({x:.2f}, {y:.2f}, {z:.2f}) ({age})."
        return f"Last saw '{name}' (camera pose unknown) ({age})."

    # ----------------------------------------------------------- internal

    def _find_and_project(
        self,
        prompt: str,
        build_query: Callable[[Any, Any], Any],
    ) -> tuple[list[DetObject], float | None]:
        """Composed memory2 pipeline for ONE prompt class.

        Returns (objects, observation_ts). ``build_query`` is a callable
        ``(stream, query_vec) -> filtered Stream`` so find_objects /
        find_objects_near can share the rest of the pipeline.
        """
        if self._clip is None:
            return [], None
        try:
            vec = self._clip.embed_text(prompt)
            obs = (
                build_query(self.store.streams.color_image_embedded, vec)
                    .filter(lambda o: (o.similarity or 0) >= self.config.min_similarity)
                    .order_by("ts", desc=True)
                    .first()
            )
        except (AttributeError, LookupError):
            return [], None
        except Exception as e:
            logger.warning("_find_and_project(%r): %s", prompt, e)
            return [], None

        return self._detect_and_project_one(obs, prompt), obs.ts

    def _detect_and_project_one(self, color_obs: Any, prompt: str) -> list[DetObject]:
        """VLM detection + 3D projection for ONE peak frame, ONE prompt class."""
        if self._vlm is None:
            return []

        # Aligned depth + latest intrinsics. .first()/.last() raise LookupError on empty.
        try:
            depth_obs = self.store.streams.depth_image.at(
                color_obs.ts, tolerance=0.1
            ).first()
            info_obs = self.store.streams.camera_info.last()
        except LookupError:
            logger.warning("missing depth/info near ts=%.3f", color_obs.ts)
            return []
        except AttributeError:
            logger.warning("depth_image or camera_info stream not yet available")
            return []

        # VLM can hang (network) or OOM (CUDA). Treat failure as no-detection.
        try:
            dets_2d = self._vlm.query_detections(color_obs.data, prompt)
        except Exception as e:
            logger.warning("VLM failed (prompt=%r ts=%.3f): %s", prompt, color_obs.ts, e)
            return []
        if not dets_2d.detections:
            return []

        camera_transform = self._camera_transform_from_pose(color_obs.pose)
        try:
            # from_2d_to_list's annotation says ImageDetections2D[Detection2DSeg]
            # but the implementation at object.py:205-215 handles bbox-only
            # detections too (synthesizes a rectangular mask). Moondream returns
            # bbox-only — works at runtime.
            return DetObject.from_2d_to_list(
                detections_2d=dets_2d,  # type: ignore[arg-type]
                color_image=color_obs.data,
                depth_image=depth_obs.data,
                camera_info=info_obs.data,
                camera_transform=camera_transform,
                max_distance=self.config.max_distance,
                use_aabb=self.config.use_aabb,
                max_obstacle_width=self.config.max_obstacle_width,
            )
        except Exception as e:
            logger.warning("from_2d_to_list failed at ts=%.3f: %s", color_obs.ts, e)
            return []

    def _camera_transform_from_pose(self, pose: Any) -> Transform | None:
        """Build a Transform from the recorder's pose-stamped observation.

        Observation.pose is always a 7-tuple (x, y, z, qx, qy, qz, qw) or None
        (memory2/observationstore/sqlite.py:_decompose_pose).
        Object.from_2d_to_list(camera_transform=T) applies T to a camera-frame
        pointcloud to produce world-frame output — T is camera→world; the
        recorder's pose-in-world IS that. NO .inverse().
        """
        if pose is None:
            return None
        try:
            x, y, z, qx, qy, qz, qw = pose
        except (TypeError, ValueError):
            logger.warning("unexpected pose shape: %r", type(pose).__name__)
            return None
        return Transform(
            translation=Vector3(x, y, z),
            rotation=Quaternion(qx, qy, qz, qw),
        )

    @staticmethod
    def _fmt_object_line(o: DetObject) -> str:
        return f"  - {o.name} at ({o.center.x:.2f}, {o.center.y:.2f}, {o.center.z:.2f})"
