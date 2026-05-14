"""RGBDCameraRecorder — Recorder + continuous CLIP embed for RGBD streams.

Two things this module does on top of the base ``Recorder`` pattern at
``dimos/robot/unitree/go2/blueprints/smart/unitree_go2.py:49-53``:

1. **Continuous CLIP embed pipeline.** Same logic as ``memory2.SemanticSearch``,
   but co-located here so it shares the recorder's store. See ``spec.py`` for
   the "why in one module" reasoning (memory2's SubjectNotifier is per-store).

2. **Resume-if-exists semantics.** ``start()`` overrides ``Recorder.start()``
   to skip the overwrite-or-fail gate. If ``recording.db`` exists, we open it
   and append (cross-session memory). If it doesn't, SQLite creates it fresh.
   No flag flipping, no manual file deletion between runs. Users wanting a
   clean slate delete the file themselves.
"""

from __future__ import annotations

from typing import Any

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.manipulation.memory2.spec import RGBDCameraRecorderConfig
from dimos.memory2.embed import EmbedImages
from dimos.memory2.module import Recorder
from dimos.memory2.stream import Stream as MemoryStream
from dimos.memory2.transform import QualityWindow
from dimos.models.embedding.base import EmbeddingModel
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class RGBDCameraRecorder(Recorder):
    """Records RGBD streams (color + depth + intrinsics) and continuously
    CLIP-embeds qualifying color frames into ``color_image_embedded``."""

    color_image: In[Image]
    depth_image: In[Image]
    camera_info: In[CameraInfo]

    config: RGBDCameraRecorderConfig

    _embedding_model: EmbeddingModel | None = None
    _embeddings: MemoryStream[Any] | None = None

    @rpc
    def start(self) -> None:

        super(Recorder, self).start()

        if self.config.g.replay:
            logger.info(
                "Replay mode active — recording disabled, leaving %s untouched",
                self.config.db_path,
            )
            return

        if not self.inputs:
            logger.warning("RGBDCameraRecorder has no In ports — nothing to record")
            return

        # Replicate Recorder.start()'s port-to-stream registration. Touching
        # self.store lazily opens the SqliteStore at db_path (existing file or new).
        for name, port in self.inputs.items():
            stream: MemoryStream[Any] = self.store.stream(name, port.type)
            self._port_to_stream(name, port, stream)
            logger.info("Recording %s (%s)", name, port.type.__name__)

        # Continuous CLIP embed — same store, same notifier, .live() works.
        self._embedding_model = self.register_disposable(self.config.embedding_model())
        self._embedding_model.start()
        self._embeddings = self.store.stream("color_image_embedded", Image)

        # fmt: off
        self.store.streams.color_image \
            .live() \
            .filter(lambda obs: obs.data.brightness > 0.1) \
            .transform(QualityWindow(lambda img: img.sharpness, window=0.5)) \
            .transform(EmbedImages(self._embedding_model, batch_size=2)) \
            .save(self._embeddings) \
            .drain_thread()
        # fmt: on
        logger.info("RGBDCameraRecorder: continuous CLIP embed pipeline active")
