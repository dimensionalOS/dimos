"""RGBDCameraRecorder — Recorder + continuous CLIP embed for RGBD streams.

Two things this module does on top of the base ``Recorder`` pattern

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
        #
        # depth_image MUST use a lossless codec. memory2's codec auto-dispatch
        # maps every `Image`-typed stream to JpegCodec (lossy 8-bit DCT) — correct
        # for color RGB, catastrophic for uint16 depth (millimeters get shredded,
        # then 3D projection finds no coherent points and find_objects returns
        # nothing). We use memory2's documented per-stream `codec=` override
        # (store/base.py: "Per-stream overrides ... codec=") to record depth as
        # lossless lz4+lcm (raw Image.lcm_encode, no pixel loss). Color stays
        # JPEG — lossy is fine for RGB and keeps the DB small.
        for name, port in self.inputs.items():
            if name == "depth_image":
                stream: MemoryStream[Any] = self.store.stream(name, port.type, codec="lz4+lcm")
            else:
                stream = self.store.stream(name, port.type)
            self._port_to_stream(name, port, stream)
            logger.info("Recording %s (%s)", name, port.type.__name__)

        # Continuous CLIP embed — same store, same notifier, .live() works.
        self._embedding_model = self.register_disposable(self.config.embedding_model())
        self._embedding_model.start()
        self._embeddings = self.store.stream("color_image_embedded", Image)

        # Find the timestamp of the last embedded frame so we don't re-embed history
        try:
            last_embedded_ts = self._embeddings.last().ts
        except LookupError:
            last_embedded_ts = 0.0

        # fmt: off
        self.store.streams.color_image \
            .after(last_embedded_ts) \
            .live() \
            .filter(lambda obs: obs.data.brightness > 0.1) \
            .transform(QualityWindow(lambda img: img.sharpness, window=0.5)) \
            .transform(EmbedImages(self._embedding_model, batch_size=2)) \
            .save(self._embeddings) \
            .drain_thread()
        # fmt: on
        logger.info("RGBDCameraRecorder: continuous CLIP embed pipeline active")
