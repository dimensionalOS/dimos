from __future__ import annotations  # 允许类型注解延迟解析，减少循环导入风险

from pathlib import Path
from typing import Any

from dimos.core.core import rpc  # 导入 rpc 装饰器，让方法可通过 DimOS RPC 调用
from dimos.core.module import Module, ModuleConfig  # 导入模块基类和模块配置基类
from dimos.core.stream import In, Out  # 导入输入输出流类型
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.utils.data import get_data_dir
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure

logger = setup_logger()

_YOLOE_MODEL_DIR_NAME = "models_yoloe"
_YOLOE_LRPC_MODEL_NAME = "yoloe-11s-seg-pf.pt"


def _local_yoloe_model_path(
    model_path: str = _YOLOE_MODEL_DIR_NAME,
    model_name: str | None = None,
) -> Path:
    return get_data_dir(model_path) / (model_name or _YOLOE_LRPC_MODEL_NAME)


def _format_missing_yoloe_model_error(model_path: Path) -> str:
    return (
        f"Missing local YOLOE model: {model_path}. "
        "This blueprint runs offline and will not pull model data at startup. "
        "Prepare the model in an online environment first: "
        "git lfs pull --include data/.lfs/models_yoloe.tar.gz && "
        "uv run python -c 'from dimos.utils.data import get_data; print(get_data(\"models_yoloe\"))' && "
        f"ls -lh {model_path}"
    )


def _require_yoloe_lrpc_model() -> str | None:
    """Blueprint requirement check: verify the YOLOE model file is pre-downloaded."""
    model_path = _local_yoloe_model_path()
    if model_path.exists():
        return None
    return _format_missing_yoloe_model_error(model_path)


class YoloeTrackingConfig(ModuleConfig):
    max_freq: float = 10.0
    model_path: str = _YOLOE_MODEL_DIR_NAME
    model_name: str | None = None
    device: str | None = None
    max_area_ratio: float | None = 0.3


class YoloeTrackingModule(Module):
    """Run offline YOLOE tracking on color_image and publish Detection2DArray."""

    config: YoloeTrackingConfig

    color_image: In[Image]
    detections: Out[Detection2DArray]

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._detector: Yoloe2DDetector | None = None

    @rpc
    def start(self) -> None:
        super().start()

        model_path = _local_yoloe_model_path(self.config.model_path, self.config.model_name)
        if not model_path.exists():
            raise RuntimeError(_format_missing_yoloe_model_error(model_path))

        self._detector = Yoloe2DDetector(
            model_path=self.config.model_path,
            model_name=self.config.model_name or _YOLOE_LRPC_MODEL_NAME,
            device=self.config.device,
            prompt_mode=YoloePromptMode.LRPC,
            max_area_ratio=self.config.max_area_ratio,
        )

        stream = backpressure(
            self.color_image.pure_observable().pipe(sharpness_barrier(self.config.max_freq))
        )
        self.register_disposable(stream.subscribe(self._process_image))

    @rpc
    def stop(self) -> None:
        if self._detector is not None:
            self._detector.stop()
            self._detector = None
        super().stop()

    def _process_image(self, image: Image) -> None:
        if self._detector is None:
            return
        detections = self._detector.process_image(image)
        self.detections.publish(detections.to_ros_detection2d_array())


__all__ = [
    "YoloeTrackingConfig",
    "YoloeTrackingModule",
    "_require_yoloe_lrpc_model",
]
