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

from __future__ import annotations

from pathlib import Path
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.global_config import global_config
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import LCMTransport
from dimos.msgs.sensor_msgs.Image import Image, sharpness_barrier
from dimos.msgs.vision_msgs.Detection2DArray import Detection2DArray
from dimos.perception.detection.detectors.yoloe import Yoloe2DDetector, YoloePromptMode
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import (
    rerun_config as go2_rerun_config,
    unitree_go2_basic,
)
from dimos.utils.data import get_data_dir
from dimos.utils.logging_config import setup_logger
from dimos.utils.reactive import backpressure
from dimos.visualization.vis_module import vis_module

logger = setup_logger()

_YOLOE_MODEL_DIR_NAME = "models_yoloe"
_YOLOE_LRPC_MODEL_NAME = "yoloe-11s-seg-pf.pt"
_YOLOE_DETECTIONS_TOPIC = "/color_image/yoloe_detections"
_YOLOE_DETECTIONS_ENTITY = "world/color_image/yoloe_detections"


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


def _yoloe_detections_to_rerun(detections: Detection2DArray) -> Any:
    import rerun as rr

    boxes: list[list[float]] = []
    labels: list[str] = []
    colors: list[list[int]] = []

    for index, detection in enumerate(detections.detections):
        bbox = detection.bbox
        center = bbox.center.position
        x = float(center.x - bbox.size_x / 2.0)
        y = float(center.y - bbox.size_y / 2.0)
        width = float(bbox.size_x)
        height = float(bbox.size_y)
        detection_id = str(detection.id) if detection.id else str(index)

        boxes.append([x, y, width, height])
        labels.append(detection_id)
        colors.append([0, 220, 255, 220])

    return rr.Boxes2D(
        array=boxes,
        array_format=rr.Box2DFormat.XYWH,
        colors=colors,
        labels=labels,
        show_labels=True,
        draw_order=95.0,
    )


def _yoloe_tracking_rerun_blueprint() -> Any:
    import rerun as rr
    import rerun.blueprint as rrb

    return rrb.Blueprint(
        rrb.Horizontal(
            rrb.Spatial2DView(
                origin="world/color_image",
                contents=["world/color_image/**"],
                name="Camera",
            ),
            rrb.Spatial3DView(
                origin="world",
                contents=[
                    "world/**",
                    f"-{_YOLOE_DETECTIONS_ENTITY}",
                ],
                name="3D",
                background=rrb.Background(kind="SolidColor", color=[0, 0, 0]),
                line_grid=rrb.LineGrid3D(
                    plane=rr.components.Plane3D.XY.with_distance(0.5),
                ),
                overrides={
                    "world/lidar": rrb.EntityBehavior(visible=False),
                },
            ),
            column_shares=[1, 2],
        ),
        rrb.TimePanel(state="hidden"),
        rrb.SelectionPanel(state="hidden"),
    )


_yoloe_tracking_rerun_config = {
    **go2_rerun_config,
    "blueprint": _yoloe_tracking_rerun_blueprint,
    "visual_override": {
        **go2_rerun_config["visual_override"],
        _YOLOE_DETECTIONS_ENTITY: _yoloe_detections_to_rerun,
    },
}

_yoloe_tracking_vis = vis_module(
    viewer_backend=global_config.viewer,
    rerun_config=_yoloe_tracking_rerun_config,
)


yoloe_tracking_test = (
    autoconnect(
        unitree_go2_basic,
        _yoloe_tracking_vis,
        YoloeTrackingModule.blueprint(),
    )
    .global_config(
        n_workers=6,
        robot_model="unitree_go2",
    )
    .transports(
        {
            ("detections", Detection2DArray): LCMTransport(
                _YOLOE_DETECTIONS_TOPIC,
                Detection2DArray,
            ),
        }
    )
    .requirements(
        _require_yoloe_lrpc_model,
    )
)


__all__ = [
    "YoloeTrackingConfig",
    "YoloeTrackingModule",
    "yoloe_tracking_test",
]


if __name__ == "__main__":
    ModuleCoordinator.build(yoloe_tracking_test).loop()
