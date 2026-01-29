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

from collections.abc import Generator
from contextlib import contextmanager
import os
from pathlib import Path
import shutil
import tempfile
from typing import TYPE_CHECKING, Any, TypedDict

import cv2
from hydra.utils import instantiate  # type: ignore[import-not-found]
import numpy as np
from numpy.typing import NDArray
from omegaconf import OmegaConf  # type: ignore[import-not-found]
from PIL import Image as PILImage
import torch

from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection.type import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from sam2.sam2_video_predictor import SAM2VideoPredictor  # type: ignore[import-untyped]

os.environ['TQDM_DISABLE'] = '1'

logger = setup_logger()


class SAM2InferenceState(TypedDict):
    images: list[torch.Tensor | None]
    num_frames: int
    cached_features: dict[int, Any]


class EdgeTAMProcessor(Detector):
    _predictor: "SAM2VideoPredictor"
    _inference_state: SAM2InferenceState | None
    _frame_count: int
    _is_tracking: bool
    _buffer_size: int

    def __init__(
        self,
    ) -> None:
        local_config_path = Path(__file__).parent / "configs" / "edgetam.yaml"

        if not local_config_path.exists():
            raise FileNotFoundError(f"EdgeTAM config not found at {local_config_path}")

        if not torch.cuda.is_available():
            raise RuntimeError("EdgeTAM requires a CUDA-capable GPU")

        cfg = OmegaConf.load(local_config_path)

        overrides = {
            "model.sam_mask_decoder_extra_args.dynamic_multimask_via_stability": True,
            "model.sam_mask_decoder_extra_args.dynamic_multimask_stability_delta": 0.05,
            "model.sam_mask_decoder_extra_args.dynamic_multimask_stability_thresh": 0.98,
            "model.binarize_mask_from_pts_for_mem_enc": True,
            "model.fill_hole_area": 8,
        }

        for key, value in overrides.items():
            OmegaConf.update(cfg, key, value)

        if cfg.model._target_ != "sam2.sam2_video_predictor.SAM2VideoPredictor":
            logger.warning(
                f"Config target is {cfg.model._target_}, forcing SAM2VideoPredictor"
            )
            cfg.model._target_ = "sam2.sam2_video_predictor.SAM2VideoPredictor"

        self._predictor = instantiate(cfg.model, _recursive_=True)

        ckpt_path = str(get_data("models_edgetam") / "edgetam.pt")

        sd = torch.load(ckpt_path, map_location="cpu", weights_only=True)["model"]
        missing_keys, unexpected_keys = self._predictor.load_state_dict(sd)
        if missing_keys:
            raise RuntimeError("Missing keys in checkpoint")
        if unexpected_keys:
            raise RuntimeError("Unexpected keys in checkpoint")

        self._predictor = self._predictor.to("cuda")
        self._predictor.eval()

        self._inference_state = None
        self._frame_count = 0
        self._is_tracking = False
        self._buffer_size = 100  # Keep last N frames in memory to avoid OOM

    def _prepare_frame(self, image: Image) -> torch.Tensor:
        """Prepare frame for SAM2 (resize, normalize, convert to tensor)."""

        cv_image = image.to_opencv()
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        pil_image = PILImage.fromarray(rgb_image)

        img_np = np.array(
            pil_image.resize((self._predictor.image_size, self._predictor.image_size))
        )
        img_np = img_np.astype(np.float32) / 255.0

        img_mean = np.array([0.485, 0.456, 0.406], dtype=np.float32).reshape(1, 1, 3)
        img_std = np.array([0.229, 0.224, 0.225], dtype=np.float32).reshape(1, 1, 3)
        img_np -= img_mean
        img_np /= img_std

        img_tensor = torch.from_numpy(img_np).permute(2, 0, 1).float()
        img_tensor = img_tensor.cuda()

        return img_tensor

    def init_track(
        self,
        image: Image,
        points: NDArray[np.floating[Any]] | None = None,
        labels: NDArray[np.integer[Any]] | None = None,
        box: NDArray[np.floating[Any]] | None = None,
        obj_id: int = 1,
    ) -> ImageDetections2D:
        """Initialize tracking with a prompt (points or box).

        Args:
            image: Initial frame to start tracking from
            points: Point prompts for segmentation (Nx2 array of [x, y] coordinates)
            labels: Labels for points (1 = foreground, 0 = background)
            box: Bounding box prompt in [x1, y1, x2, y2] format
            obj_id: Object ID for tracking

        Returns:
            ImageDetections2D with initial segmentation mask
        """
        if self._inference_state is not None:
            self.stop()

        self._frame_count = 0

        with _temp_dir_context(image) as video_path:
            self._inference_state = self._predictor.init_state(video_path=video_path)

        self._predictor.reset_state(self._inference_state)

        if torch.is_tensor(self._inference_state["images"]):
            self._inference_state["images"] = [self._inference_state["images"][0]]

        self._is_tracking = True

        if points is not None:
            points = points.astype(np.float32)
        if labels is not None:
            labels = labels.astype(np.int32)
        if box is not None:
            box = box.astype(np.float32)

        with torch.no_grad():
            _, out_obj_ids, out_mask_logits = self._predictor.add_new_points_or_box(
                inference_state=self._inference_state,
                frame_idx=0,
                obj_id=obj_id,
                points=points,
                labels=labels,
                box=box,
            )

        return self._process_results(image, out_obj_ids, out_mask_logits)

    def process_image(self, image: Image) -> ImageDetections2D:
        """Process a new video frame and propagate tracking.

        Args:
            image: New frame to process

        Returns:
            ImageDetections2D with tracked object segmentation masks
        """
        if not self._is_tracking or self._inference_state is None:
            return ImageDetections2D(image=image)

        self._frame_count += 1

        # Append new frame to inference state
        new_frame_tensor = self._prepare_frame(image)
        self._inference_state["images"].append(new_frame_tensor)
        self._inference_state["num_frames"] += 1

        # Memory management
        cached_features = self._inference_state["cached_features"]
        if len(cached_features) > self._buffer_size:
            oldest_frame = min(cached_features.keys())
            if oldest_frame < self._frame_count - self._buffer_size:
                del cached_features[oldest_frame]

        if len(self._inference_state["images"]) > self._buffer_size + 10:
            idx_to_drop = self._frame_count - self._buffer_size - 5
            if idx_to_drop >= 0 and idx_to_drop < len(self._inference_state["images"]):
                if self._inference_state["images"][idx_to_drop] is not None:
                    self._inference_state["images"][idx_to_drop] = None

        detections: ImageDetections2D = ImageDetections2D(image=image)

        with torch.no_grad():
            for out_frame_idx, out_obj_ids, out_mask_logits in self._predictor.propagate_in_video(
                self._inference_state, start_frame_idx=self._frame_count, max_frame_num_to_track=1
            ):
                if out_frame_idx == self._frame_count:
                    return self._process_results(image, out_obj_ids, out_mask_logits)

        return detections

    def _process_results(
        self,
        image: Image,
        obj_ids: list[int],
        mask_logits: torch.Tensor | NDArray[np.floating[Any]],
    ) -> ImageDetections2D:
        detections: ImageDetections2D = ImageDetections2D(image=image)

        if len(obj_ids) == 0:
            return detections

        if isinstance(mask_logits, torch.Tensor):
            mask_logits = mask_logits.cpu().numpy()

        for i, obj_id in enumerate(obj_ids):
            mask = mask_logits[i]
            seg = Detection2DSeg.from_sam2_result(
                mask=mask,
                obj_id=obj_id,
                image=image,
                name="object",
            )

            if seg.is_valid():
                detections.detections.append(seg)

        return detections

    def stop(self) -> None:
        self._is_tracking = False
        self._inference_state = None


@contextmanager
def _temp_dir_context(image: Image) -> Generator[str, None, None]:
    path = tempfile.mkdtemp()

    image.save(f"{path}/00000.jpg")

    try:
        yield path
    finally:
        shutil.rmtree(path)
