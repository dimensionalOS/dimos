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
from contextlib import contextmanager, nullcontext
from functools import lru_cache
from pathlib import Path
import shutil
import tempfile
from typing import TYPE_CHECKING, Any, TypedDict

from hydra.utils import instantiate
import numpy as np
from numpy.typing import NDArray
from omegaconf import OmegaConf
from PIL import Image as PILImage
import torch

from dimos.models.base import default_torch_device
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.detectors.base import Detector
from dimos.perception.detection.type.detection2d.bbox import Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.utils.data import get_data
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from sam2.sam2_video_predictor import SAM2VideoPredictor


logger = setup_logger()

# Hole/sprinkle cleanup in SAM2 needs the optional CUDA extension sam2._C.
# Requesting it without _C triggers a per-call UserWarning and then skips.
_FILL_HOLE_AREA = 8
_MIN_MASK_REGION_AREA = 150


@lru_cache(maxsize=1)
def _sam2_cuda_postprocess_available() -> bool:
    try:
        from sam2 import _C  # noqa: F401
    except ImportError:
        return False
    return True


class SAM2InferenceState(TypedDict):
    images: list[torch.Tensor | None]
    num_frames: int
    cached_features: dict[int, Any]


def _build_model() -> "SAM2VideoPredictor":
    """Build the EdgeTAM SAM2 model from the local config + checkpoint."""
    local_config_path = Path(__file__).parent / "configs" / "edgetam.yaml"

    if not local_config_path.exists():
        raise FileNotFoundError(f"EdgeTAM config not found at {local_config_path}")

    device = default_torch_device()
    if device == "cpu":
        raise RuntimeError("EdgeTAM requires a CUDA or MPS device")

    cfg = OmegaConf.load(local_config_path)

    fill_hole_area = _FILL_HOLE_AREA if _sam2_cuda_postprocess_available() else 0
    overrides = {
        "model.sam_mask_decoder_extra_args.dynamic_multimask_via_stability": True,
        "model.sam_mask_decoder_extra_args.dynamic_multimask_stability_delta": 0.05,
        "model.sam_mask_decoder_extra_args.dynamic_multimask_stability_thresh": 0.98,
        "model.binarize_mask_from_pts_for_mem_enc": True,
        "model.fill_hole_area": fill_hole_area,
    }

    for key, value in overrides.items():
        OmegaConf.update(cfg, key, value)

    if cfg.model._target_ != "sam2.sam2_video_predictor.SAM2VideoPredictor":
        logger.warning(f"Config target is {cfg.model._target_}, forcing SAM2VideoPredictor")
        cfg.model._target_ = "sam2.sam2_video_predictor.SAM2VideoPredictor"

    predictor: SAM2VideoPredictor = instantiate(cfg.model, _recursive_=True)

    # Suppress the per-frame "propagate in video" tqdm bar from sam2
    import sam2.sam2_video_predictor as _svp

    _svp.tqdm = lambda iterable, *a, **kw: iterable

    ckpt_path = str(get_data("models_edgetam") / "edgetam.pt")

    sd = torch.load(ckpt_path, map_location="cpu", weights_only=True)["model"]
    missing_keys, unexpected_keys = predictor.load_state_dict(sd)
    if missing_keys:
        raise RuntimeError("Missing keys in checkpoint")
    if unexpected_keys:
        raise RuntimeError("Unexpected keys in checkpoint")

    predictor = predictor.to(device)
    predictor.eval()
    return predictor


class EdgeTAMImageSegmenter:
    """Box-prompted single-image segmentation using the EdgeTAM checkpoint."""

    def __init__(self) -> None:
        from sam2.sam2_image_predictor import SAM2ImagePredictor

        self._predictor = SAM2ImagePredictor(_build_model())
        self._mask_generators: dict[int, Any] = {}

    def propose_all(
        self,
        image: Image,
        points_per_side: int = 24,
        pred_iou_thresh: float = 0.55,
        stability_score_thresh: float = 0.85,
        min_mask_region_area: int = _MIN_MASK_REGION_AREA,
    ) -> ImageDetections2D:
        """Class-agnostic mask proposals over the whole image, no vocabulary.

        Wraps ``SAM2AutomaticMaskGenerator`` on the shared EdgeTAM model.
        Proposals carry no names ("proposal") and ``confidence`` is the
        predicted mask IoU.
        """
        import cv2
        from sam2.automatic_mask_generator import SAM2AutomaticMaskGenerator

        if not _sam2_cuda_postprocess_available():
            min_mask_region_area = 0

        generator = self._mask_generators.get(points_per_side)
        if generator is None:
            generator = SAM2AutomaticMaskGenerator(
                model=self._predictor.model,
                points_per_side=points_per_side,
                pred_iou_thresh=pred_iou_thresh,
                stability_score_thresh=stability_score_thresh,
                min_mask_region_area=min_mask_region_area,
            )
            self._mask_generators[points_per_side] = generator

        rgb = cv2.cvtColor(image.to_opencv(), cv2.COLOR_BGR2RGB)
        amp = (
            torch.autocast("cuda", dtype=torch.bfloat16)
            if self._predictor.device.type == "cuda"
            else nullcontext()
        )
        with torch.no_grad(), amp:
            proposals = generator.generate(rgb)

        detections: list[Detection2DBBox] = [
            Detection2DSeg.from_sam2_result(
                proposal["segmentation"],
                i,
                image,
                name="proposal",
                confidence=float(proposal.get("predicted_iou", 1.0)),
            )
            for i, proposal in enumerate(proposals)
        ]
        return ImageDetections2D(image, [det for det in detections if det.is_valid()])

    def segment(self, detections: ImageDetections2D) -> ImageDetections2D:
        """Refine box detections into mask detections (Detection2DSeg)."""
        import cv2

        if not len(detections):
            return detections

        image = detections.image
        rgb = cv2.cvtColor(image.to_opencv(), cv2.COLOR_BGR2RGB)

        # bfloat16 autocast is CUDA-only; MPS runs in float32
        amp = (
            torch.autocast("cuda", dtype=torch.bfloat16)
            if self._predictor.device.type == "cuda"
            else nullcontext()
        )
        with torch.no_grad(), amp:
            self._predictor.set_image(rgb)
            boxes = np.array([det.bbox for det in detections], dtype=np.float32)
            masks, _, _ = self._predictor.predict(box=boxes, multimask_output=False)

        masks = masks.reshape(-1, *masks.shape[-2:])  # (N, H, W) regardless of batch dim
        segmented: list[Detection2DBBox] = [
            Detection2DSeg.from_sam2_result(
                mask,
                det.track_id,
                image,
                class_id=det.class_id,
                name=det.name,
                confidence=det.confidence,
            )
            for det, mask in zip(detections, masks, strict=False)
        ]
        return ImageDetections2D(image, segmented)


class EdgeTAMProcessor(Detector):
    _predictor: "SAM2VideoPredictor"
    _inference_state: SAM2InferenceState | None
    _frame_count: int
    _is_tracking: bool
    _buffer_size: int

    def __init__(
        self,
    ) -> None:
        self._predictor = _build_model()
        self._inference_state = None
        self._frame_count = 0
        self._is_tracking = False
        self._buffer_size = 100  # Keep last N frames in memory to avoid OOM

    def _prepare_frame(self, image: Image) -> torch.Tensor:
        """Prepare frame for SAM2 (resize, normalize, convert to tensor)."""
        import cv2

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
        img_tensor = img_tensor.to(self._predictor.device)

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
