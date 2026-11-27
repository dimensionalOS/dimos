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

"""What the YOLO detectors share: the Ultralytics import, its weights and its results.

Ultralytics and its weights are AGPL-3.0. DimOS does not ship them. They come with
the `yolo` extra, and Ultralytics downloads the weights from its own servers.
"""

from __future__ import annotations

from collections.abc import Callable
from pathlib import Path
from tempfile import TemporaryDirectory
from typing import TYPE_CHECKING, Any
from zipfile import is_zipfile

from filelock import FileLock
import numpy as np

from dimos.constants import CACHE_DIR
from dimos.perception.detection.type.detection2d.bbox import Bbox, Detection2DBBox
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D
from dimos.perception.detection.type.detection2d.person import Detection2DPerson
from dimos.perception.detection.type.detection2d.seg import Detection2DSeg
from dimos.utils.data import get_data

if TYPE_CHECKING:
    from ultralytics.engine.results import Results

    from dimos.msgs.sensor_msgs.Image import Image

YOLO_INSTALL_HINT = (
    "The YOLO detectors need Ultralytics, which is not installed. Install the yolo "
    "extra: `uv sync --extra yolo --inexact`, or `pip install 'dimos[yolo]'`. "
    "Ultralytics is licensed under AGPL-3.0."
)


def import_ultralytics() -> Any:
    """The ultralytics package, or an error that names the extra to install."""
    try:
        # Not at the top of the file: it is optional, and it takes about 0.5 s.
        import ultralytics
    except ImportError as error:
        raise ImportError(YOLO_INSTALL_HINT) from error
    return ultralytics


def cache_path(name: str) -> Path:
    """Where a file for Ultralytics is kept.

    An absolute path stays as it is.
    """
    return CACHE_DIR / "ultralytics" / name


def checkpoint_path(name: str, model_path: str | Path | None = None) -> Path:
    """Where a checkpoint is. A missing official one is downloaded into the cache.

    A custom model directory, or a name with a directory in it, is used as supplied.
    Nothing is downloaded there.
    """
    path = get_data(model_path) / name if model_path is not None else cache_path(name)
    if model_path is not None or name != path.name:
        if not path.exists():
            raise FileNotFoundError(f"No checkpoint at {path}")
        return path

    # Not at the top of the file: Ultralytics is optional (the yolo extra).
    from ultralytics.utils.downloads import GITHUB_ASSETS_NAMES, attempt_download_asset

    # Only official checkpoints have a known format. A download from an older
    # version can have the ZIP header but no central directory.
    official = path.suffix == ".pt" and name in GITHUB_ASSETS_NAMES
    complete: Callable[[Path], bool] = is_zipfile if official else Path.exists
    if complete(path):
        return path

    # Ultralytics writes directly to the path it receives. A worker starting
    # during a download must only see the checkpoint after the download finishes.
    path.parent.mkdir(parents=True, exist_ok=True)
    with FileLock(path.with_name(f".{path.name}.lock")):
        if not complete(path):
            with TemporaryDirectory(dir=path.parent) as staging:
                downloaded = Path(attempt_download_asset(Path(staging) / path.name))
                if not complete(downloaded):
                    raise OSError(f"Could not download a complete checkpoint for {name}")
                downloaded.replace(path)
    return path


def detections_from_results(
    image: Image, results: list[Results]
) -> ImageDetections2D[Detection2DBBox]:
    """Create ImageDetections2D from ultralytics Results.

    Dispatches to appropriate Detection2D subclass based on result type:
    - If masks present: creates Detection2DSeg
    - If keypoints present: creates Detection2DPerson
    - Otherwise: creates Detection2DBBox

    Args:
        image: Source image
        results: List of ultralytics Results objects

    Returns:
        ImageDetections2D containing appropriate detection types
    """
    detections: list[Detection2DBBox] = []
    for result in results:
        if result.boxes is None:
            continue

        num_detections = len(result.boxes.xyxy)
        for i in range(num_detections):
            detection: Detection2DBBox
            if result.masks is not None:
                # Segmentation detection with mask
                detection = seg_from_result(result, i, image)
            elif result.keypoints is not None:
                # Pose detection with keypoints
                detection = person_from_result(result, i, image)
            else:
                # Regular bbox detection
                detection = bbox_from_result(result, i, image)
            if detection.is_valid():
                detections.append(detection)

    return ImageDetections2D(image=image, detections=detections)


def bbox_from_result(
    result: Results,
    idx: int,
    image: Image,
    *,
    detection_type: type[Detection2DBBox] = Detection2DBBox,
) -> Detection2DBBox:
    """Create Detection2DBBox from ultralytics Results object.

    Args:
        result: Ultralytics Results object containing detection data
        idx: Index of the detection in the results
        image: Source image

    Returns:
        Detection2DBBox instance
    """
    if result.boxes is None:
        raise ValueError("Result has no boxes")

    # Extract bounding box coordinates
    bbox_array = result.boxes.xyxy[idx].cpu().numpy()
    bbox: Bbox = (
        float(bbox_array[0]),
        float(bbox_array[1]),
        float(bbox_array[2]),
        float(bbox_array[3]),
    )

    # Extract confidence
    confidence = float(result.boxes.conf[idx].cpu())

    # Extract class ID and name
    class_id = int(result.boxes.cls[idx].cpu())
    name = (
        result.names.get(class_id, f"class_{class_id}")
        if hasattr(result, "names")
        else f"class_{class_id}"
    )

    # Extract track ID if available
    track_id = -1
    if hasattr(result.boxes, "id") and result.boxes.id is not None:
        track_id = int(result.boxes.id[idx].cpu())

    return detection_type(
        bbox=bbox,
        track_id=track_id,
        class_id=class_id,
        confidence=confidence,
        name=name,
        ts=image.ts,
        image=image,
    )


def person_from_result(
    result: Results,
    idx: int,
    image: Image,
    *,
    detection_type: type[Detection2DPerson] = Detection2DPerson,
) -> Detection2DPerson:
    """Create Detection2DPerson from ultralytics Results object with pose keypoints.

    Args:
        result: Ultralytics Results object containing detection and keypoint data
        idx: Index of the detection in the results
        image: Source image

    Returns:
        Detection2DPerson instance

    Raises:
        ValueError: If the result doesn't contain keypoints or is not a person detection
    """
    # Validate that this is a pose detection result
    if not hasattr(result, "keypoints") or result.keypoints is None:
        raise ValueError(
            "Cannot create Detection2DPerson from result without keypoints. "
            "This appears to be a regular detection result, not a pose detection. "
            "Use bbox_from_result() instead."
        )

    if not hasattr(result, "boxes") or result.boxes is None:
        raise ValueError("Cannot create Detection2DPerson from result without bounding boxes")

    # Check if this is actually a person detection (class 0 in COCO)
    class_id = int(result.boxes.cls[idx].cpu())
    if class_id != 0:  # Person is class 0 in COCO
        class_name = (
            result.names.get(class_id, f"class_{class_id}")
            if hasattr(result, "names")
            else f"class_{class_id}"
        )
        raise ValueError(
            f"Cannot create Detection2DPerson from non-person detection. "
            f"Got class {class_id} ({class_name}), expected class 0 (person)."
        )

    # Extract bounding box as tuple for Detection2DBBox
    bbox_array = result.boxes.xyxy[idx].cpu().numpy()

    bbox: Bbox = (
        float(bbox_array[0]),
        float(bbox_array[1]),
        float(bbox_array[2]),
        float(bbox_array[3]),
    )

    bbox_norm = result.boxes.xyxyn[idx].cpu().numpy() if hasattr(result.boxes, "xyxyn") else None

    confidence = float(result.boxes.conf[idx].cpu())
    class_id = int(result.boxes.cls[idx].cpu())

    # Extract keypoints
    if result.keypoints.xy is None or result.keypoints.conf is None:
        raise ValueError("Keypoints xy or conf data is missing from the result")

    keypoints = result.keypoints.xy[idx].cpu().numpy()
    keypoint_scores = result.keypoints.conf[idx].cpu().numpy()
    keypoints_norm = (
        result.keypoints.xyn[idx].cpu().numpy()
        if hasattr(result.keypoints, "xyn") and result.keypoints.xyn is not None
        else None
    )

    # Get image dimensions
    height, width = result.orig_shape

    # Extract track ID if available
    track_id = idx  # Use index as default
    if hasattr(result.boxes, "id") and result.boxes.id is not None:
        track_id = int(result.boxes.id[idx].cpu())

    # Get class name
    name = result.names.get(class_id, "person") if hasattr(result, "names") else "person"

    return detection_type(
        # Detection2DBBox fields
        bbox=bbox,
        track_id=track_id,
        class_id=class_id,
        confidence=confidence,
        name=name,
        ts=image.ts,
        image=image,
        # Person specific fields
        keypoints=keypoints,
        keypoint_scores=keypoint_scores,
        bbox_normalized=bbox_norm,
        keypoints_normalized=keypoints_norm,
        image_width=width,
        image_height=height,
    )


def seg_from_result(
    result: Results,
    idx: int,
    image: Image,
    *,
    detection_type: type[Detection2DSeg] = Detection2DSeg,
) -> Detection2DSeg:
    """Create Detection2DSeg from ultralytics Results object with segmentation mask.

    Args:
        result: Ultralytics Results object containing detection and mask data
        idx: Index of the detection in the results
        image: Source image

    Returns:
        Detection2DSeg instance
    """
    # Not at the top of the file: cv2 is heavy, see test_inline_heavy_imports.py.
    import cv2

    if result.boxes is None:
        raise ValueError("Result has no boxes")

    # Extract bounding box coordinates
    bbox_array = result.boxes.xyxy[idx].cpu().numpy()
    bbox: Bbox = (
        float(bbox_array[0]),
        float(bbox_array[1]),
        float(bbox_array[2]),
        float(bbox_array[3]),
    )

    # Extract confidence
    confidence = float(result.boxes.conf[idx].cpu())

    # Extract class ID and name
    class_id = int(result.boxes.cls[idx].cpu())
    if hasattr(result, "names") and result.names is not None:
        if isinstance(result.names, dict):
            name = result.names.get(class_id, f"class_{class_id}")
        elif isinstance(result.names, list) and class_id < len(result.names):
            name = result.names[class_id]
        else:
            name = f"class_{class_id}"
    else:
        name = f"class_{class_id}"

    # Extract track ID if available
    track_id = -1
    if hasattr(result.boxes, "id") and result.boxes.id is not None:
        track_id = int(result.boxes.id[idx].cpu())

    # Extract mask
    mask = np.zeros((image.height, image.width), dtype=np.uint8)
    if result.masks is not None and idx < len(result.masks.data):
        mask_tensor = result.masks.data[idx]
        mask_np = mask_tensor.cpu().numpy()

        # Resize mask to image size if needed
        if mask_np.shape != (image.height, image.width):
            mask_np = cv2.resize(
                mask_np.astype(np.float32),
                (image.width, image.height),
                interpolation=cv2.INTER_LINEAR,
            )

        # Binarize mask
        mask = (mask_np > 0.5).astype(np.uint8) * 255

    return detection_type(
        bbox=bbox,
        track_id=track_id,
        class_id=class_id,
        confidence=confidence,
        name=name,
        ts=image.ts,
        image=image,
        mask=mask,
    )
