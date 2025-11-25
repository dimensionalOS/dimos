# Copyright 2025 Dimensional Inc.
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

from collections.abc import Sequence
import os
import sys

import numpy as np

# Add Detic to Python path
from dimos.constants import DIMOS_PROJECT_ROOT
from dimos.msgs.sensor_msgs import Image
from dimos.perception.detection.detectors.types import Detector
from dimos.perception.detection2d.utils import plot_results

detic_path = DIMOS_PROJECT_ROOT / "dimos/models/Detic"
if str(detic_path) not in sys.path:
    sys.path.append(str(detic_path))
    sys.path.append(str(detic_path / "third_party/CenterNet2"))

# PIL patch for compatibility
import PIL.Image

if not hasattr(PIL.Image, "LINEAR") and hasattr(PIL.Image, "BILINEAR"):
    PIL.Image.LINEAR = PIL.Image.BILINEAR  # type: ignore[attr-defined]

# Detectron2 imports
from detectron2.config import get_cfg  # type: ignore[import-not-found]
from detectron2.data import MetadataCatalog  # type: ignore[import-not-found]


# Simple tracking implementation
class SimpleTracker:
    """Simple IOU-based tracker implementation without external dependencies"""

    def __init__(self, iou_threshold: float = 0.3, max_age: int = 5) -> None:
        self.iou_threshold = iou_threshold
        self.max_age = max_age
        self.next_id = 1
        self.tracks = {}  # type: ignore[var-annotated]  # id -> {bbox, class_id, age, mask, etc}

    def _calculate_iou(self, bbox1, bbox2):  # type: ignore[no-untyped-def]
        """Calculate IoU between two bboxes in format [x1,y1,x2,y2]"""
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])

        if x2 < x1 or y2 < y1:
            return 0.0

        intersection = (x2 - x1) * (y2 - y1)
        area1 = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        area2 = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        union = area1 + area2 - intersection

        return intersection / union if union > 0 else 0

    def update(self, detections, masks):  # type: ignore[no-untyped-def]
        """Update tracker with new detections

        Args:
            detections: List of [x1,y1,x2,y2,score,class_id]
            masks: List of segmentation masks corresponding to detections

        Returns:
            List of [track_id, bbox, score, class_id, mask]
        """
        if len(detections) == 0:
            # Age existing tracks
            for track_id in list(self.tracks.keys()):
                self.tracks[track_id]["age"] += 1
                # Remove old tracks
                if self.tracks[track_id]["age"] > self.max_age:
                    del self.tracks[track_id]
            return []

        # Convert to numpy for easier handling
        if not isinstance(detections, np.ndarray):
            detections = np.array(detections)

        result = []
        matched_indices = set()

        # Update existing tracks
        for track_id, track in list(self.tracks.items()):
            track["age"] += 1

            if track["age"] > self.max_age:
                del self.tracks[track_id]
                continue

            # Find best matching detection for this track
            best_iou = self.iou_threshold
            best_idx = -1

            for i, det in enumerate(detections):
                if i in matched_indices:
                    continue

                # Check class match
                if det[5] != track["class_id"]:
                    continue

                iou = self._calculate_iou(track["bbox"], det[:4])  # type: ignore[no-untyped-call]
                if iou > best_iou:
                    best_iou = iou
                    best_idx = i

            # If we found a match, update the track
            if best_idx >= 0:
                self.tracks[track_id]["bbox"] = detections[best_idx][:4]
                self.tracks[track_id]["score"] = detections[best_idx][4]
                self.tracks[track_id]["age"] = 0
                self.tracks[track_id]["mask"] = masks[best_idx]
                matched_indices.add(best_idx)

                # Add to results with mask
                result.append(
                    [
                        track_id,
                        detections[best_idx][:4],
                        detections[best_idx][4],
                        int(detections[best_idx][5]),
                        self.tracks[track_id]["mask"],
                    ]
                )

        # Create new tracks for unmatched detections
        for i, det in enumerate(detections):
            if i in matched_indices:
                continue

            # Create new track
            new_id = self.next_id
            self.next_id += 1

            self.tracks[new_id] = {
                "bbox": det[:4],
                "score": det[4],
                "class_id": int(det[5]),
                "age": 0,
                "mask": masks[i],
            }

            # Add to results with mask directly from the track
            result.append([new_id, det[:4], det[4], int(det[5]), masks[i]])

        return result


class Detic2DDetector(Detector):
    def __init__(  # type: ignore[no-untyped-def]
        self, model_path=None, device: str = "cuda", vocabulary=None, threshold: float = 0.5
    ) -> None:
        """
        Initialize the Detic detector with open vocabulary support.

        Args:
            model_path (str): Path to a custom Detic model weights (optional)
            device (str): Device to run inference on ('cuda' or 'cpu')
            vocabulary (list): Custom vocabulary (list of class names) or 'lvis', 'objects365', 'openimages', 'coco'
            threshold (float): Detection confidence threshold
        """
        self.device = device
        self.threshold = threshold

        # Set up Detic paths - already added to sys.path at module level

        # Import Detic modules
        from centernet.config import add_centernet_config  # type: ignore[import-not-found]
        from detic.config import add_detic_config  # type: ignore[import-not-found]
        from detic.modeling.text.text_encoder import (  # type: ignore[import-not-found]
            build_text_encoder,
        )
        from detic.modeling.utils import reset_cls_test  # type: ignore[import-not-found]

        # Keep reference to these functions for later use
        self.reset_cls_test = reset_cls_test
        self.build_text_encoder = build_text_encoder

        # Setup model configuration
        self.cfg = get_cfg()
        add_centernet_config(self.cfg)
        add_detic_config(self.cfg)

        # Use default Detic config
        self.cfg.merge_from_file(
            os.path.join(
                detic_path, "configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml"
            )
        )

        # Set default weights if not provided
        if model_path is None:
            self.cfg.MODEL.WEIGHTS = "https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth"
        else:
            self.cfg.MODEL.WEIGHTS = model_path

        # Set device
        if device == "cpu":
            self.cfg.MODEL.DEVICE = "cpu"

        # Set detection threshold
        self.cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = threshold
        self.cfg.MODEL.ROI_BOX_HEAD.ZEROSHOT_WEIGHT_PATH = "rand"
        self.cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL = True

        # Built-in datasets for Detic - use absolute paths with detic_path
        self.builtin_datasets = {
            "lvis": {
                "metadata": "lvis_v1_val",
                "classifier": os.path.join(
                    detic_path, "datasets/metadata/lvis_v1_clip_a+cname.npy"
                ),
            },
            "objects365": {
                "metadata": "objects365_v2_val",
                "classifier": os.path.join(
                    detic_path, "datasets/metadata/o365_clip_a+cnamefix.npy"
                ),
            },
            "openimages": {
                "metadata": "oid_val_expanded",
                "classifier": os.path.join(detic_path, "datasets/metadata/oid_clip_a+cname.npy"),
            },
            "coco": {
                "metadata": "coco_2017_val",
                "classifier": os.path.join(detic_path, "datasets/metadata/coco_clip_a+cname.npy"),
            },
        }

        # Override config paths to use absolute paths
        self.cfg.MODEL.ROI_BOX_HEAD.CAT_FREQ_PATH = os.path.join(
            detic_path, "datasets/metadata/lvis_v1_train_cat_info.json"
        )

        # Initialize model
        self.predictor = None

        # Setup with initial vocabulary
        vocabulary = vocabulary or "lvis"
        self.setup_vocabulary(vocabulary)  # type: ignore[no-untyped-call]

        # Initialize our simple tracker
        self.tracker = SimpleTracker(iou_threshold=0.5, max_age=5)

    def setup_vocabulary(self, vocabulary):  # type: ignore[no-untyped-def]
        """
        Setup the model's vocabulary.

        Args:
            vocabulary: Either a string ('lvis', 'objects365', 'openimages', 'coco')
                       or a list of class names for custom vocabulary.
        """
        if self.predictor is None:
            # Initialize the model
            from detectron2.engine import DefaultPredictor  # type: ignore[import-not-found]

            self.predictor = DefaultPredictor(self.cfg)

        if isinstance(vocabulary, str) and vocabulary in self.builtin_datasets:
            # Use built-in dataset
            dataset = vocabulary
            metadata = MetadataCatalog.get(self.builtin_datasets[dataset]["metadata"])
            classifier = self.builtin_datasets[dataset]["classifier"]
            num_classes = len(metadata.thing_classes)
            self.class_names = metadata.thing_classes
        else:
            # Use custom vocabulary
            if isinstance(vocabulary, str):
                # If it's a string but not a built-in dataset, treat as a file
                try:
                    with open(vocabulary) as f:
                        class_names = [line.strip() for line in f if line.strip()]
                except:
                    # Default to LVIS if there's an issue
                    print(f"Error loading vocabulary from {vocabulary}, using LVIS")
                    return self.setup_vocabulary("lvis")  # type: ignore[no-untyped-call]
            else:
                # Assume it's a list of class names
                class_names = vocabulary

            # Create classifier from text embeddings
            metadata = MetadataCatalog.get("__unused")
            metadata.thing_classes = class_names
            self.class_names = class_names

            # Generate CLIP embeddings for custom vocabulary
            classifier = self._get_clip_embeddings(class_names)
            num_classes = len(class_names)

        # Reset model with new vocabulary
        self.reset_cls_test(self.predictor.model, classifier, num_classes)  # type: ignore[attr-defined]
        return self.class_names

    def _get_clip_embeddings(self, vocabulary, prompt: str = "a "):  # type: ignore[no-untyped-def]
        """
        Generate CLIP embeddings for a vocabulary list.

        Args:
            vocabulary (list): List of class names
            prompt (str): Prompt prefix to use for CLIP

        Returns:
            torch.Tensor: Tensor of embeddings
        """
        text_encoder = self.build_text_encoder(pretrain=True)
        text_encoder.eval()
        texts = [prompt + x for x in vocabulary]
        emb = text_encoder(texts).detach().permute(1, 0).contiguous().cpu()
        return emb

    def process_image(self, image: Image):  # type: ignore[no-untyped-def]
        """
        Process an image and return detection results.

        Args:
            image: Input image in BGR format (OpenCV)

        Returns:
            tuple: (bboxes, track_ids, class_ids, confidences, names, masks)
                - bboxes: list of [x1, y1, x2, y2] coordinates
                - track_ids: list of tracking IDs (or -1 if no tracking)
                - class_ids: list of class indices
                - confidences: list of detection confidences
                - names: list of class names
                - masks: list of segmentation masks (numpy arrays)
        """
        # Run inference with Detic
        outputs = self.predictor(image.to_opencv())  # type: ignore[misc]
        instances = outputs["instances"].to("cpu")

        # Extract bounding boxes, classes, scores, and masks
        if len(instances) == 0:
            return [], [], [], [], []  # , []

        boxes = instances.pred_boxes.tensor.numpy()
        class_ids = instances.pred_classes.numpy()
        scores = instances.scores.numpy()
        masks = instances.pred_masks.numpy()

        # Convert boxes to [x1, y1, x2, y2] format
        bboxes = []
        for box in boxes:
            x1, y1, x2, y2 = box.tolist()
            bboxes.append([x1, y1, x2, y2])

        # Get class names
        [self.class_names[class_id] for class_id in class_ids]

        # Apply tracking
        detections = []
        filtered_masks = []
        for i, bbox in enumerate(bboxes):
            if scores[i] >= self.threshold:
                # Format for tracker: [x1, y1, x2, y2, score, class_id]
                detections.append([*bbox, scores[i], class_ids[i]])
                filtered_masks.append(masks[i])

        if not detections:
            return [], [], [], [], []  # , []

        # Update tracker with detections and correctly aligned masks
        track_results = self.tracker.update(detections, filtered_masks)  # type: ignore[no-untyped-call]

        # Process tracking results
        track_ids = []
        tracked_bboxes = []
        tracked_class_ids = []
        tracked_scores = []
        tracked_names = []
        tracked_masks = []

        for track_id, bbox, score, class_id, mask in track_results:
            track_ids.append(int(track_id))
            tracked_bboxes.append(bbox.tolist() if isinstance(bbox, np.ndarray) else bbox)
            tracked_class_ids.append(int(class_id))
            tracked_scores.append(score)
            tracked_names.append(self.class_names[int(class_id)])
            tracked_masks.append(mask)

        return (
            tracked_bboxes,
            track_ids,
            tracked_class_ids,
            tracked_scores,
            tracked_names,
            # tracked_masks,
        )

    def visualize_results(  # type: ignore[no-untyped-def]
        self, image, bboxes, track_ids, class_ids, confidences, names: Sequence[str]
    ):
        """
        Generate visualization of detection results.

        Args:
            image: Original input image
            bboxes: List of bounding boxes
            track_ids: List of tracking IDs
            class_ids: List of class indices
            confidences: List of detection confidences
            names: List of class names

        Returns:
            Image with visualized detections
        """

        return plot_results(image, bboxes, track_ids, class_ids, confidences, names)

    def cleanup(self) -> None:
        """Clean up resources."""
        # Nothing specific to clean up for Detic
        pass
