from __future__ import annotations
import logging
from pathlib import Path

import numpy as np

from demo_app.types import Detection

logger = logging.getLogger(__name__)


class YoloToolDetector:
    INFER_SIZE = 1280

    def __init__(self, prompts: list[str], conf_threshold: float, model_name: str) -> None:
        from ultralytics import YOLO
        model_path = Path(model_name)
        if not model_path.is_absolute():
            candidate = Path.cwd() / model_name
            if candidate.exists():
                model_path = candidate
        logger.info(
            "YOLO init step 1/4: model_path=%s exists=%s",
            model_path,
            model_path.exists(),
        )
        self._model = YOLO(str(model_path))
        logger.info("YOLO init step 2/4: closed-set YOLO model object created")
        self._prompts = list(prompts)
        logger.warning(
            "Using closed-set YOLO for this demo. It will detect only built-in classes "
            "and then filter them against prompts=%s",
            self._prompts,
        )
        self._conf = conf_threshold

    def detect(self, frame: np.ndarray, timestamp: float) -> list[Detection]:
        results = self._model.predict(frame, conf=self._conf, imgsz=self.INFER_SIZE, verbose=False)
        if not results:
            return []
        result = results[0]
        if result.boxes is None:
            return []

        detections: list[Detection] = []
        for box in result.boxes:
            xyxy = box.xyxy[0].cpu().numpy().astype(int)
            cls_idx = int(box.cls[0].item())
            conf = float(box.conf[0].item())
            class_name = self._class_name_from_result(result, cls_idx)
            matched_prompt = self._match_prompt(class_name)
            if matched_prompt is None:
                continue
            class_name = matched_prompt
            detections.append(
                Detection(
                    bbox=(int(xyxy[0]), int(xyxy[1]), int(xyxy[2]), int(xyxy[3])),
                    class_name=class_name,
                    confidence=conf,
                    frame=frame,
                    timestamp=timestamp,
                )
            )
        return detections

    def _class_name_from_result(self, result, cls_idx: int) -> str:
        names = getattr(result, "names", None)
        if isinstance(names, dict):
            return str(names.get(cls_idx, "unknown"))
        if isinstance(names, list) and 0 <= cls_idx < len(names):
            return str(names[cls_idx])
        model_names = getattr(self._model, "names", None)
        if isinstance(model_names, dict):
            return str(model_names.get(cls_idx, "unknown"))
        if isinstance(model_names, list) and 0 <= cls_idx < len(model_names):
            return str(model_names[cls_idx])
        return "unknown"

    def _match_prompt(self, detected_name: str) -> str | None:
        detected_norm = self._normalize(detected_name)
        for prompt in self._prompts:
            prompt_norm = self._normalize(prompt)
            if prompt_norm == detected_norm:
                return prompt
            if prompt_norm in detected_norm or detected_norm in prompt_norm:
                return prompt

        aliases = {
            "person": {"human", "man", "woman", "boy", "girl", "pedestrian"},
            "chair": {"seat", "stool", "armchair"},
            "office chair": {"desk chair", "rolling chair", "swivel chair"},
            "couch": {"sofa"},
            "bench": {"park bench"},
            "suitcase": {"luggage", "case"},
            "backpack": {"bag", "rucksack"},
        }
        for prompt in self._prompts:
            alias_set = aliases.get(prompt, set())
            if detected_norm in alias_set:
                return prompt
        return None

    @staticmethod
    def _normalize(value: str) -> str:
        return " ".join(value.lower().replace("_", " ").replace("-", " ").split())
