# Copyright (c) Facebook, Inc. and its affiliates.
import logging
import os

from detectron2.data.datasets.builtin_meta import _get_builtin_metadata
from detectron2.data.datasets.lvis import get_lvis_instances_meta
from .lvis_v1 import custom_register_lvis_instances

_CUSTOM_SPLITS = {
    "cc3m_v1_val": ("cc3m/validation/", "cc3m/val_image_info.json"),
    "cc3m_v1_train": ("cc3m/training/", "cc3m/train_image_info.json"),
    "cc3m_v1_train_tags": ("cc3m/training/", "cc3m/train_image_info_tags.json"),

}

for key, (image_root, json_file) in _CUSTOM_SPLITS.items():
    custom_register_lvis_instances(
        key,
        get_lvis_instances_meta('lvis_v1'),
        os.path.join("datasets", json_file) if "://" not in json_file else json_file,
        os.path.join("datasets", image_root),
    )

