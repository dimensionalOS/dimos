# Copyright (c) Facebook, Inc. and its affiliates.
import logging
import os

from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.data.datasets.lvis import get_lvis_instances_meta
from .lvis_v1 import custom_load_lvis_json, get_lvis_22k_meta
def custom_register_imagenet_instances(name, metadata, json_file, image_root):
    """
    """
    DatasetCatalog.register(name, lambda: custom_load_lvis_json(
        json_file, image_root, name))
    MetadataCatalog.get(name).set(
        json_file=json_file, image_root=image_root, 
        evaluator_type="imagenet", **metadata
    )

_CUSTOM_SPLITS_IMAGENET = {
    "imagenet_lvis_v1": ("imagenet/ImageNet-LVIS/", "imagenet/annotations/imagenet_lvis_image_info.json"),
}

for key, (image_root, json_file) in _CUSTOM_SPLITS_IMAGENET.items():
    custom_register_imagenet_instances(
        key,
        get_lvis_instances_meta('lvis_v1'),
        os.path.join("datasets", json_file) if "://" not in json_file else json_file,
        os.path.join("datasets", image_root),
    )


_CUSTOM_SPLITS_IMAGENET_22K = {
    "imagenet_lvis-22k": ("imagenet/ImageNet-LVIS/", "imagenet/annotations/imagenet-22k_image_info_lvis-22k.json"),
}

for key, (image_root, json_file) in _CUSTOM_SPLITS_IMAGENET_22K.items():
    custom_register_imagenet_instances(
        key,
        get_lvis_22k_meta(),
        os.path.join("datasets", json_file) if "://" not in json_file else json_file,
        os.path.join("datasets", image_root),
    )