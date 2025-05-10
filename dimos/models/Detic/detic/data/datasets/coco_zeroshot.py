# Copyright (c) Facebook, Inc. and its affiliates.
import os

from detectron2.data.datasets.register_coco import register_coco_instances
from detectron2.data.datasets.builtin_meta import _get_builtin_metadata
from .lvis_v1 import custom_register_lvis_instances

categories_seen = [
    {'id': 1, 'name': 'person'},
    {'id': 2, 'name': 'bicycle'},
    {'id': 3, 'name': 'car'},
    {'id': 4, 'name': 'motorcycle'},
    {'id': 7, 'name': 'train'},
    {'id': 8, 'name': 'truck'},
    {'id': 9, 'name': 'boat'},
    {'id': 15, 'name': 'bench'},
    {'id': 16, 'name': 'bird'},
    {'id': 19, 'name': 'horse'},
    {'id': 20, 'name': 'sheep'},
    {'id': 23, 'name': 'bear'},
    {'id': 24, 'name': 'zebra'},
    {'id': 25, 'name': 'giraffe'},
    {'id': 27, 'name': 'backpack'},
    {'id': 31, 'name': 'handbag'},
    {'id': 33, 'name': 'suitcase'},
    {'id': 34, 'name': 'frisbee'},
    {'id': 35, 'name': 'skis'},
    {'id': 38, 'name': 'kite'},
    {'id': 42, 'name': 'surfboard'},
    {'id': 44, 'name': 'bottle'},
    {'id': 48, 'name': 'fork'},
    {'id': 50, 'name': 'spoon'},
    {'id': 51, 'name': 'bowl'},
    {'id': 52, 'name': 'banana'},
    {'id': 53, 'name': 'apple'},
    {'id': 54, 'name': 'sandwich'},
    {'id': 55, 'name': 'orange'},
    {'id': 56, 'name': 'broccoli'},
    {'id': 57, 'name': 'carrot'},
    {'id': 59, 'name': 'pizza'},
    {'id': 60, 'name': 'donut'},
    {'id': 62, 'name': 'chair'},
    {'id': 65, 'name': 'bed'},
    {'id': 70, 'name': 'toilet'},
    {'id': 72, 'name': 'tv'},
    {'id': 73, 'name': 'laptop'},
    {'id': 74, 'name': 'mouse'},
    {'id': 75, 'name': 'remote'},
    {'id': 78, 'name': 'microwave'},
    {'id': 79, 'name': 'oven'},
    {'id': 80, 'name': 'toaster'},
    {'id': 82, 'name': 'refrigerator'},
    {'id': 84, 'name': 'book'},
    {'id': 85, 'name': 'clock'},
    {'id': 86, 'name': 'vase'},
    {'id': 90, 'name': 'toothbrush'},
]

categories_unseen = [
    {'id': 5, 'name': 'airplane'},
    {'id': 6, 'name': 'bus'},
    {'id': 17, 'name': 'cat'},
    {'id': 18, 'name': 'dog'},
    {'id': 21, 'name': 'cow'},
    {'id': 22, 'name': 'elephant'},
    {'id': 28, 'name': 'umbrella'},
    {'id': 32, 'name': 'tie'},
    {'id': 36, 'name': 'snowboard'},
    {'id': 41, 'name': 'skateboard'},
    {'id': 47, 'name': 'cup'},
    {'id': 49, 'name': 'knife'},
    {'id': 61, 'name': 'cake'},
    {'id': 63, 'name': 'couch'},
    {'id': 76, 'name': 'keyboard'},
    {'id': 81, 'name': 'sink'},
    {'id': 87, 'name': 'scissors'},
]

def _get_metadata(cat):
    if cat == 'all':
        return _get_builtin_metadata('coco')
    elif cat == 'seen':
        id_to_name = {x['id']: x['name'] for x in categories_seen}
    else:
        assert cat == 'unseen'
        id_to_name = {x['id']: x['name'] for x in categories_unseen}

    thing_dataset_id_to_contiguous_id = {
        x: i for i, x in enumerate(sorted(id_to_name))}
    thing_classes = [id_to_name[k] for k in sorted(id_to_name)]
    return {
        "thing_dataset_id_to_contiguous_id": thing_dataset_id_to_contiguous_id,
        "thing_classes": thing_classes}

_PREDEFINED_SPLITS_COCO = {
    "coco_zeroshot_train": ("coco/train2017", "coco/zero-shot/instances_train2017_seen_2.json", 'seen'),
    "coco_zeroshot_val": ("coco/val2017", "coco/zero-shot/instances_val2017_unseen_2.json", 'unseen'),
    "coco_not_zeroshot_val": ("coco/val2017", "coco/zero-shot/instances_val2017_seen_2.json", 'seen'),
    "coco_generalized_zeroshot_val": ("coco/val2017", "coco/zero-shot/instances_val2017_all_2_oriorder.json", 'all'),
    "coco_zeroshot_train_oriorder": ("coco/train2017", "coco/zero-shot/instances_train2017_seen_2_oriorder.json", 'all'),
}

for key, (image_root, json_file, cat) in _PREDEFINED_SPLITS_COCO.items():
    register_coco_instances(
        key,
        _get_metadata(cat),
        os.path.join("datasets", json_file) if "://" not in json_file else json_file,
        os.path.join("datasets", image_root),
    )

_CUSTOM_SPLITS_COCO = {
    "cc3m_coco_train_tags": ("cc3m/training/", "cc3m/coco_train_image_info_tags.json"),
    "coco_caption_train_tags": ("coco/train2017/", "coco/annotations/captions_train2017_tags_allcaps.json"),}

for key, (image_root, json_file) in _CUSTOM_SPLITS_COCO.items():
    custom_register_lvis_instances(
        key,
        _get_builtin_metadata('coco'),
        os.path.join("datasets", json_file) if "://" not in json_file else json_file,
        os.path.join("datasets", image_root),
    )