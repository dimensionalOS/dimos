# Copyright (c) Facebook, Inc. and its affiliates.
# Modified by Xingyi Zhou from https://github.com/facebookresearch/detectron2/blob/master/detectron2/data/datasets/coco.py
import copy
import io
import logging
import contextlib
import os
import datetime
import json
import numpy as np

from PIL import Image

from fvcore.common.timer import Timer
from fvcore.common.file_io import PathManager, file_lock
from detectron2.structures import BoxMode, PolygonMasks, Boxes
from detectron2.data import DatasetCatalog, MetadataCatalog

logger = logging.getLogger(__name__)

"""
This file contains functions to register a COCO-format dataset to the DatasetCatalog.
"""

__all__ = ["register_coco_instances", "register_coco_panoptic_separated"]



def register_oid_instances(name, metadata, json_file, image_root):
    """
    """
    # 1. register a function which returns dicts
    DatasetCatalog.register(name, lambda: load_coco_json_mem_efficient(
        json_file, image_root, name))

    # 2. Optionally, add metadata about this dataset,
    # since they might be useful in evaluation, visualization or logging
    MetadataCatalog.get(name).set(
        json_file=json_file, image_root=image_root, evaluator_type="oid", **metadata
    )


def load_coco_json_mem_efficient(json_file, image_root, dataset_name=None, extra_annotation_keys=None):
    """
    Actually not mem efficient
    """
    from pycocotools.coco import COCO

    timer = Timer()
    json_file = PathManager.get_local_path(json_file)
    with contextlib.redirect_stdout(io.StringIO()):
        coco_api = COCO(json_file)
    if timer.seconds() > 1:
        logger.info("Loading {} takes {:.2f} seconds.".format(json_file, timer.seconds()))

    id_map = None
    if dataset_name is not None:
        meta = MetadataCatalog.get(dataset_name)
        cat_ids = sorted(coco_api.getCatIds())
        cats = coco_api.loadCats(cat_ids)
        # The categories in a custom json file may not be sorted.
        thing_classes = [c["name"] for c in sorted(cats, key=lambda x: x["id"])]
        meta.thing_classes = thing_classes

        if not (min(cat_ids) == 1 and max(cat_ids) == len(cat_ids)):
            if "coco" not in dataset_name:
                logger.warning(
                    """
                    Category ids in annotations are not in [1, #categories]! We'll apply a mapping for you.
                    """
                )
        id_map = {v: i for i, v in enumerate(cat_ids)}
        meta.thing_dataset_id_to_contiguous_id = id_map

    # sort indices for reproducible results
    img_ids = sorted(coco_api.imgs.keys())
    imgs = coco_api.loadImgs(img_ids)
    logger.info("Loaded {} images in COCO format from {}".format(len(imgs), json_file))

    dataset_dicts = []

    ann_keys = ["iscrowd", "bbox", "category_id"] + (extra_annotation_keys or [])

    for img_dict in imgs:
        record = {}
        record["file_name"] = os.path.join(image_root, img_dict["file_name"])
        record["height"] = img_dict["height"]
        record["width"] = img_dict["width"]
        image_id = record["image_id"] = img_dict["id"]
        anno_dict_list = coco_api.imgToAnns[image_id]
        if 'neg_category_ids' in img_dict:
            record['neg_category_ids'] = \
                [id_map[x] for x in img_dict['neg_category_ids']]

        objs = []
        for anno in anno_dict_list:
            assert anno["image_id"] == image_id

            assert anno.get("ignore", 0) == 0

            obj = {key: anno[key] for key in ann_keys if key in anno}

            segm = anno.get("segmentation", None)
            if segm:  # either list[list[float]] or dict(RLE)
                if not isinstance(segm, dict):
                    # filter out invalid polygons (< 3 points)
                    segm = [poly for poly in segm if len(poly) % 2 == 0 and len(poly) >= 6]
                    if len(segm) == 0:
                        num_instances_without_valid_segmentation += 1
                        continue  # ignore this instance
                obj["segmentation"] = segm

            obj["bbox_mode"] = BoxMode.XYWH_ABS

            if id_map:
                obj["category_id"] = id_map[obj["category_id"]]
            objs.append(obj)
        record["annotations"] = objs
        dataset_dicts.append(record)
    
    del coco_api
    return dataset_dicts