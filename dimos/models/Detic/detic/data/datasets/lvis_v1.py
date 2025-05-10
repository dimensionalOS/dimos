# Copyright (c) Facebook, Inc. and its affiliates.
import logging
import os

from fvcore.common.timer import Timer
from detectron2.structures import BoxMode
from fvcore.common.file_io import PathManager
from detectron2.data import DatasetCatalog, MetadataCatalog
from detectron2.data.datasets.lvis import get_lvis_instances_meta

logger = logging.getLogger(__name__)

__all__ = ["custom_load_lvis_json", "custom_register_lvis_instances"]


def custom_register_lvis_instances(name, metadata, json_file, image_root):
    """
    """
    DatasetCatalog.register(name, lambda: custom_load_lvis_json(
        json_file, image_root, name))
    MetadataCatalog.get(name).set(
        json_file=json_file, image_root=image_root, 
        evaluator_type="lvis", **metadata
    )


def custom_load_lvis_json(json_file, image_root, dataset_name=None):
    '''
    Modifications:
      use `file_name`
      convert neg_category_ids
      add pos_category_ids
    '''
    from lvis import LVIS

    json_file = PathManager.get_local_path(json_file)

    timer = Timer()
    lvis_api = LVIS(json_file)
    if timer.seconds() > 1:
        logger.info("Loading {} takes {:.2f} seconds.".format(
            json_file, timer.seconds()))

    catid2contid = {x['id']: i for i, x in enumerate(
        sorted(lvis_api.dataset['categories'], key=lambda x: x['id']))}
    if len(lvis_api.dataset['categories']) == 1203:
        for x in lvis_api.dataset['categories']:
            assert catid2contid[x['id']] == x['id'] - 1
    img_ids = sorted(lvis_api.imgs.keys())
    imgs = lvis_api.load_imgs(img_ids)
    anns = [lvis_api.img_ann_map[img_id] for img_id in img_ids]

    ann_ids = [ann["id"] for anns_per_image in anns for ann in anns_per_image]
    assert len(set(ann_ids)) == len(ann_ids), \
        "Annotation ids in '{}' are not unique".format(json_file)

    imgs_anns = list(zip(imgs, anns))
    logger.info("Loaded {} images in the LVIS v1 format from {}".format(
        len(imgs_anns), json_file))

    dataset_dicts = []

    for (img_dict, anno_dict_list) in imgs_anns:
        record = {}
        if "file_name" in img_dict:
            file_name = img_dict["file_name"]
            if img_dict["file_name"].startswith("COCO"):
                file_name = file_name[-16:]
            record["file_name"] = os.path.join(image_root, file_name)
        elif 'coco_url' in img_dict:
            # e.g., http://images.cocodataset.org/train2017/000000391895.jpg
            file_name = img_dict["coco_url"][30:]
            record["file_name"] = os.path.join(image_root, file_name)
        elif 'tar_index' in img_dict:
            record['tar_index'] = img_dict['tar_index']
        
        record["height"] = img_dict["height"]
        record["width"] = img_dict["width"]
        record["not_exhaustive_category_ids"] = img_dict.get(
            "not_exhaustive_category_ids", [])
        record["neg_category_ids"] = img_dict.get("neg_category_ids", [])
        # NOTE: modified by Xingyi: convert to 0-based
        record["neg_category_ids"] = [
            catid2contid[x] for x in record["neg_category_ids"]]
        if 'pos_category_ids' in img_dict:
            record['pos_category_ids'] = [
                catid2contid[x] for x in img_dict.get("pos_category_ids", [])]
        if 'captions' in img_dict:
            record['captions'] = img_dict['captions']
        if 'caption_features' in img_dict:
            record['caption_features'] = img_dict['caption_features']
        image_id = record["image_id"] = img_dict["id"]

        objs = []
        for anno in anno_dict_list:
            assert anno["image_id"] == image_id
            if anno.get('iscrowd', 0) > 0:
                continue
            obj = {"bbox": anno["bbox"], "bbox_mode": BoxMode.XYWH_ABS}
            obj["category_id"] = catid2contid[anno['category_id']] 
            if 'segmentation' in anno:
                segm = anno["segmentation"]
                valid_segm = [poly for poly in segm \
                    if len(poly) % 2 == 0 and len(poly) >= 6]
                # assert len(segm) == len(
                #     valid_segm
                # ), "Annotation contains an invalid polygon with < 3 points"
                if not len(segm) == len(valid_segm):
                    print('Annotation contains an invalid polygon with < 3 points')
                assert len(segm) > 0
                obj["segmentation"] = segm
            objs.append(obj)
        record["annotations"] = objs
        dataset_dicts.append(record)

    return dataset_dicts

_CUSTOM_SPLITS_LVIS = {
    "lvis_v1_train+coco": ("coco/", "lvis/lvis_v1_train+coco_mask.json"),
    "lvis_v1_train_norare": ("coco/", "lvis/lvis_v1_train_norare.json"),
}


for key, (image_root, json_file) in _CUSTOM_SPLITS_LVIS.items():
    custom_register_lvis_instances(
        key,
        get_lvis_instances_meta(key),
        os.path.join("datasets", json_file) if "://" not in json_file else json_file,
        os.path.join("datasets", image_root),
    )


def get_lvis_22k_meta():
    from .lvis_22k_categories import CATEGORIES
    cat_ids = [k["id"] for k in CATEGORIES]
    assert min(cat_ids) == 1 and max(cat_ids) == len(
        cat_ids
    ), "Category ids are not in [1, #categories], as expected"
    # Ensure that the category list is sorted by id
    lvis_categories = sorted(CATEGORIES, key=lambda x: x["id"])
    thing_classes = [k["name"] for k in lvis_categories]
    meta = {"thing_classes": thing_classes}
    return meta

_CUSTOM_SPLITS_LVIS_22K = {
    "lvis_v1_train_22k": ("coco/", "lvis/lvis_v1_train_lvis-22k.json"),
}

for key, (image_root, json_file) in _CUSTOM_SPLITS_LVIS_22K.items():
    custom_register_lvis_instances(
        key,
        get_lvis_22k_meta(),
        os.path.join("datasets", json_file) if "://" not in json_file else json_file,
        os.path.join("datasets", image_root),
    )