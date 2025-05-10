# Copyright (c) Facebook, Inc. and its affiliates.
from collections import defaultdict
import torch
import sys
import json
import numpy as np

from detectron2.structures import Boxes, pairwise_iou
COCO_PATH = 'datasets/coco/annotations/instances_train2017.json'
IMG_PATH = 'datasets/coco/train2017/'
LVIS_PATH = 'datasets/lvis/lvis_v1_train.json'
NO_SEG = False
if NO_SEG:
    SAVE_PATH = 'datasets/lvis/lvis_v1_train+coco_box.json'
else:
    SAVE_PATH = 'datasets/lvis/lvis_v1_train+coco_mask.json'
THRESH = 0.7
DEBUG = False

# This mapping is extracted from the official LVIS mapping:
# https://github.com/lvis-dataset/lvis-api/blob/master/data/coco_to_synset.json
COCO_SYNSET_CATEGORIES = [
    {"synset": "person.n.01", "coco_cat_id": 1},
    {"synset": "bicycle.n.01", "coco_cat_id": 2},
    {"synset": "car.n.01", "coco_cat_id": 3},
    {"synset": "motorcycle.n.01", "coco_cat_id": 4},
    {"synset": "airplane.n.01", "coco_cat_id": 5},
    {"synset": "bus.n.01", "coco_cat_id": 6},
    {"synset": "train.n.01", "coco_cat_id": 7},
    {"synset": "truck.n.01", "coco_cat_id": 8},
    {"synset": "boat.n.01", "coco_cat_id": 9},
    {"synset": "traffic_light.n.01", "coco_cat_id": 10},
    {"synset": "fireplug.n.01", "coco_cat_id": 11},
    {"synset": "stop_sign.n.01", "coco_cat_id": 13},
    {"synset": "parking_meter.n.01", "coco_cat_id": 14},
    {"synset": "bench.n.01", "coco_cat_id": 15},
    {"synset": "bird.n.01", "coco_cat_id": 16},
    {"synset": "cat.n.01", "coco_cat_id": 17},
    {"synset": "dog.n.01", "coco_cat_id": 18},
    {"synset": "horse.n.01", "coco_cat_id": 19},
    {"synset": "sheep.n.01", "coco_cat_id": 20},
    {"synset": "beef.n.01", "coco_cat_id": 21},
    {"synset": "elephant.n.01", "coco_cat_id": 22},
    {"synset": "bear.n.01", "coco_cat_id": 23},
    {"synset": "zebra.n.01", "coco_cat_id": 24},
    {"synset": "giraffe.n.01", "coco_cat_id": 25},
    {"synset": "backpack.n.01", "coco_cat_id": 27},
    {"synset": "umbrella.n.01", "coco_cat_id": 28},
    {"synset": "bag.n.04", "coco_cat_id": 31},
    {"synset": "necktie.n.01", "coco_cat_id": 32},
    {"synset": "bag.n.06", "coco_cat_id": 33},
    {"synset": "frisbee.n.01", "coco_cat_id": 34},
    {"synset": "ski.n.01", "coco_cat_id": 35},
    {"synset": "snowboard.n.01", "coco_cat_id": 36},
    {"synset": "ball.n.06", "coco_cat_id": 37},
    {"synset": "kite.n.03", "coco_cat_id": 38},
    {"synset": "baseball_bat.n.01", "coco_cat_id": 39},
    {"synset": "baseball_glove.n.01", "coco_cat_id": 40},
    {"synset": "skateboard.n.01", "coco_cat_id": 41},
    {"synset": "surfboard.n.01", "coco_cat_id": 42},
    {"synset": "tennis_racket.n.01", "coco_cat_id": 43},
    {"synset": "bottle.n.01", "coco_cat_id": 44},
    {"synset": "wineglass.n.01", "coco_cat_id": 46},
    {"synset": "cup.n.01", "coco_cat_id": 47},
    {"synset": "fork.n.01", "coco_cat_id": 48},
    {"synset": "knife.n.01", "coco_cat_id": 49},
    {"synset": "spoon.n.01", "coco_cat_id": 50},
    {"synset": "bowl.n.03", "coco_cat_id": 51},
    {"synset": "banana.n.02", "coco_cat_id": 52},
    {"synset": "apple.n.01", "coco_cat_id": 53},
    {"synset": "sandwich.n.01", "coco_cat_id": 54},
    {"synset": "orange.n.01", "coco_cat_id": 55},
    {"synset": "broccoli.n.01", "coco_cat_id": 56},
    {"synset": "carrot.n.01", "coco_cat_id": 57},
    # {"synset": "frank.n.02", "coco_cat_id": 58},
    {"synset": "sausage.n.01", "coco_cat_id": 58},
    {"synset": "pizza.n.01", "coco_cat_id": 59},
    {"synset": "doughnut.n.02", "coco_cat_id": 60},
    {"synset": "cake.n.03", "coco_cat_id": 61},
    {"synset": "chair.n.01", "coco_cat_id": 62},
    {"synset": "sofa.n.01", "coco_cat_id": 63},
    {"synset": "pot.n.04", "coco_cat_id": 64},
    {"synset": "bed.n.01", "coco_cat_id": 65},
    {"synset": "dining_table.n.01", "coco_cat_id": 67},
    {"synset": "toilet.n.02", "coco_cat_id": 70},
    {"synset": "television_receiver.n.01", "coco_cat_id": 72},
    {"synset": "laptop.n.01", "coco_cat_id": 73},
    {"synset": "mouse.n.04", "coco_cat_id": 74},
    {"synset": "remote_control.n.01", "coco_cat_id": 75},
    {"synset": "computer_keyboard.n.01", "coco_cat_id": 76},
    {"synset": "cellular_telephone.n.01", "coco_cat_id": 77},
    {"synset": "microwave.n.02", "coco_cat_id": 78},
    {"synset": "oven.n.01", "coco_cat_id": 79},
    {"synset": "toaster.n.02", "coco_cat_id": 80},
    {"synset": "sink.n.01", "coco_cat_id": 81},
    {"synset": "electric_refrigerator.n.01", "coco_cat_id": 82},
    {"synset": "book.n.01", "coco_cat_id": 84},
    {"synset": "clock.n.01", "coco_cat_id": 85},
    {"synset": "vase.n.01", "coco_cat_id": 86},
    {"synset": "scissors.n.01", "coco_cat_id": 87},
    {"synset": "teddy.n.01", "coco_cat_id": 88},
    {"synset": "hand_blower.n.01", "coco_cat_id": 89},
    {"synset": "toothbrush.n.01", "coco_cat_id": 90},
]


def get_bbox(ann):
    bbox = ann['bbox']
    return [bbox[0], bbox[1], bbox[0] + bbox[2], bbox[1] + bbox[3]]


if __name__ == '__main__':
    file_name_key = 'file_name' if 'v0.5' in LVIS_PATH else 'coco_url'
    coco_data = json.load(open(COCO_PATH, 'r'))
    lvis_data = json.load(open(LVIS_PATH, 'r'))

    coco_cats = coco_data['categories']
    lvis_cats = lvis_data['categories']

    num_find = 0
    num_not_find = 0
    num_twice = 0
    coco2lviscats = {}
    synset2lvisid = {x['synset']: x['id'] for x in lvis_cats}
    # cocoid2synset = {x['coco_cat_id']: x['synset'] for x in COCO_SYNSET_CATEGORIES}
    coco2lviscats = {x['coco_cat_id']: synset2lvisid[x['synset']] \
        for x in COCO_SYNSET_CATEGORIES if x['synset'] in synset2lvisid}
    print(len(coco2lviscats))
    
    lvis_file2id = {x[file_name_key][-16:]: x['id'] for x in lvis_data['images']}
    lvis_id2img = {x['id']: x for x in lvis_data['images']}
    lvis_catid2name = {x['id']: x['name'] for x in lvis_data['categories']}

    coco_file2anns = {}
    coco_id2img = {x['id']: x for x in coco_data['images']}
    coco_img2anns = defaultdict(list)
    for ann in coco_data['annotations']:
        coco_img = coco_id2img[ann['image_id']]
        file_name = coco_img['file_name'][-16:]
        if ann['category_id'] in coco2lviscats and \
            file_name in lvis_file2id:
            lvis_image_id = lvis_file2id[file_name]
            lvis_image = lvis_id2img[lvis_image_id]
            lvis_cat_id = coco2lviscats[ann['category_id']]
            if lvis_cat_id in lvis_image['neg_category_ids']:
                continue
                if DEBUG:
                    import cv2
                    img_path = IMG_PATH + file_name
                    img = cv2.imread(img_path)
                    print(lvis_catid2name[lvis_cat_id])
                    print('neg', [lvis_catid2name[x] for x in lvis_image['neg_category_ids']])
                    cv2.imshow('img', img)
                    cv2.waitKey()
            ann['category_id'] = lvis_cat_id
            ann['image_id'] = lvis_image_id
            coco_img2anns[file_name].append(ann)
    
    lvis_img2anns = defaultdict(list)
    for ann in lvis_data['annotations']:
        lvis_img = lvis_id2img[ann['image_id']]
        file_name = lvis_img[file_name_key][-16:]
        lvis_img2anns[file_name].append(ann)

    ann_id_count = 0
    anns = []
    for file_name in lvis_img2anns:
        coco_anns = coco_img2anns[file_name]
        lvis_anns = lvis_img2anns[file_name]
        ious = pairwise_iou(
            Boxes(torch.tensor([get_bbox(x) for x in coco_anns])), 
            Boxes(torch.tensor([get_bbox(x) for x in lvis_anns]))
        )

        for ann in lvis_anns:
            ann_id_count = ann_id_count + 1
            ann['id'] = ann_id_count
            anns.append(ann)

        for i, ann in enumerate(coco_anns):
            if len(ious[i]) == 0 or ious[i].max() < THRESH:
                ann_id_count = ann_id_count + 1
                ann['id'] = ann_id_count
                anns.append(ann)
            else:
                duplicated = False
                for j in range(len(ious[i])):
                    if ious[i, j] >= THRESH and \
                        coco_anns[i]['category_id'] == lvis_anns[j]['category_id']:
                        duplicated = True
                if not duplicated:
                    ann_id_count = ann_id_count + 1
                    ann['id'] = ann_id_count
                    anns.append(ann)
    if NO_SEG:
        for ann in anns:
            del ann['segmentation']
    lvis_data['annotations'] = anns
    
    print('# Images', len(lvis_data['images']))
    print('# Anns', len(lvis_data['annotations']))
    json.dump(lvis_data, open(SAVE_PATH, 'w'))
