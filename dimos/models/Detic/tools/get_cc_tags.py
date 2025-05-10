# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import json
from collections import defaultdict
from detectron2.data.datasets.lvis_v1_categories import LVIS_CATEGORIES

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

def map_name(x):
    x = x.replace('_', ' ')
    if '(' in x:
        x = x[:x.find('(')]
    return x.lower().strip()

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cc_ann', default='datasets/cc3m/train_image_info.json')
    parser.add_argument('--out_path', default='datasets/cc3m/train_image_info_tags.json')
    parser.add_argument('--keep_images', action='store_true')
    parser.add_argument('--allcaps', action='store_true')
    parser.add_argument('--cat_path', default='')
    parser.add_argument('--convert_caption', action='store_true')
    # parser.add_argument('--lvis_ann', default='datasets/lvis/lvis_v1_val.json')
    args = parser.parse_args()

    # lvis_data = json.load(open(args.lvis_ann, 'r'))
    cc_data = json.load(open(args.cc_ann, 'r'))
    if args.convert_caption:
        num_caps = 0
        caps = defaultdict(list)
        for x in cc_data['annotations']:
            caps[x['image_id']].append(x['caption'])
        for x in cc_data['images']:
            x['captions'] = caps[x['id']]
            num_caps += len(x['captions'])
        print('# captions', num_caps)

    if args.cat_path != '':
        print('Loading', args.cat_path)
        cats = json.load(open(args.cat_path))['categories']
        if 'synonyms' not in cats[0]:
            cocoid2synset = {x['coco_cat_id']: x['synset'] \
                for x in COCO_SYNSET_CATEGORIES}
            synset2synonyms = {x['synset']: x['synonyms'] \
                for x in LVIS_CATEGORIES}
            for x in cats:
                synonyms = synset2synonyms[cocoid2synset[x['id']]]
                x['synonyms'] = synonyms
                x['frequency'] = 'f'
        cc_data['categories'] = cats

    id2cat = {x['id']: x for x in cc_data['categories']}
    class_count = {x['id']: 0 for x in cc_data['categories']}
    class_data = {x['id']: [' ' + map_name(xx) + ' ' for xx in x['synonyms']] \
            for x in cc_data['categories']}
    num_examples = 5
    examples = {x['id']: [] for x in cc_data['categories']}

    print('class_data', class_data)

    images = []
    for i, x in enumerate(cc_data['images']):
        if i % 10000 == 0:
            print(i, len(cc_data['images']))
        if args.allcaps:
            caption = (' '.join(x['captions'])).lower()
        else:
            caption = x['captions'][0].lower()
        x['pos_category_ids'] = []
        for cat_id, cat_names in class_data.items():
            find = False
            for c in cat_names:
                if c in caption or caption.startswith(c[1:]) \
                    or caption.endswith(c[:-1]):
                    find = True
                    break
            if find:
                x['pos_category_ids'].append(cat_id)
                class_count[cat_id] += 1
                if len(examples[cat_id]) < num_examples:
                    examples[cat_id].append(caption)
        if len(x['pos_category_ids']) > 0 or args.keep_images:
            images.append(x)

    zero_class = []
    for cat_id, count in class_count.items():
        print(id2cat[cat_id]['name'], count, end=', ')
        if count == 0:
            zero_class.append(id2cat[cat_id])
    print('==')
    print('zero class', zero_class)

    # for freq in ['r', 'c', 'f']:
    #     print('#cats', freq, len([x for x in cc_data['categories'] \
    #         if x['frequency'] == freq] and class_count[x['id']] > 0))

    for freq in ['r', 'c', 'f']:
        print('#Images', freq, sum([v for k, v in class_count.items() \
        if id2cat[k]['frequency'] == freq]))

    try:
        out_data = {'images': images, 'categories': cc_data['categories'], \
            'annotations': []}
        for k, v in out_data.items():
            print(k, len(v))
        if args.keep_images and not args.out_path.endswith('_full.json'):
            args.out_path = args.out_path[:-5] + '_full.json'
        print('Writing to', args.out_path)
        json.dump(out_data, open(args.out_path, 'w'))
    except:
        pass
