# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import json
import os
import cv2
from nltk.corpus import wordnet
from detectron2.data.detection_utils import read_image

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--imagenet_path', default='datasets/imagenet/ImageNet-LVIS')
    parser.add_argument('--lvis_meta_path', default='datasets/lvis/lvis_v1_val.json')
    parser.add_argument('--out_path', default='datasets/imagenet/annotations/imagenet_lvis_image_info.json')
    args = parser.parse_args()

    print('Loading LVIS meta')
    data = json.load(open(args.lvis_meta_path, 'r'))
    print('Done')
    synset2cat = {x['synset']: x for x in data['categories']}
    count = 0
    images = []
    image_counts = {}
    folders = sorted(os.listdir(args.imagenet_path))
    for i, folder in enumerate(folders):
        class_path = args.imagenet_path + folder
        files = sorted(os.listdir(class_path))
        synset = wordnet.synset_from_pos_and_offset('n', int(folder[1:])).name()
        cat = synset2cat[synset]
        cat_id = cat['id']
        cat_name = cat['name']
        cat_images = []
        for file in files:
            count = count + 1
            file_name = '{}/{}'.format(folder, file)
            # img = cv2.imread('{}/{}'.format(args.imagenet_path, file_name))
            img = read_image('{}/{}'.format(args.imagenet_path, file_name))
            h, w = img.shape[:2]
            image = {
                'id': count,
                'file_name': file_name,
                'pos_category_ids': [cat_id],
                'width': w,
                'height': h
            }
            cat_images.append(image)
        images.extend(cat_images)
        image_counts[cat_id] = len(cat_images)
        print(i, cat_name, len(cat_images))
    print('# Images', len(images))
    for x in data['categories']:
        x['image_count'] = image_counts[x['id']] if x['id'] in image_counts else 0
    out = {'categories': data['categories'], 'images': images, 'annotations': []}
    print('Writing to', args.out_path)
    json.dump(out, open(args.out_path, 'w'))
