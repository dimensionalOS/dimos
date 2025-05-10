# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import json
import path
import os

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ann", default='datasets/objects365/annotations/zhiyuan_objv2_train_fixname.json')
    parser.add_argument("--img_dir", default='datasets/objects365/train/')
    args = parser.parse_args()

    print('Loading', args.ann)
    data = json.load(open(args.ann, 'r'))
    images = []
    count = 0
    for x in data['images']:
        path = '{}/{}'.format(args.img_dir, x['file_name'])
        if os.path.exists(path):
            images.append(x)
        else:
            print(path)
            count = count + 1
    print('Missing', count, 'images')
    data['images'] = images
    out_name = args.ann[:-5] + '_fixmiss.json'
    print('Saving to', out_name)
    json.dump(data, open(out_name, 'w'))
