# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import json

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--ann", default='datasets/lvis/lvis_v1_train.json')
    parser.add_argument("--add_freq", action='store_true')
    parser.add_argument("--r_thresh", type=int, default=10)
    parser.add_argument("--c_thresh", type=int, default=100)
    args = parser.parse_args()

    print('Loading', args.ann)
    data = json.load(open(args.ann, 'r'))
    cats = data['categories']
    image_count = {x['id']: set() for x in cats}
    ann_count = {x['id']: 0 for x in cats}
    for x in data['annotations']:
        image_count[x['category_id']].add(x['image_id'])
        ann_count[x['category_id']] += 1
    num_freqs = {x: 0 for x in ['r', 'f', 'c']}
    for x in cats:
        x['image_count'] = len(image_count[x['id']])
        x['instance_count'] = ann_count[x['id']]
        if args.add_freq:
            freq = 'f'
            if x['image_count'] < args.c_thresh:
                freq = 'c'
            if x['image_count'] < args.r_thresh:
                freq = 'r'
            x['frequency'] = freq
            num_freqs[freq] += 1
    print(cats)
    image_counts = sorted([x['image_count'] for x in cats])
    # print('image count', image_counts)
    # import pdb; pdb.set_trace()
    if args.add_freq:
        for x in ['r', 'c', 'f']:
            print(x, num_freqs[x])
    out = cats # {'categories': cats}
    out_path = args.ann[:-5] + '_cat_info.json'
    print('Saving to', out_path)
    json.dump(out, open(out_path, 'w'))
    
