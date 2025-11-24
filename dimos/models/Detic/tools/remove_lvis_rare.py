# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import json

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--ann', default='datasets/lvis/lvis_v1_train.json')
    args = parser.parse_args()

    print('Loading', args.ann)
    data = json.load(open(args.ann, 'r'))
    catid2freq = {x['id']: x['frequency'] for x in data['categories']}
    print('ori #anns', len(data['annotations']))
    exclude = ['r']
    data['annotations'] = [x for x in data['annotations'] \
        if catid2freq[x['category_id']] not in exclude]
    print('filtered #anns', len(data['annotations']))
    out_path = args.ann[:-5] + '_norare.json'
    print('Saving to', out_path)
    json.dump(data, open(out_path, 'w'))
