# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import copy
import json

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--imagenet_path', default='datasets/imagenet/annotations/imagenet-21k_image_info.json')
    parser.add_argument('--lvis_path', default='datasets/lvis/lvis_v1_train.json')
    parser.add_argument('--save_categories', default='')
    parser.add_argument('--not_save_imagenet', action='store_true')
    parser.add_argument('--not_save_lvis', action='store_true')
    parser.add_argument('--mark', default='lvis-21k')
    args = parser.parse_args()

    print('Loading', args.imagenet_path)
    in_data = json.load(open(args.imagenet_path, 'r'))
    print('Loading', args.lvis_path)
    lvis_data = json.load(open(args.lvis_path, 'r'))

    categories = copy.deepcopy(lvis_data['categories'])
    cat_count = max(x['id'] for x in categories)
    synset2id = {x['synset']: x['id'] for x in categories}
    name2id = {x['name']: x['id'] for x in categories}
    in_id_map = {}
    for x in in_data['categories']:
        if x['synset'] in synset2id:
            in_id_map[x['id']] = synset2id[x['synset']]
        elif x['name'] in name2id:
            in_id_map[x['id']] = name2id[x['name']]
            x['id'] = name2id[x['name']]
        else:
            cat_count = cat_count + 1
            name2id[x['name']] = cat_count
            in_id_map[x['id']] = cat_count
            x['id'] = cat_count
            categories.append(x)
    
    print('lvis cats', len(lvis_data['categories']))
    print('imagenet cats', len(in_data['categories']))
    print('merge cats', len(categories))

    filtered_images = []
    for x in in_data['images']:
        x['pos_category_ids'] = [in_id_map[xx] for xx in x['pos_category_ids']]
        x['pos_category_ids'] = [xx for xx in \
            sorted(set(x['pos_category_ids'])) if xx >= 0]
        if len(x['pos_category_ids']) > 0:
            filtered_images.append(x)

    in_data['categories'] = categories
    lvis_data['categories'] = categories

    if not args.not_save_imagenet:
        in_out_path = args.imagenet_path[:-5] + '_{}.json'.format(args.mark)
        for k, v in in_data.items():
            print('imagenet', k, len(v))
        print('Saving Imagenet to', in_out_path)
        json.dump(in_data, open(in_out_path, 'w'))
    
    if not args.not_save_lvis:
        lvis_out_path = args.lvis_path[:-5] + '_{}.json'.format(args.mark)
        for k, v in lvis_data.items():
            print('lvis', k, len(v))
        print('Saving LVIS to', lvis_out_path)
        json.dump(lvis_data, open(lvis_out_path, 'w'))

    if args.save_categories != '':
        for x in categories:
            for k in ['image_count', 'instance_count', 'synonyms', 'def']:
                if k in x:
                    del x[k]
        CATEGORIES = repr(categories) + "  # noqa"
        open(args.save_categories, 'wt').write(f"CATEGORIES = {CATEGORIES}")
