# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import json
import numpy as np
import pickle
import io
import gzip
import sys
import time
from nltk.corpus import wordnet
from tqdm import tqdm
import operator
import torch

sys.path.insert(0, 'third_party/CenterNet2/')
sys.path.insert(0, 'third_party/Deformable-DETR')
from detic.data.tar_dataset import DiskTarDataset, _TarDataset

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument("--imagenet_dir", default='datasets/imagenet/ImageNet-21k/')
    parser.add_argument("--tarfile_path", default='datasets/imagenet/metadata-22k/tar_files.npy')
    parser.add_argument("--tar_index_dir", default='datasets/imagenet/metadata-22k/tarindex_npy')
    parser.add_argument("--out_path", default='datasets/imagenet/annotations/imagenet-22k_image_info.json')
    parser.add_argument("--workers", default=16, type=int)
    args = parser.parse_args()


    start_time = time.time()
    print('Building dataset')
    dataset = DiskTarDataset(args.tarfile_path, args.tar_index_dir)
    end_time = time.time()
    print(f"Took {end_time-start_time} seconds to make the dataset.")
    print(f"Have {len(dataset)} samples.")
    print('dataset', dataset)
    

    tar_files = np.load(args.tarfile_path)
    categories = []
    for i, tar_file in enumerate(tar_files):
        wnid = tar_file[-13:-4]
        synset = wordnet.synset_from_pos_and_offset('n', int(wnid[1:]))
        synonyms = [x.name() for x in synset.lemmas()]
        category = {
            'id': i + 1,
            'synset': synset.name(),
            'name': synonyms[0],
            'def': synset.definition(),
            'synonyms': synonyms,
        }
        categories.append(category)
    print('categories', len(categories))

    data_loader = torch.utils.data.DataLoader(
        dataset, batch_size=1, shuffle=False,
        num_workers=args.workers,
        collate_fn=operator.itemgetter(0),
    )
    images = []
    for img, label, index in tqdm(data_loader):
        if label == -1:
            continue
        image = {
            'id': int(index) + 1,
            'pos_category_ids': [int(label) + 1],
            'height': int(img.height),
            'width': int(img.width),
            'tar_index': int(index),
        }
        images.append(image)
    
    data = {'categories': categories, 'images': images, 'annotations': []}
    try:
        for k, v in data.items():
            print(k, len(v))
        print('Saving to ', args.out_path)
        json.dump(data, open(args.out_path, 'w'))
    except:
        pass
    import pdb; pdb.set_trace()
    
