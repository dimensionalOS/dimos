#!/usr/bin/env python3
# Copyright (c) Facebook, Inc. and its affiliates.

import os
import numpy as np
import sys

sys.path.insert(0, 'third_party/CenterNet2/')
sys.path.insert(0, 'third_party/Deformable-DETR')
from detic.data.tar_dataset import _TarDataset, DiskTarDataset
import pickle
import io
import gzip
import time


class _RawTarDataset(object):

    def __init__(self, filename, indexname, preload=False):
        self.filename = filename
        self.names = []
        self.offsets = []

        for l in open(indexname):
            ll = l.split()
            a, b, c = ll[:3]
            offset = int(b[:-1])
            if l.endswith('** Block of NULs **\n'):
                self.offsets.append(offset)
                break
            else:
                if c.endswith('JPEG'):
                    self.names.append(c)
                    self.offsets.append(offset)
                else:
                    # ignore directories
                    pass
        if preload:
            self.data = np.memmap(filename, mode='r', dtype='uint8')
        else:
            self.data = None

    def __len__(self):
        return len(self.names)

    def __getitem__(self, idx):
        if self.data is None:
            self.data = np.memmap(self.filename, mode='r', dtype='uint8')
        ofs = self.offsets[idx] * 512
        fsize = 512 * (self.offsets[idx + 1] - self.offsets[idx])
        data = self.data[ofs:ofs + fsize]

        if data[:13].tostring() == '././@LongLink':
            data = data[3 * 512:]
        else:
            data = data[512:]

        # just to make it more fun a few JPEGs are GZIP compressed...
        # catch this case
        if tuple(data[:2]) == (0x1f, 0x8b):
            s = io.StringIO(data.tostring())
            g = gzip.GzipFile(None, 'r', 0, s)
            sdata = g.read()
        else:
            sdata = data.tostring()
        return sdata



def preprocess():
    # Follow https://github.com/Alibaba-MIIL/ImageNet21K/blob/main/dataset_preprocessing/processing_script.sh
    # Expect 12358684 samples with 11221 classes
    # ImageNet folder has 21841 classes (synsets)

    i22kdir = '/datasets01/imagenet-22k/062717/'
    i22ktarlogs = '/checkpoint/imisra/datasets/imagenet-22k/tarindex'
    class_names_file = '/checkpoint/imisra/datasets/imagenet-22k/words.txt'

    output_dir = '/checkpoint/zhouxy/Datasets/ImageNet/metadata-22k/'
    i22knpytarlogs = '/checkpoint/zhouxy/Datasets/ImageNet/metadata-22k/tarindex_npy'
    print('Listing dir')
    log_files = os.listdir(i22ktarlogs)
    log_files = [x for x in log_files if x.endswith(".tarlog")]
    log_files.sort()
    chunk_datasets = []
    dataset_lens = []
    min_count = 0
    create_npy_tarlogs = True
    print('Creating folders')
    if create_npy_tarlogs:
        os.makedirs(i22knpytarlogs, exist_ok=True)
        for log_file in log_files:
            syn = log_file.replace(".tarlog", "")
            dataset = _RawTarDataset(os.path.join(i22kdir, syn + ".tar"),
                                    os.path.join(i22ktarlogs, syn + ".tarlog"),
                                    preload=False)
            names = np.array(dataset.names)
            offsets = np.array(dataset.offsets, dtype=np.int64)
            np.save(os.path.join(i22knpytarlogs, f"{syn}_names.npy"), names)
            np.save(os.path.join(i22knpytarlogs, f"{syn}_offsets.npy"), offsets)

    os.makedirs(output_dir, exist_ok=True)

    start_time = time.time()
    for log_file in log_files:
        syn = log_file.replace(".tarlog", "")
        dataset = _TarDataset(os.path.join(i22kdir, syn + ".tar"), i22knpytarlogs)
        # dataset = _RawTarDataset(os.path.join(i22kdir, syn + ".tar"),
        #                             os.path.join(i22ktarlogs, syn + ".tarlog"),
        #                             preload=False)
        dataset_lens.append(len(dataset))
    end_time = time.time()
    print(f"Time {end_time - start_time}")


    dataset_lens = np.array(dataset_lens)
    dataset_valid = dataset_lens > min_count

    syn2class = {}
    with open(class_names_file) as fh:
        for line in fh:
            line = line.strip().split("\t")
            syn2class[line[0]] = line[1]

    tarlog_files = []
    class_names = []
    tar_files = []
    for k in range(len(dataset_valid)):
        if not dataset_valid[k]:
            continue
        syn = log_files[k].replace(".tarlog", "")
        tarlog_files.append(os.path.join(i22ktarlogs, syn + ".tarlog"))
        tar_files.append(os.path.join(i22kdir, syn + ".tar"))
        class_names.append(syn2class[syn])

    tarlog_files = np.array(tarlog_files)
    tar_files = np.array(tar_files)
    class_names = np.array(class_names)
    print(f"Have {len(class_names)} classes and {dataset_lens[dataset_valid].sum()} samples")

    np.save(os.path.join(output_dir, "tarlog_files.npy"), tarlog_files)
    np.save(os.path.join(output_dir, "tar_files.npy"), tar_files)
    np.save(os.path.join(output_dir, "class_names.npy"), class_names)
    np.save(os.path.join(output_dir, "tar_files.npy"), tar_files)


if __name__ == "__main__":
    preprocess()
