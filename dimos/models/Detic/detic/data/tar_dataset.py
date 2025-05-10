#!/usr/bin/env python3
# Copyright (c) Facebook, Inc. and its affiliates.
import os
import gzip
import numpy as np
import io
from PIL import Image
from torch.utils.data import Dataset

try:
    from PIL import UnidentifiedImageError

    unidentified_error_available = True
except ImportError:
    # UnidentifiedImageError isn't available in older versions of PIL
    unidentified_error_available = False

class DiskTarDataset(Dataset):
    def __init__(self, 
        tarfile_path='dataset/imagenet/ImageNet-21k/metadata/tar_files.npy',
        tar_index_dir='dataset/imagenet/ImageNet-21k/metadata/tarindex_npy',
        preload=False, 
        num_synsets="all"):
        """
        - preload (bool): Recommend to set preload to False when using
        - num_synsets (integer or string "all"): set to small number for debugging
            will load subset of dataset
        """
        tar_files = np.load(tarfile_path)

        chunk_datasets = []
        dataset_lens = []
        if isinstance(num_synsets, int):
            assert num_synsets < len(tar_files)
            tar_files = tar_files[:num_synsets]
        for tar_file in tar_files:
            dataset = _TarDataset(tar_file, tar_index_dir, preload=preload)
            chunk_datasets.append(dataset)
            dataset_lens.append(len(dataset))

        self.chunk_datasets = chunk_datasets
        self.dataset_lens = np.array(dataset_lens).astype(np.int32)
        self.dataset_cumsums = np.cumsum(self.dataset_lens)
        self.num_samples = sum(self.dataset_lens)
        labels = np.zeros(self.dataset_lens.sum(), dtype=np.int64)
        sI = 0
        for k in range(len(self.dataset_lens)):
            assert (sI+self.dataset_lens[k]) <= len(labels), f"{k} {sI+self.dataset_lens[k]} vs. {len(labels)}"
            labels[sI:(sI+self.dataset_lens[k])] = k
            sI += self.dataset_lens[k]
        self.labels = labels

    def __len__(self):
        return self.num_samples

    def __getitem__(self, index):
        assert index >= 0 and index < len(self)
        # find the dataset file we need to go to
        d_index = np.searchsorted(self.dataset_cumsums, index)

        # edge case, if index is at edge of chunks, move right
        if index in self.dataset_cumsums:
            d_index += 1

        assert d_index == self.labels[index], f"{d_index} vs. {self.labels[index]} mismatch for {index}"

        # change index to local dataset index
        if d_index == 0:
            local_index = index
        else:
            local_index = index - self.dataset_cumsums[d_index - 1]
        data_bytes = self.chunk_datasets[d_index][local_index]
        exception_to_catch = UnidentifiedImageError if unidentified_error_available else Exception
        try:
            image = Image.open(data_bytes).convert("RGB")
        except exception_to_catch:
            image = Image.fromarray(np.ones((224,224,3), dtype=np.uint8)*128)
            d_index = -1

        # label is the dataset (synset) we indexed into
        return image, d_index, index

    def __repr__(self):
        st = f"DiskTarDataset(subdatasets={len(self.dataset_lens)},samples={self.num_samples})"
        return st

class _TarDataset(object):

    def __init__(self, filename, npy_index_dir, preload=False):
        # translated from
        # fbcode/experimental/deeplearning/matthijs/comp_descs/tardataset.lua
        self.filename = filename
        self.names = []
        self.offsets = []
        self.npy_index_dir = npy_index_dir
        names, offsets = self.load_index()

        self.num_samples = len(names)
        if preload:
            self.data = np.memmap(filename, mode='r', dtype='uint8')
            self.offsets = offsets
        else:
            self.data = None


    def __len__(self):
        return self.num_samples

    def load_index(self):
        basename = os.path.basename(self.filename)
        basename = os.path.splitext(basename)[0]
        names = np.load(os.path.join(self.npy_index_dir, f"{basename}_names.npy"))
        offsets = np.load(os.path.join(self.npy_index_dir, f"{basename}_offsets.npy"))
        return names, offsets

    def __getitem__(self, idx):
        if self.data is None:
            self.data = np.memmap(self.filename, mode='r', dtype='uint8')
            _, self.offsets = self.load_index()

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
            s = io.BytesIO(data.tostring())
            g = gzip.GzipFile(None, 'r', 0, s)
            sdata = g.read()
        else:
            sdata = data.tostring()
        return io.BytesIO(sdata)