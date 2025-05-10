# Copyright (c) Facebook, Inc. and its affiliates.
import os
import argparse

if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--src_path', default='datasets/imagenet/ImageNet-21K/')
    parser.add_argument('--dst_path', default='datasets/imagenet/ImageNet-LVIS/')
    parser.add_argument('--data_path', default='datasets/imagenet_lvis_wnid.txt')
    args = parser.parse_args()

    f = open(args.data_path)
    for i, line in enumerate(f):
      cmd = 'mkdir {x} && tar -xf {src}/{l}.tar -C {x}'.format(
          src=args.src_path,
          l=line.strip(),
          x=args.dst_path + '/' + line.strip())
      print(i, cmd)
      os.system(cmd)
