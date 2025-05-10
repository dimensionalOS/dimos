# ------------------------------------------------------------------------
# Deformable DETR
# Copyright (c) 2020 SenseTime. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 [see LICENSE for details]
# ------------------------------------------------------------------------

"""
Benchmark inference speed of Deformable DETR.
"""
import os
import time
import argparse

import torch

from main import get_args_parser as get_main_args_parser
from models import build_model
from datasets import build_dataset
from util.misc import nested_tensor_from_tensor_list


def get_benckmark_arg_parser():
    parser = argparse.ArgumentParser('Benchmark inference speed of Deformable DETR.')
    parser.add_argument('--num_iters', type=int, default=300, help='total iters to benchmark speed')
    parser.add_argument('--warm_iters', type=int, default=5, help='ignore first several iters that are very slow')
    parser.add_argument('--batch_size', type=int, default=1, help='batch size in inference')
    parser.add_argument('--resume', type=str, help='load the pre-trained checkpoint')
    return parser


@torch.no_grad()
def measure_average_inference_time(model, inputs, num_iters=100, warm_iters=5):
    ts = []
    for iter_ in range(num_iters):
        torch.cuda.synchronize()
        t_ = time.perf_counter()
        model(inputs)
        torch.cuda.synchronize()
        t = time.perf_counter() - t_
        if iter_ >= warm_iters:
          ts.append(t)
    print(ts)
    return sum(ts) / len(ts)


def benchmark():
    args, _ = get_benckmark_arg_parser().parse_known_args()
    main_args = get_main_args_parser().parse_args(_)
    assert args.warm_iters < args.num_iters and args.num_iters > 0 and args.warm_iters >= 0
    assert args.batch_size > 0
    assert args.resume is None or os.path.exists(args.resume)
    dataset = build_dataset('val', main_args)
    model, _, _ = build_model(main_args)
    model.cuda()
    model.eval()
    if args.resume is not None:
        ckpt = torch.load(args.resume, map_location=lambda storage, loc: storage)
        model.load_state_dict(ckpt['model'])
    inputs = nested_tensor_from_tensor_list([dataset.__getitem__(0)[0].cuda() for _ in range(args.batch_size)])
    t = measure_average_inference_time(model, inputs, args.num_iters, args.warm_iters)
    return 1.0 / t * args.batch_size


if __name__ == '__main__':
    fps = benchmark()
    print(f'Inference Speed: {fps:.1f} FPS')

