#!/usr/bin/env python
# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
import argparse
import pickle 
import torch

"""
Usage:

cd DETIC_ROOT/models/
wget https://miil-public-eu.oss-eu-central-1.aliyuncs.com/model-zoo/ImageNet_21K_P/models/resnet50_miil_21k.pth
python ../tools/convert-thirdparty-pretrained-model-to-d2.py --path resnet50_miil_21k.pth

wget https://github.com/SwinTransformer/storage/releases/download/v1.0.0/swin_base_patch4_window7_224_22k.pth
python ../tools/convert-thirdparty-pretrained-model-to-d2.py --path swin_base_patch4_window7_224_22k.pth

"""


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--path', default='')
    args = parser.parse_args()

    print('Loading', args.path)
    model = torch.load(args.path, map_location="cpu")
    # import pdb; pdb.set_trace()
    if 'model' in model:
        model = model['model']
    if 'state_dict' in model:
        model = model['state_dict']
    ret = {
        "model": model, 
        "__author__": "third_party", 
        "matching_heuristics": True
    }
    out_path = args.path.replace('.pth', '.pkl')
    print('Saving to', out_path)
    pickle.dump(ret, open(out_path, "wb"))
