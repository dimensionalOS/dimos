# Copyright (c) Facebook, Inc. and its affiliates.
import argparse
import json
import os

import numpy as np
from PIL import Image

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--ann", default="datasets/cc3m/Train_GCC-training.tsv")
    parser.add_argument("--save_image_path", default="datasets/cc3m/training/")
    parser.add_argument("--cat_info", default="datasets/lvis/lvis_v1_val.json")
    parser.add_argument("--out_path", default="datasets/cc3m/train_image_info.json")
    parser.add_argument("--not_download_image", action="store_true")
    args = parser.parse_args()
    categories = json.load(open(args.cat_info))["categories"]
    images = []
    if not os.path.exists(args.save_image_path):
        os.makedirs(args.save_image_path)
    f = open(args.ann)
    for i, line in enumerate(f):
        cap, path = line[:-1].split("\t")
        print(i, cap, path)
        if not args.not_download_image:
            os.system(f"wget {path} -O {args.save_image_path}/{i + 1}.jpg")
        try:
            img = Image.open(open(f"{args.save_image_path}/{i + 1}.jpg", "rb"))
            img = np.asarray(img.convert("RGB"))
            h, w = img.shape[:2]
        except:
            continue
        image_info = {
            "id": i + 1,
            "file_name": f"{i + 1}.jpg",
            "height": h,
            "width": w,
            "captions": [cap],
        }
        images.append(image_info)
    data = {"categories": categories, "images": images, "annotations": []}
    for k, v in data.items():
        print(k, len(v))
    print("Saving to", args.out_path)
    json.dump(data, open(args.out_path, "w"))
