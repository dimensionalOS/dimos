# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np
import open_clip
import torch
import torch.nn.functional as F
from PIL import Image


def test_embed():
    # 1) Pick a MobileCLIP variant that OpenCLIP exposes directly
    # Good starts: 'MobileCLIP-S2' or 'MobileCLIP-B' with pretrained='datacompdr'
    model_name = "MobileCLIP-S2"
    pretrained = "datacompdr"  # OpenCLIP key
    device = "cuda"

    model, _, preprocess = open_clip.create_model_and_transforms(model_name, pretrained=pretrained)
    tokenizer = open_clip.get_tokenizer(model_name)
    model = model.eval().to(device)

    # 2) Encode an image (or crops) → unit-norm embedding
    def embed_images(imgs_rgb: list[Image.Image]) -> np.ndarray:
        with torch.inference_mode(), torch.cuda.amp.autocast(True):
            batch = torch.stack([preprocess(im.convert("RGB")) for im in imgs_rgb]).to(device)
            feats = model.encode_image(batch)
            feats = F.normalize(feats, dim=-1)
        return feats.detach().cpu().numpy()

    # 3) Cosine distance for re-ID
    def cosine_distance(u, v):  # u,v are L2-normalized
        return 1.0 - float((u @ v))

    # Example
    im = Image.open("person_crop.jpg")
    emb = embed_images([im])[0]
    print(emb.shape)  # e.g. (512,) depending on backbone
