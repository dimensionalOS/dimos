# Copyright (c) Facebook, Inc. and its affiliates.
import torch
import json
import numpy as np
from torch.nn import functional as F

def load_class_freq(
    path='datasets/metadata/lvis_v1_train_cat_info.json', freq_weight=1.0):
    cat_info = json.load(open(path, 'r'))
    cat_info = torch.tensor(
        [c['image_count'] for c in sorted(cat_info, key=lambda x: x['id'])])
    freq_weight = cat_info.float() ** freq_weight
    return freq_weight


def get_fed_loss_inds(gt_classes, num_sample_cats, C, weight=None):
    appeared = torch.unique(gt_classes) # C'
    prob = appeared.new_ones(C + 1).float()
    prob[-1] = 0
    if len(appeared) < num_sample_cats:
        if weight is not None:
            prob[:C] = weight.float().clone()
        prob[appeared] = 0
        more_appeared = torch.multinomial(
            prob, num_sample_cats - len(appeared),
            replacement=False)
        appeared = torch.cat([appeared, more_appeared])
    return appeared



def reset_cls_test(model, cls_path, num_classes):
    model.roi_heads.num_classes = num_classes
    if type(cls_path) == str:
        print('Resetting zs_weight', cls_path)
        zs_weight = torch.tensor(
            np.load(cls_path), 
            dtype=torch.float32).permute(1, 0).contiguous() # D x C
    else:
        zs_weight = cls_path
    zs_weight = torch.cat(
        [zs_weight, zs_weight.new_zeros((zs_weight.shape[0], 1))], 
        dim=1) # D x (C + 1)
    if model.roi_heads.box_predictor[0].cls_score.norm_weight:
        zs_weight = F.normalize(zs_weight, p=2, dim=0)
    zs_weight = zs_weight.to(model.device)
    for k in range(len(model.roi_heads.box_predictor)):
        del model.roi_heads.box_predictor[k].cls_score.zs_weight
        model.roi_heads.box_predictor[k].cls_score.zs_weight = zs_weight