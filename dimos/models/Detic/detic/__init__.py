# Copyright (c) Facebook, Inc. and its affiliates.
from .data.datasets import cc, coco_zeroshot, imagenet, lvis_v1, objects365, oid
from .modeling.backbone import swintransformer, timm
from .modeling.meta_arch import custom_rcnn
from .modeling.roi_heads import detic_roi_heads, res5_roi_heads

try:
    from .modeling.meta_arch import d2_deformable_detr
except:
    pass
