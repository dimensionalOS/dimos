# Copyright (c) Facebook, Inc. and its affiliates.
import os
import torch

import detectron2.data.transforms as T
from detectron2.config import LazyCall as L
from detectron2.layers import ShapeSpec
from detectron2.data.samplers import RepeatFactorTrainingSampler
from detectron2.evaluation.lvis_evaluation import LVISEvaluator
from detectron2.layers.batch_norm import NaiveSyncBatchNorm
from detectron2.solver import WarmupParamScheduler
from detectron2.solver.build import get_default_optimizer_params
from detectron2.modeling.matcher import Matcher
from detectron2.modeling.roi_heads import FastRCNNConvFCHead
from detectron2.modeling.box_regression import Box2BoxTransform
from detectron2.model_zoo import get_config
from fvcore.common.param_scheduler import CosineParamScheduler

from detic.modeling.roi_heads.zero_shot_classifier import ZeroShotClassifier
from detic.modeling.roi_heads.detic_roi_heads import DeticCascadeROIHeads
from detic.modeling.roi_heads.detic_fast_rcnn import DeticFastRCNNOutputLayers

default_configs = get_config('new_baselines/mask_rcnn_R_50_FPN_100ep_LSJ.py')
dataloader = default_configs['dataloader']
model = default_configs['model']
train = default_configs['train']

[model.roi_heads.pop(k) for k in ["box_head", "box_predictor", "proposal_matcher"]]

model.roi_heads.update(
    _target_=DeticCascadeROIHeads,
    num_classes=1203,
    box_heads=[
        L(FastRCNNConvFCHead)(
            input_shape=ShapeSpec(channels=256, height=7, width=7),
            conv_dims=[256, 256, 256, 256],
            fc_dims=[1024],
            conv_norm=lambda c: NaiveSyncBatchNorm(c, stats_mode="N")
        )
        for _ in range(1)
    ],
    box_predictors=[
        L(DeticFastRCNNOutputLayers)(
            input_shape=ShapeSpec(channels=1024),
            test_score_thresh=0.0001,
            test_topk_per_image=300,
            box2box_transform=L(Box2BoxTransform)(weights=(w1, w1, w2, w2)),
            cls_agnostic_bbox_reg=True,
            num_classes="${...num_classes}",
            cls_score=L(ZeroShotClassifier)(
                input_shape=ShapeSpec(channels=1024),
                num_classes=1203,
                zs_weight_path='datasets/metadata/lvis_v1_clip_a+cname.npy',
                norm_weight=True,
                # use_bias=-4.6,
            ),
            use_zeroshot_cls=True,
            use_sigmoid_ce=True,
            ignore_zero_cats=True,
            cat_freq_path='datasets/lvis/lvis_v1_train_norare_cat_info.json'
        )
        for (w1, w2) in [(10, 5)]
    ],
    proposal_matchers=[
        L(Matcher)(thresholds=[th], labels=[0, 1], allow_low_quality_matches=False)
        for th in [0.5]
    ],
)
model.roi_heads.mask_head.num_classes = 1

dataloader.train.dataset.names="lvis_v1_train_norare"
dataloader.train.sampler=L(RepeatFactorTrainingSampler)(
    repeat_factors=L(RepeatFactorTrainingSampler.repeat_factors_from_category_frequency)(
        dataset_dicts="${dataloader.train.dataset}", repeat_thresh=0.001)
)
image_size = 896
dataloader.train.mapper.augmentations = [
    L(T.ResizeScale)(
        min_scale=0.1, max_scale=2.0, target_height=image_size, target_width=image_size
    ),
    L(T.FixedSizeCrop)(crop_size=(image_size, image_size)),
    L(T.RandomFlip)(horizontal=True),
]
dataloader.train.num_workers=32

dataloader.test.dataset.names="lvis_v1_val"
dataloader.evaluator = L(LVISEvaluator)(
    dataset_name="${..test.dataset.names}",
)

num_nodes = 4

dataloader.train.total_batch_size = 64 * num_nodes
train.max_iter = 184375 * 2 // num_nodes

lr_multiplier = L(WarmupParamScheduler)(
    scheduler=CosineParamScheduler(1.0, 0.0),
    warmup_length=500 / train.max_iter,
    warmup_factor=0.067,
)

optimizer = L(torch.optim.AdamW)(
    params=L(get_default_optimizer_params)(
        weight_decay_norm=0.0
    ),
    lr=0.0002 * num_nodes,
    weight_decay=1e-4,
)

train.checkpointer.period=20000 // num_nodes
train.output_dir='./output/Lazy/{}'.format(os.path.basename(__file__)[:-3])