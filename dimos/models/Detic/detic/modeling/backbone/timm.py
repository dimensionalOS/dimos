#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Copyright (c) Facebook, Inc. and its affiliates.
import math
from os.path import join
import numpy as np
import copy
from functools import partial

import torch
from torch import nn
import torch.utils.model_zoo as model_zoo
import torch.nn.functional as F
import fvcore.nn.weight_init as weight_init

from detectron2.modeling.backbone import FPN
from detectron2.modeling.backbone.build import BACKBONE_REGISTRY
from detectron2.layers.batch_norm import get_norm, FrozenBatchNorm2d
from detectron2.modeling.backbone import Backbone

from timm import create_model
from timm.models.helpers import build_model_with_cfg
from timm.models.registry import register_model
from timm.models.resnet import ResNet, Bottleneck
from timm.models.resnet import default_cfgs as default_cfgs_resnet
from timm.models.convnext import ConvNeXt, default_cfgs, checkpoint_filter_fn


@register_model
def convnext_tiny_21k(pretrained=False, **kwargs):
    model_args = dict(depths=(3, 3, 9, 3), dims=(96, 192, 384, 768), **kwargs)
    cfg = default_cfgs['convnext_tiny']
    cfg['url'] = 'https://dl.fbaipublicfiles.com/convnext/convnext_tiny_22k_224.pth'
    model = build_model_with_cfg(
        ConvNeXt, 'convnext_tiny', pretrained,
        default_cfg=cfg,
        pretrained_filter_fn=checkpoint_filter_fn,
        feature_cfg=dict(out_indices=(0, 1, 2, 3), flatten_sequential=True),
        **model_args)
    return model

class CustomResNet(ResNet):
    def __init__(self, **kwargs):
        self.out_indices = kwargs.pop('out_indices')
        super().__init__(**kwargs)


    def forward(self, x):
        x = self.conv1(x)
        x = self.bn1(x)
        x = self.act1(x)
        x = self.maxpool(x)
        ret = [x]
        x = self.layer1(x)
        ret.append(x)
        x = self.layer2(x)
        ret.append(x)
        x = self.layer3(x)
        ret.append(x)
        x = self.layer4(x)
        ret.append(x)
        return [ret[i] for i in self.out_indices]


    def load_pretrained(self, cached_file):
        data = torch.load(cached_file, map_location='cpu')
        if 'state_dict' in data:
            self.load_state_dict(data['state_dict'])
        else:
            self.load_state_dict(data)


model_params = {
    'resnet50_in21k': dict(block=Bottleneck, layers=[3, 4, 6, 3]),
}


def create_timm_resnet(variant, out_indices, pretrained=False, **kwargs):
    params = model_params[variant]
    default_cfgs_resnet['resnet50_in21k'] = \
        copy.deepcopy(default_cfgs_resnet['resnet50'])
    default_cfgs_resnet['resnet50_in21k']['url'] = \
        'https://miil-public-eu.oss-eu-central-1.aliyuncs.com/model-zoo/ImageNet_21K_P/models/resnet50_miil_21k.pth'
    default_cfgs_resnet['resnet50_in21k']['num_classes'] = 11221

    return build_model_with_cfg(
        CustomResNet, variant, pretrained,
        default_cfg=default_cfgs_resnet[variant],
        out_indices=out_indices,
        pretrained_custom_load=True,
        **params,
        **kwargs)


class LastLevelP6P7_P5(nn.Module):
    """
    """
    def __init__(self, in_channels, out_channels):
        super().__init__()
        self.num_levels = 2
        self.in_feature = "p5"
        self.p6 = nn.Conv2d(in_channels, out_channels, 3, 2, 1)
        self.p7 = nn.Conv2d(out_channels, out_channels, 3, 2, 1)
        for module in [self.p6, self.p7]:
            weight_init.c2_xavier_fill(module)

    def forward(self, c5):
        p6 = self.p6(c5)
        p7 = self.p7(F.relu(p6))
        return [p6, p7]


def freeze_module(x):
    """
    """
    for p in x.parameters():
        p.requires_grad = False
    FrozenBatchNorm2d.convert_frozen_batchnorm(x)
    return x


class TIMM(Backbone):
    def __init__(self, base_name, out_levels, freeze_at=0, norm='FrozenBN', pretrained=False):
        super().__init__()
        out_indices = [x - 1 for x in out_levels]
        if base_name in model_params:
            self.base = create_timm_resnet(
                base_name, out_indices=out_indices, 
                pretrained=False)
        elif 'eff' in base_name or 'resnet' in base_name or 'regnet' in base_name:
            self.base = create_model(
                base_name, features_only=True, 
                out_indices=out_indices, pretrained=pretrained)
        elif 'convnext' in base_name:
            drop_path_rate = 0.2 \
                if ('tiny' in base_name or 'small' in base_name) else 0.3
            self.base = create_model(
                base_name, features_only=True, 
                out_indices=out_indices, pretrained=pretrained,
                drop_path_rate=drop_path_rate)
        else:
            assert 0, base_name
        feature_info = [dict(num_chs=f['num_chs'], reduction=f['reduction']) \
            for i, f in enumerate(self.base.feature_info)] 
        self._out_features = ['layer{}'.format(x) for x in out_levels]
        self._out_feature_channels = {
            'layer{}'.format(l): feature_info[l - 1]['num_chs'] for l in out_levels}
        self._out_feature_strides = {
            'layer{}'.format(l): feature_info[l - 1]['reduction'] for l in out_levels}
        self._size_divisibility = max(self._out_feature_strides.values())
        if 'resnet' in base_name:
            self.freeze(freeze_at)
        if norm == 'FrozenBN':
            self = FrozenBatchNorm2d.convert_frozen_batchnorm(self)

    def freeze(self, freeze_at=0):
        """
        """
        if freeze_at >= 1:
            print('Frezing', self.base.conv1)
            self.base.conv1 = freeze_module(self.base.conv1)
        if freeze_at >= 2:
            print('Frezing', self.base.layer1)
            self.base.layer1 = freeze_module(self.base.layer1)

    def forward(self, x):
        features = self.base(x)
        ret = {k: v for k, v in zip(self._out_features, features)}
        return ret
    
    @property
    def size_divisibility(self):
        return self._size_divisibility


@BACKBONE_REGISTRY.register()
def build_timm_backbone(cfg, input_shape):
    model = TIMM(
        cfg.MODEL.TIMM.BASE_NAME, 
        cfg.MODEL.TIMM.OUT_LEVELS,
        freeze_at=cfg.MODEL.TIMM.FREEZE_AT,
        norm=cfg.MODEL.TIMM.NORM,
        pretrained=cfg.MODEL.TIMM.PRETRAINED,
    )
    return model


@BACKBONE_REGISTRY.register()
def build_p67_timm_fpn_backbone(cfg, input_shape):
    """
    """
    bottom_up = build_timm_backbone(cfg, input_shape)
    in_features = cfg.MODEL.FPN.IN_FEATURES
    out_channels = cfg.MODEL.FPN.OUT_CHANNELS
    backbone = FPN(
        bottom_up=bottom_up,
        in_features=in_features,
        out_channels=out_channels,
        norm=cfg.MODEL.FPN.NORM,
        top_block=LastLevelP6P7_P5(out_channels, out_channels),
        fuse_type=cfg.MODEL.FPN.FUSE_TYPE,
    )
    return backbone

@BACKBONE_REGISTRY.register()
def build_p35_timm_fpn_backbone(cfg, input_shape):
    """
    """
    bottom_up = build_timm_backbone(cfg, input_shape)
    
    in_features = cfg.MODEL.FPN.IN_FEATURES
    out_channels = cfg.MODEL.FPN.OUT_CHANNELS
    backbone = FPN(
        bottom_up=bottom_up,
        in_features=in_features,
        out_channels=out_channels,
        norm=cfg.MODEL.FPN.NORM,
        top_block=None,
        fuse_type=cfg.MODEL.FPN.FUSE_TYPE,
    )
    return backbone