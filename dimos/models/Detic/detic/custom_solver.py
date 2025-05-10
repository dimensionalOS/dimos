# Copyright (c) Facebook, Inc. and its affiliates. All Rights Reserved
from enum import Enum
import itertools
from typing import Any, Callable, Dict, Iterable, List, Set, Type, Union
import torch

from detectron2.config import CfgNode

from detectron2.solver.build import maybe_add_gradient_clipping

def match_name_keywords(n, name_keywords):
    out = False
    for b in name_keywords:
        if b in n:
            out = True
            break
    return out

def build_custom_optimizer(cfg: CfgNode, model: torch.nn.Module) -> torch.optim.Optimizer:
    """
    Build an optimizer from config.
    """
    params: List[Dict[str, Any]] = []
    memo: Set[torch.nn.parameter.Parameter] = set()
    custom_multiplier_name = cfg.SOLVER.CUSTOM_MULTIPLIER_NAME
    optimizer_type = cfg.SOLVER.OPTIMIZER
    for key, value in model.named_parameters(recurse=True):
        if not value.requires_grad:
            continue
        # Avoid duplicating parameters
        if value in memo:
            continue
        memo.add(value)
        lr = cfg.SOLVER.BASE_LR
        weight_decay = cfg.SOLVER.WEIGHT_DECAY
        if "backbone" in key:
            lr = lr * cfg.SOLVER.BACKBONE_MULTIPLIER
        if match_name_keywords(key, custom_multiplier_name):
            lr = lr * cfg.SOLVER.CUSTOM_MULTIPLIER
            print('Costum LR', key, lr)
        param = {"params": [value], "lr": lr}
        if optimizer_type != 'ADAMW':
            param['weight_decay'] = weight_decay
        params += [param]

    def maybe_add_full_model_gradient_clipping(optim):  # optim: the optimizer class
        # detectron2 doesn't have full model gradient clipping now
        clip_norm_val = cfg.SOLVER.CLIP_GRADIENTS.CLIP_VALUE
        enable = (
            cfg.SOLVER.CLIP_GRADIENTS.ENABLED
            and cfg.SOLVER.CLIP_GRADIENTS.CLIP_TYPE == "full_model"
            and clip_norm_val > 0.0
        )

        class FullModelGradientClippingOptimizer(optim):
            def step(self, closure=None):
                all_params = itertools.chain(*[x["params"] for x in self.param_groups])
                torch.nn.utils.clip_grad_norm_(all_params, clip_norm_val)
                super().step(closure=closure)

        return FullModelGradientClippingOptimizer if enable else optim

    
    if optimizer_type == 'SGD':
        optimizer = maybe_add_full_model_gradient_clipping(torch.optim.SGD)(
            params, cfg.SOLVER.BASE_LR, momentum=cfg.SOLVER.MOMENTUM, 
            nesterov=cfg.SOLVER.NESTEROV
        )
    elif optimizer_type == 'ADAMW':
        optimizer = maybe_add_full_model_gradient_clipping(torch.optim.AdamW)(
            params, cfg.SOLVER.BASE_LR, 
            weight_decay=cfg.SOLVER.WEIGHT_DECAY
        )
    else:
        raise NotImplementedError(f"no optimizer type {optimizer_type}")
    if not cfg.SOLVER.CLIP_GRADIENTS.CLIP_TYPE == "full_model":
        optimizer = maybe_add_gradient_clipping(cfg, optimizer)
    return optimizer