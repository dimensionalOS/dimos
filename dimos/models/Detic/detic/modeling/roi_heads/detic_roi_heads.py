# Copyright (c) Facebook, Inc. and its affiliates.
import copy
import numpy as np
import json
import math
import torch
from torch import nn
from torch.autograd.function import Function
from typing import Dict, List, Optional, Tuple, Union
from torch.nn import functional as F

from detectron2.config import configurable
from detectron2.layers import ShapeSpec
from detectron2.layers import batched_nms
from detectron2.structures import Boxes, Instances, pairwise_iou
from detectron2.utils.events import get_event_storage

from detectron2.modeling.box_regression import Box2BoxTransform
from detectron2.modeling.roi_heads.fast_rcnn import fast_rcnn_inference
from detectron2.modeling.roi_heads.roi_heads import ROI_HEADS_REGISTRY, StandardROIHeads
from detectron2.modeling.roi_heads.cascade_rcnn import CascadeROIHeads, _ScaleGradient
from detectron2.modeling.roi_heads.box_head import build_box_head
from .detic_fast_rcnn import DeticFastRCNNOutputLayers
from ..debug import debug_second_stage

from torch.cuda.amp import autocast

@ROI_HEADS_REGISTRY.register()
class DeticCascadeROIHeads(CascadeROIHeads):
    @configurable
    def __init__(
        self,
        *,
        mult_proposal_score: bool = False,
        with_image_labels: bool = False,
        add_image_box: bool = False,
        image_box_size: float = 1.0,
        ws_num_props: int = 512,
        add_feature_to_prop: bool = False,
        mask_weight: float = 1.0,
        one_class_per_proposal: bool = False,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.mult_proposal_score = mult_proposal_score
        self.with_image_labels = with_image_labels
        self.add_image_box = add_image_box
        self.image_box_size = image_box_size
        self.ws_num_props = ws_num_props
        self.add_feature_to_prop = add_feature_to_prop
        self.mask_weight = mask_weight
        self.one_class_per_proposal = one_class_per_proposal

    @classmethod
    def from_config(cls, cfg, input_shape):
        ret = super().from_config(cfg, input_shape)
        ret.update({
            'mult_proposal_score': cfg.MODEL.ROI_BOX_HEAD.MULT_PROPOSAL_SCORE,
            'with_image_labels': cfg.WITH_IMAGE_LABELS,
            'add_image_box': cfg.MODEL.ROI_BOX_HEAD.ADD_IMAGE_BOX,
            'image_box_size': cfg.MODEL.ROI_BOX_HEAD.IMAGE_BOX_SIZE,
            'ws_num_props': cfg.MODEL.ROI_BOX_HEAD.WS_NUM_PROPS,
            'add_feature_to_prop': cfg.MODEL.ROI_BOX_HEAD.ADD_FEATURE_TO_PROP,
            'mask_weight': cfg.MODEL.ROI_HEADS.MASK_WEIGHT,
            'one_class_per_proposal': cfg.MODEL.ROI_HEADS.ONE_CLASS_PER_PROPOSAL,
        })
        return ret


    @classmethod
    def _init_box_head(self, cfg, input_shape):
        ret = super()._init_box_head(cfg, input_shape)
        del ret['box_predictors']
        cascade_bbox_reg_weights = cfg.MODEL.ROI_BOX_CASCADE_HEAD.BBOX_REG_WEIGHTS
        box_predictors = []
        for box_head, bbox_reg_weights in zip(ret['box_heads'], \
            cascade_bbox_reg_weights):
            box_predictors.append(
                DeticFastRCNNOutputLayers(
                    cfg, box_head.output_shape,
                    box2box_transform=Box2BoxTransform(weights=bbox_reg_weights)
                ))
        ret['box_predictors'] = box_predictors
        return ret


    def _forward_box(self, features, proposals, targets=None, 
        ann_type='box', classifier_info=(None,None,None)):
        """
        Add mult proposal scores at testing
        Add ann_type
        """
        if (not self.training) and self.mult_proposal_score:
            if len(proposals) > 0 and proposals[0].has('scores'):
                proposal_scores = [p.get('scores') for p in proposals]
            else:
                proposal_scores = [p.get('objectness_logits') for p in proposals]
        
        features = [features[f] for f in self.box_in_features]
        head_outputs = []  # (predictor, predictions, proposals)
        prev_pred_boxes = None
        image_sizes = [x.image_size for x in proposals]

        for k in range(self.num_cascade_stages):
            if k > 0:
                proposals = self._create_proposals_from_boxes(
                    prev_pred_boxes, image_sizes,
                    logits=[p.objectness_logits for p in proposals])
                if self.training and ann_type in ['box']:
                    proposals = self._match_and_label_boxes(
                        proposals, k, targets)
            predictions = self._run_stage(features, proposals, k, 
                classifier_info=classifier_info)
            prev_pred_boxes = self.box_predictor[k].predict_boxes(
                (predictions[0], predictions[1]), proposals)
            head_outputs.append((self.box_predictor[k], predictions, proposals))
        
        if self.training:
            losses = {}
            storage = get_event_storage()
            for stage, (predictor, predictions, proposals) in enumerate(head_outputs):
                with storage.name_scope("stage{}".format(stage)):
                    if ann_type != 'box': 
                        stage_losses = {}
                        if ann_type in ['image', 'caption', 'captiontag']:
                            image_labels = [x._pos_category_ids for x in targets]
                            weak_losses = predictor.image_label_losses(
                                predictions, proposals, image_labels,
                                classifier_info=classifier_info,
                                ann_type=ann_type)
                            stage_losses.update(weak_losses)
                    else: # supervised
                        stage_losses = predictor.losses(
                            (predictions[0], predictions[1]), proposals,
                            classifier_info=classifier_info)
                        if self.with_image_labels:
                            stage_losses['image_loss'] = \
                                predictions[0].new_zeros([1])[0]
                losses.update({k + "_stage{}".format(stage): v \
                    for k, v in stage_losses.items()})
            return losses
        else:
            # Each is a list[Tensor] of length #image. Each tensor is Ri x (K+1)
            scores_per_stage = [h[0].predict_probs(h[1], h[2]) for h in head_outputs]
            scores = [
                sum(list(scores_per_image)) * (1.0 / self.num_cascade_stages)
                for scores_per_image in zip(*scores_per_stage)
            ]
            if self.mult_proposal_score:
                scores = [(s * ps[:, None]) ** 0.5 \
                    for s, ps in zip(scores, proposal_scores)]
            if self.one_class_per_proposal:
                scores = [s * (s == s[:, :-1].max(dim=1)[0][:, None]).float() for s in scores]
            predictor, predictions, proposals = head_outputs[-1]
            boxes = predictor.predict_boxes(
                (predictions[0], predictions[1]), proposals)
            pred_instances, _ = fast_rcnn_inference(
                boxes,
                scores,
                image_sizes,
                predictor.test_score_thresh,
                predictor.test_nms_thresh,
                predictor.test_topk_per_image,
            )
            return pred_instances


    def forward(self, images, features, proposals, targets=None,
        ann_type='box', classifier_info=(None,None,None)):
        '''
        enable debug and image labels
        classifier_info is shared across the batch
        '''
        if self.training:
            if ann_type in ['box', 'prop', 'proptag']:
                proposals = self.label_and_sample_proposals(
                    proposals, targets)
            else:
                proposals = self.get_top_proposals(proposals)
            
            losses = self._forward_box(features, proposals, targets, \
                ann_type=ann_type, classifier_info=classifier_info)
            if ann_type == 'box' and targets[0].has('gt_masks'):
                mask_losses = self._forward_mask(features, proposals)
                losses.update({k: v * self.mask_weight \
                    for k, v in mask_losses.items()})
                losses.update(self._forward_keypoint(features, proposals))
            else:
                losses.update(self._get_empty_mask_loss(
                    features, proposals,
                    device=proposals[0].objectness_logits.device))
            return proposals, losses
        else:
            pred_instances = self._forward_box(
                features, proposals, classifier_info=classifier_info)
            pred_instances = self.forward_with_given_boxes(features, pred_instances)
            return pred_instances, {}


    def get_top_proposals(self, proposals):
        for i in range(len(proposals)):
            proposals[i].proposal_boxes.clip(proposals[i].image_size)
        proposals = [p[:self.ws_num_props] for p in proposals]
        for i, p in enumerate(proposals):
            p.proposal_boxes.tensor = p.proposal_boxes.tensor.detach()
            if self.add_image_box:
                proposals[i] = self._add_image_box(p)
        return proposals


    def _add_image_box(self, p):
        image_box = Instances(p.image_size)
        n = 1
        h, w = p.image_size
        f = self.image_box_size
        image_box.proposal_boxes = Boxes(
            p.proposal_boxes.tensor.new_tensor(
                [w * (1. - f) / 2., 
                    h * (1. - f) / 2.,
                    w * (1. - (1. - f) / 2.), 
                    h * (1. - (1. - f) / 2.)]
                ).view(n, 4))
        image_box.objectness_logits = p.objectness_logits.new_ones(n)
        return Instances.cat([p, image_box])


    def _get_empty_mask_loss(self, features, proposals, device):
        if self.mask_on:
            return {'loss_mask': torch.zeros(
                (1, ), device=device, dtype=torch.float32)[0]}
        else:
            return {}


    def _create_proposals_from_boxes(self, boxes, image_sizes, logits):
        """
        Add objectness_logits
        """
        boxes = [Boxes(b.detach()) for b in boxes]
        proposals = []
        for boxes_per_image, image_size, logit in zip(
            boxes, image_sizes, logits):
            boxes_per_image.clip(image_size)
            if self.training:
                inds = boxes_per_image.nonempty()
                boxes_per_image = boxes_per_image[inds]
                logit = logit[inds]
            prop = Instances(image_size)
            prop.proposal_boxes = boxes_per_image
            prop.objectness_logits = logit
            proposals.append(prop)
        return proposals


    def _run_stage(self, features, proposals, stage, \
        classifier_info=(None,None,None)):
        """
        Support classifier_info and add_feature_to_prop
        """
        pool_boxes = [x.proposal_boxes for x in proposals]
        box_features = self.box_pooler(features, pool_boxes)
        box_features = _ScaleGradient.apply(box_features, 1.0 / self.num_cascade_stages)
        box_features = self.box_head[stage](box_features)
        if self.add_feature_to_prop:
            feats_per_image = box_features.split(
                [len(p) for p in proposals], dim=0)
            for feat, p in zip(feats_per_image, proposals):
                p.feat = feat
        return self.box_predictor[stage](
            box_features, 
            classifier_info=classifier_info)
