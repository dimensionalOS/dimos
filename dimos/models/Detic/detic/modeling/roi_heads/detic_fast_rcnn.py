# Copyright (c) Facebook, Inc. and its affiliates.
import logging
import math
import json
import numpy as np
from typing import Dict, Union
import torch
from fvcore.nn import giou_loss, smooth_l1_loss
from torch import nn
from torch.nn import functional as F
import fvcore.nn.weight_init as weight_init
import detectron2.utils.comm as comm
from detectron2.config import configurable
from detectron2.layers import ShapeSpec, batched_nms, cat, cross_entropy, nonzero_tuple
from detectron2.structures import Boxes, Instances
from detectron2.utils.events import get_event_storage
from detectron2.modeling.box_regression import Box2BoxTransform
from detectron2.modeling.roi_heads.fast_rcnn import FastRCNNOutputLayers
from detectron2.modeling.roi_heads.fast_rcnn import fast_rcnn_inference
from detectron2.modeling.roi_heads.fast_rcnn import _log_classification_stats

from torch.cuda.amp import autocast
from ..utils import load_class_freq, get_fed_loss_inds
from .zero_shot_classifier import ZeroShotClassifier

__all__ = ["DeticFastRCNNOutputLayers"]


class DeticFastRCNNOutputLayers(FastRCNNOutputLayers):
    @configurable
    def __init__(
        self, 
        input_shape: ShapeSpec,
        *,
        mult_proposal_score=False,
        cls_score=None,
        sync_caption_batch = False,
        use_sigmoid_ce = False,
        use_fed_loss = False,
        ignore_zero_cats = False,
        fed_loss_num_cat = 50,
        dynamic_classifier = False,
        image_label_loss = '',
        use_zeroshot_cls = False,
        image_loss_weight = 0.1,
        with_softmax_prop = False,
        caption_weight = 1.0,
        neg_cap_weight = 1.0,
        add_image_box = False,
        debug = False,
        prior_prob = 0.01,
        cat_freq_path = '',
        fed_loss_freq_weight = 0.5,
        softmax_weak_loss = False,
        **kwargs,
    ):
        super().__init__(
            input_shape=input_shape, 
            **kwargs,
        )
        self.mult_proposal_score = mult_proposal_score
        self.sync_caption_batch = sync_caption_batch
        self.use_sigmoid_ce = use_sigmoid_ce
        self.use_fed_loss = use_fed_loss
        self.ignore_zero_cats = ignore_zero_cats
        self.fed_loss_num_cat = fed_loss_num_cat
        self.dynamic_classifier = dynamic_classifier
        self.image_label_loss = image_label_loss
        self.use_zeroshot_cls = use_zeroshot_cls
        self.image_loss_weight = image_loss_weight
        self.with_softmax_prop = with_softmax_prop
        self.caption_weight = caption_weight
        self.neg_cap_weight = neg_cap_weight
        self.add_image_box = add_image_box
        self.softmax_weak_loss = softmax_weak_loss
        self.debug = debug

        if softmax_weak_loss:
            assert image_label_loss in ['max_size'] 

        if self.use_sigmoid_ce:
            bias_value = -math.log((1 - prior_prob) / prior_prob)
            nn.init.constant_(self.cls_score.bias, bias_value)
        
        if self.use_fed_loss or self.ignore_zero_cats:
            freq_weight = load_class_freq(cat_freq_path, fed_loss_freq_weight)
            self.register_buffer('freq_weight', freq_weight)
        else:
            self.freq_weight = None

        if self.use_fed_loss and len(self.freq_weight) < self.num_classes:
            # assert self.num_classes == 11493
            print('Extending federated loss weight')
            self.freq_weight = torch.cat(
                [self.freq_weight, 
                self.freq_weight.new_zeros(
                    self.num_classes - len(self.freq_weight))]
            )

        assert (not self.dynamic_classifier) or (not self.use_fed_loss)
        input_size = input_shape.channels * \
            (input_shape.width or 1) * (input_shape.height or 1)
        
        if self.use_zeroshot_cls:
            del self.cls_score
            del self.bbox_pred
            assert cls_score is not None
            self.cls_score = cls_score
            self.bbox_pred = nn.Sequential(
                nn.Linear(input_size, input_size),
                nn.ReLU(inplace=True),
                nn.Linear(input_size, 4)
            )
            weight_init.c2_xavier_fill(self.bbox_pred[0])
            nn.init.normal_(self.bbox_pred[-1].weight, std=0.001)
            nn.init.constant_(self.bbox_pred[-1].bias, 0)

        if self.with_softmax_prop:
            self.prop_score = nn.Sequential(
                nn.Linear(input_size, input_size),
                nn.ReLU(inplace=True),
                nn.Linear(input_size, self.num_classes + 1),
            )
            weight_init.c2_xavier_fill(self.prop_score[0])
            nn.init.normal_(self.prop_score[-1].weight, mean=0, std=0.001)
            nn.init.constant_(self.prop_score[-1].bias, 0)


    @classmethod
    def from_config(cls, cfg, input_shape):
        ret = super().from_config(cfg, input_shape)
        ret.update({
            'mult_proposal_score': cfg.MODEL.ROI_BOX_HEAD.MULT_PROPOSAL_SCORE,
            'sync_caption_batch': cfg.MODEL.SYNC_CAPTION_BATCH,
            'use_sigmoid_ce': cfg.MODEL.ROI_BOX_HEAD.USE_SIGMOID_CE,
            'use_fed_loss': cfg.MODEL.ROI_BOX_HEAD.USE_FED_LOSS,
            'ignore_zero_cats': cfg.MODEL.ROI_BOX_HEAD.IGNORE_ZERO_CATS,
            'fed_loss_num_cat': cfg.MODEL.ROI_BOX_HEAD.FED_LOSS_NUM_CAT,
            'dynamic_classifier': cfg.MODEL.DYNAMIC_CLASSIFIER,
            'image_label_loss': cfg.MODEL.ROI_BOX_HEAD.IMAGE_LABEL_LOSS,
            'use_zeroshot_cls': cfg.MODEL.ROI_BOX_HEAD.USE_ZEROSHOT_CLS,
            'image_loss_weight': cfg.MODEL.ROI_BOX_HEAD.IMAGE_LOSS_WEIGHT,
            'with_softmax_prop': cfg.MODEL.ROI_BOX_HEAD.WITH_SOFTMAX_PROP,
            'caption_weight': cfg.MODEL.ROI_BOX_HEAD.CAPTION_WEIGHT,
            'neg_cap_weight': cfg.MODEL.ROI_BOX_HEAD.NEG_CAP_WEIGHT,
            'add_image_box': cfg.MODEL.ROI_BOX_HEAD.ADD_IMAGE_BOX,
            'debug': cfg.DEBUG or cfg.SAVE_DEBUG or cfg.IS_DEBUG,
            'prior_prob': cfg.MODEL.ROI_BOX_HEAD.PRIOR_PROB,
            'cat_freq_path': cfg.MODEL.ROI_BOX_HEAD.CAT_FREQ_PATH,
            'fed_loss_freq_weight': cfg.MODEL.ROI_BOX_HEAD.FED_LOSS_FREQ_WEIGHT,
            'softmax_weak_loss': cfg.MODEL.ROI_BOX_HEAD.SOFTMAX_WEAK_LOSS,
        })
        if ret['use_zeroshot_cls']:
            ret['cls_score'] = ZeroShotClassifier(cfg, input_shape)
        return ret

    def losses(self, predictions, proposals, \
        use_advanced_loss=True,
        classifier_info=(None,None,None)):
        """
        enable advanced loss
        """
        scores, proposal_deltas = predictions
        gt_classes = (
            cat([p.gt_classes for p in proposals], dim=0) if len(proposals) else torch.empty(0)
        )
        num_classes = self.num_classes
        if self.dynamic_classifier:
            _, cls_id_map = classifier_info[1]
            gt_classes = cls_id_map[gt_classes]
            num_classes = scores.shape[1] - 1
            assert cls_id_map[self.num_classes] == num_classes
        _log_classification_stats(scores, gt_classes)

        if len(proposals):
            proposal_boxes = cat([p.proposal_boxes.tensor for p in proposals], dim=0)  # Nx4
            assert not proposal_boxes.requires_grad, "Proposals should not require gradients!"
            gt_boxes = cat(
                [(p.gt_boxes if p.has("gt_boxes") else p.proposal_boxes).tensor for p in proposals],
                dim=0,
            )
        else:
            proposal_boxes = gt_boxes = torch.empty((0, 4), device=proposal_deltas.device)

        if self.use_sigmoid_ce:
            loss_cls = self.sigmoid_cross_entropy_loss(scores, gt_classes)
        else:
            loss_cls = self.softmax_cross_entropy_loss(scores, gt_classes)
        return {
            "loss_cls": loss_cls, 
            "loss_box_reg": self.box_reg_loss(
                proposal_boxes, gt_boxes, proposal_deltas, gt_classes, 
                num_classes=num_classes)
        }


    def sigmoid_cross_entropy_loss(self, pred_class_logits, gt_classes):
        if pred_class_logits.numel() == 0:
            return pred_class_logits.new_zeros([1])[0] # This is more robust than .sum() * 0.

        B = pred_class_logits.shape[0]
        C = pred_class_logits.shape[1] - 1

        target = pred_class_logits.new_zeros(B, C + 1)
        target[range(len(gt_classes)), gt_classes] = 1 # B x (C + 1)
        target = target[:, :C] # B x C

        weight = 1
 
        if self.use_fed_loss and (self.freq_weight is not None): # fedloss
            appeared = get_fed_loss_inds(
                gt_classes, 
                num_sample_cats=self.fed_loss_num_cat,
                C=C,
                weight=self.freq_weight)
            appeared_mask = appeared.new_zeros(C + 1)
            appeared_mask[appeared] = 1 # C + 1
            appeared_mask = appeared_mask[:C]
            fed_w = appeared_mask.view(1, C).expand(B, C)
            weight = weight * fed_w.float()
        if self.ignore_zero_cats and (self.freq_weight is not None):
            w = (self.freq_weight.view(-1) > 1e-4).float()
            weight = weight * w.view(1, C).expand(B, C)
            # import pdb; pdb.set_trace()

        cls_loss = F.binary_cross_entropy_with_logits(
            pred_class_logits[:, :-1], target, reduction='none') # B x C
        loss =  torch.sum(cls_loss * weight) / B  
        return loss
        
    
    def softmax_cross_entropy_loss(self, pred_class_logits, gt_classes):
        """
        change _no_instance handling
        """
        if pred_class_logits.numel() == 0:
            return pred_class_logits.new_zeros([1])[0]

        if self.ignore_zero_cats and (self.freq_weight is not None):
            zero_weight = torch.cat([
                (self.freq_weight.view(-1) > 1e-4).float(),
                self.freq_weight.new_ones(1)]) # C + 1
            loss = F.cross_entropy(
                pred_class_logits, gt_classes, 
                weight=zero_weight, reduction="mean")
        elif self.use_fed_loss and (self.freq_weight is not None): # fedloss
            C = pred_class_logits.shape[1] - 1
            appeared = get_fed_loss_inds(
                gt_classes, 
                num_sample_cats=self.fed_loss_num_cat,
                C=C,
                weight=self.freq_weight)
            appeared_mask = appeared.new_zeros(C + 1).float()
            appeared_mask[appeared] = 1. # C + 1
            appeared_mask[C] = 1.
            loss = F.cross_entropy(
                pred_class_logits, gt_classes, 
                weight=appeared_mask, reduction="mean")        
        else:
            loss = F.cross_entropy(
                pred_class_logits, gt_classes, reduction="mean")                  
        return loss


    def box_reg_loss(
        self, proposal_boxes, gt_boxes, pred_deltas, gt_classes, 
        num_classes=-1):
        """
        Allow custom background index
        """
        num_classes = num_classes if num_classes > 0 else self.num_classes
        box_dim = proposal_boxes.shape[1]  # 4 or 5
        fg_inds = nonzero_tuple((gt_classes >= 0) & (gt_classes < num_classes))[0]
        if pred_deltas.shape[1] == box_dim:  # cls-agnostic regression
            fg_pred_deltas = pred_deltas[fg_inds]
        else:
            fg_pred_deltas = pred_deltas.view(-1, self.num_classes, box_dim)[
                fg_inds, gt_classes[fg_inds]
            ]

        if self.box_reg_loss_type == "smooth_l1":
            gt_pred_deltas = self.box2box_transform.get_deltas(
                proposal_boxes[fg_inds],
                gt_boxes[fg_inds],
            )
            loss_box_reg = smooth_l1_loss(
                fg_pred_deltas, gt_pred_deltas, self.smooth_l1_beta, reduction="sum"
            )
        elif self.box_reg_loss_type == "giou":
            fg_pred_boxes = self.box2box_transform.apply_deltas(
                fg_pred_deltas, proposal_boxes[fg_inds]
            )
            loss_box_reg = giou_loss(fg_pred_boxes, gt_boxes[fg_inds], reduction="sum")
        else:
            raise ValueError(f"Invalid bbox reg loss type '{self.box_reg_loss_type}'")
        return loss_box_reg / max(gt_classes.numel(), 1.0)

    def inference(self, predictions, proposals):
        """
        enable use proposal boxes
        """
        predictions = (predictions[0], predictions[1])
        boxes = self.predict_boxes(predictions, proposals)
        scores = self.predict_probs(predictions, proposals)
        if self.mult_proposal_score:
            proposal_scores = [p.get('objectness_logits') for p in proposals]
            scores = [(s * ps[:, None]) ** 0.5 \
                for s, ps in zip(scores, proposal_scores)]
        image_shapes = [x.image_size for x in proposals]
        return fast_rcnn_inference(
            boxes,
            scores,
            image_shapes,
            self.test_score_thresh,
            self.test_nms_thresh,
            self.test_topk_per_image,
        )


    def predict_probs(self, predictions, proposals):
        """
        support sigmoid
        """
        # scores, _ = predictions
        scores = predictions[0]
        num_inst_per_image = [len(p) for p in proposals]
        if self.use_sigmoid_ce:
            probs = scores.sigmoid()
        else:
            probs = F.softmax(scores, dim=-1)
        return probs.split(num_inst_per_image, dim=0)


    def image_label_losses(self, predictions, proposals, image_labels, \
        classifier_info=(None,None,None), ann_type='image'):
        '''
        Inputs:
            scores: N x (C + 1)
            image_labels B x 1
        '''
        num_inst_per_image = [len(p) for p in proposals]
        scores = predictions[0]
        scores = scores.split(num_inst_per_image, dim=0) # B x n x (C + 1)
        if self.with_softmax_prop:
            prop_scores = predictions[2].split(num_inst_per_image, dim=0)
        else:
            prop_scores = [None for _ in num_inst_per_image]
        B = len(scores)
        img_box_count = 0
        select_size_count = 0
        select_x_count = 0
        select_y_count = 0
        max_score_count = 0
        storage = get_event_storage()
        loss = scores[0].new_zeros([1])[0]
        caption_loss = scores[0].new_zeros([1])[0]
        for idx, (score, labels, prop_score, p) in enumerate(zip(
            scores, image_labels, prop_scores, proposals)):
            if score.shape[0] == 0:
                loss += score.new_zeros([1])[0]
                continue
            if 'caption' in ann_type:
                score, caption_loss_img = self._caption_loss(
                    score, classifier_info, idx, B)
                caption_loss += self.caption_weight * caption_loss_img
                if ann_type == 'caption':
                    continue

            if self.debug:
                p.selected = score.new_zeros(
                    (len(p),), dtype=torch.long) - 1
            for i_l, label in enumerate(labels):
                if self.dynamic_classifier:
                    if idx == 0 and i_l == 0 and comm.is_main_process():
                        storage.put_scalar('stats_label', label)
                    label = classifier_info[1][1][label]
                    assert label < score.shape[1]
                if self.image_label_loss in ['wsod', 'wsddn']: 
                    loss_i, ind = self._wsddn_loss(score, prop_score, label)
                elif self.image_label_loss == 'max_score':
                    loss_i, ind = self._max_score_loss(score, label)
                elif self.image_label_loss == 'max_size':
                    loss_i, ind = self._max_size_loss(score, label, p)
                elif self.image_label_loss == 'first':
                    loss_i, ind = self._first_loss(score, label)
                elif self.image_label_loss == 'image':
                    loss_i, ind = self._image_loss(score, label)
                elif self.image_label_loss == 'min_loss':
                    loss_i, ind = self._min_loss_loss(score, label)
                else:
                    assert 0
                loss += loss_i / len(labels)
                if type(ind) == type([]):
                    img_box_count = sum(ind) / len(ind)
                    if self.debug:
                        for ind_i in ind:
                            p.selected[ind_i] = label
                else:
                    img_box_count = ind
                    select_size_count = p[ind].proposal_boxes.area() / \
                        (p.image_size[0] * p.image_size[1])
                    max_score_count = score[ind, label].sigmoid()
                    select_x_count = (p.proposal_boxes.tensor[ind, 0] + \
                        p.proposal_boxes.tensor[ind, 2]) / 2 / p.image_size[1]
                    select_y_count = (p.proposal_boxes.tensor[ind, 1] + \
                        p.proposal_boxes.tensor[ind, 3]) / 2 / p.image_size[0]
                    if self.debug:
                        p.selected[ind] = label

        loss = loss / B
        storage.put_scalar('stats_l_image', loss.item())
        if 'caption' in ann_type:
            caption_loss = caption_loss / B
            loss = loss + caption_loss
            storage.put_scalar('stats_l_caption', caption_loss.item())
        if comm.is_main_process():
            storage.put_scalar('pool_stats', img_box_count)
            storage.put_scalar('stats_select_size', select_size_count)
            storage.put_scalar('stats_select_x', select_x_count)
            storage.put_scalar('stats_select_y', select_y_count)
            storage.put_scalar('stats_max_label_score', max_score_count)

        return {
            'image_loss': loss * self.image_loss_weight,
            'loss_cls': score.new_zeros([1])[0],
            'loss_box_reg': score.new_zeros([1])[0]}


    def forward(self, x, classifier_info=(None,None,None)):
        """
        enable classifier_info
        """
        if x.dim() > 2:
            x = torch.flatten(x, start_dim=1)
        scores = []
   
        if classifier_info[0] is not None:
            cls_scores = self.cls_score(x, classifier=classifier_info[0])
            scores.append(cls_scores)
        else:
            cls_scores = self.cls_score(x)
            scores.append(cls_scores)

        if classifier_info[2] is not None:
            cap_cls = classifier_info[2]
            if self.sync_caption_batch:
                caption_scores = self.cls_score(x, classifier=cap_cls[:, :-1]) 
            else:
                caption_scores = self.cls_score(x, classifier=cap_cls)
            scores.append(caption_scores)
        scores = torch.cat(scores, dim=1) # B x C' or B x N or B x (C'+N)

        proposal_deltas = self.bbox_pred(x)
        if self.with_softmax_prop:
            prop_score = self.prop_score(x)
            return scores, proposal_deltas, prop_score
        else:
            return scores, proposal_deltas


    def _caption_loss(self, score, classifier_info, idx, B):
        assert (classifier_info[2] is not None)
        assert self.add_image_box
        cls_and_cap_num = score.shape[1]
        cap_num = classifier_info[2].shape[0]
        score, caption_score = score.split(
            [cls_and_cap_num - cap_num, cap_num], dim=1)
        # n x (C + 1), n x B
        caption_score = caption_score[-1:] # 1 x B # -1: image level box
        caption_target = caption_score.new_zeros(
            caption_score.shape) # 1 x B or 1 x MB, M: num machines
        if self.sync_caption_batch:
            # caption_target: 1 x MB
            rank = comm.get_rank()
            global_idx = B * rank + idx
            assert (classifier_info[2][
                global_idx, -1] - rank) ** 2 < 1e-8, \
                    '{} {} {} {} {}'.format(
                        rank, global_idx, 
                        classifier_info[2][global_idx, -1],
                        classifier_info[2].shape, 
                        classifier_info[2][:, -1])
            caption_target[:, global_idx] = 1.
        else:
            assert caption_score.shape[1] == B
            caption_target[:, idx] = 1.
        caption_loss_img = F.binary_cross_entropy_with_logits(
                caption_score, caption_target, reduction='none')
        if self.sync_caption_batch:
            fg_mask = (caption_target > 0.5).float()
            assert (fg_mask.sum().item() - 1.) ** 2 < 1e-8, '{} {}'.format(
                fg_mask.shape, fg_mask)
            pos_loss = (caption_loss_img * fg_mask).sum()
            neg_loss = (caption_loss_img * (1. - fg_mask)).sum()
            caption_loss_img = pos_loss + self.neg_cap_weight * neg_loss
        else:
            caption_loss_img = caption_loss_img.sum()
        return score, caption_loss_img


    def _wsddn_loss(self, score, prop_score, label):
        assert prop_score is not None
        loss = 0
        final_score = score.sigmoid() * \
            F.softmax(prop_score, dim=0) # B x (C + 1)
        img_score = torch.clamp(
            torch.sum(final_score, dim=0), 
            min=1e-10, max=1-1e-10) # (C + 1)
        target = img_score.new_zeros(img_score.shape) # (C + 1)
        target[label] = 1.
        loss += F.binary_cross_entropy(img_score, target)
        ind = final_score[:, label].argmax()
        return loss, ind


    def _max_score_loss(self, score, label):
        loss = 0
        target = score.new_zeros(score.shape[1])
        target[label] = 1.
        ind = score[:, label].argmax().item()
        loss += F.binary_cross_entropy_with_logits(
            score[ind], target, reduction='sum')
        return loss, ind


    def _min_loss_loss(self, score, label):
        loss = 0
        target = score.new_zeros(score.shape)
        target[:, label] = 1.
        with torch.no_grad():
            x = F.binary_cross_entropy_with_logits(
                score, target, reduction='none').sum(dim=1) # n
        ind = x.argmin().item()
        loss += F.binary_cross_entropy_with_logits(
            score[ind], target[0], reduction='sum')
        return loss, ind


    def _first_loss(self, score, label):
        loss = 0
        target = score.new_zeros(score.shape[1])
        target[label] = 1.
        ind = 0
        loss += F.binary_cross_entropy_with_logits(
            score[ind], target, reduction='sum')
        return loss, ind


    def _image_loss(self, score, label):
        assert self.add_image_box
        target = score.new_zeros(score.shape[1])
        target[label] = 1.
        ind = score.shape[0] - 1
        loss = F.binary_cross_entropy_with_logits(
            score[ind], target, reduction='sum')
        return loss, ind


    def _max_size_loss(self, score, label, p):
        loss = 0
        target = score.new_zeros(score.shape[1])
        target[label] = 1.
        sizes = p.proposal_boxes.area()
        ind = sizes[:-1].argmax().item() if len(sizes) > 1 else 0
        if self.softmax_weak_loss:
            loss += F.cross_entropy(
                score[ind:ind+1], 
                score.new_tensor(label, dtype=torch.long).view(1), 
                reduction='sum')
        else:
            loss += F.binary_cross_entropy_with_logits(
                score[ind], target, reduction='sum')
        return loss, ind



def put_label_distribution(storage, hist_name, hist_counts, num_classes):
    """
    """
    ht_min, ht_max = 0, num_classes
    hist_edges = torch.linspace(
        start=ht_min, end=ht_max, steps=num_classes + 1, dtype=torch.float32)

    hist_params = dict(
        tag=hist_name,
        min=ht_min,
        max=ht_max,
        num=float(hist_counts.sum()),
        sum=float((hist_counts * torch.arange(len(hist_counts))).sum()),
        sum_squares=float(((hist_counts * torch.arange(len(hist_counts))) ** 2).sum()),
        bucket_limits=hist_edges[1:].tolist(),
        bucket_counts=hist_counts.tolist(),
        global_step=storage._iter,
    )
    storage._histograms.append(hist_params)