# Copyright (c) Facebook, Inc. and its affiliates. 
import torch
import torch.nn.functional as F
from torch import nn
import math

from detectron2.modeling import META_ARCH_REGISTRY, build_backbone
from detectron2.structures import Boxes, Instances
from ..utils import load_class_freq, get_fed_loss_inds

from models.backbone import Joiner
from models.deformable_detr import DeformableDETR, SetCriterion, MLP
from models.deformable_detr import _get_clones
from models.matcher import HungarianMatcher
from models.position_encoding import PositionEmbeddingSine
from models.deformable_transformer import DeformableTransformer
from models.segmentation import sigmoid_focal_loss
from util.box_ops import box_cxcywh_to_xyxy, box_xyxy_to_cxcywh
from util.misc import NestedTensor, accuracy


__all__ = ["DeformableDetr"]

class CustomSetCriterion(SetCriterion):
    def __init__(self, num_classes, matcher, weight_dict, losses, \
        focal_alpha=0.25, use_fed_loss=False):
        super().__init__(num_classes, matcher, weight_dict, losses, focal_alpha)
        self.use_fed_loss = use_fed_loss
        if self.use_fed_loss:
            self.register_buffer(
                'fed_loss_weight', load_class_freq(freq_weight=0.5))

    def loss_labels(self, outputs, targets, indices, num_boxes, log=True):
        """Classification loss (NLL)
        targets dicts must contain the key "labels" containing a tensor of dim [nb_target_boxes]
        """
        assert 'pred_logits' in outputs
        src_logits = outputs['pred_logits']

        idx = self._get_src_permutation_idx(indices)
        target_classes_o = torch.cat([t["labels"][J] for t, (_, J) in zip(targets, indices)])
        target_classes = torch.full(src_logits.shape[:2], self.num_classes,
                                    dtype=torch.int64, device=src_logits.device)
        target_classes[idx] = target_classes_o

        target_classes_onehot = torch.zeros(
            [src_logits.shape[0], src_logits.shape[1], src_logits.shape[2] + 1],
            dtype=src_logits.dtype, layout=src_logits.layout, 
            device=src_logits.device)
        target_classes_onehot.scatter_(2, target_classes.unsqueeze(-1), 1)

        target_classes_onehot = target_classes_onehot[:,:,:-1] # B x N x C
        if self.use_fed_loss:
            inds = get_fed_loss_inds(
                gt_classes=target_classes_o,
                num_sample_cats=50,
                weight=self.fed_loss_weight,
                C=target_classes_onehot.shape[2])
            loss_ce = sigmoid_focal_loss(
                src_logits[:, :, inds], 
                target_classes_onehot[:, :, inds], 
                num_boxes, 
                alpha=self.focal_alpha, 
                gamma=2) * src_logits.shape[1]
        else:
            loss_ce = sigmoid_focal_loss(
                src_logits, target_classes_onehot, num_boxes, 
                alpha=self.focal_alpha, 
                gamma=2) * src_logits.shape[1]
        losses = {'loss_ce': loss_ce}

        if log:
            # TODO this should probably be a separate loss, not hacked in this one here
            losses['class_error'] = 100 - accuracy(src_logits[idx], target_classes_o)[0]
        return losses


class MaskedBackbone(nn.Module):
    """ This is a thin wrapper around D2's backbone to provide padding masking"""

    def __init__(self, cfg):
        super().__init__()
        self.backbone = build_backbone(cfg)
        backbone_shape = self.backbone.output_shape()
        self.feature_strides = [backbone_shape[f].stride for f in backbone_shape.keys()]
        self.strides = [backbone_shape[f].stride for f in backbone_shape.keys()]
        self.num_channels = [backbone_shape[x].channels for x in backbone_shape.keys()]

    def forward(self, tensor_list: NestedTensor):
        xs = self.backbone(tensor_list.tensors)
        out = {}
        for name, x in xs.items():
            m = tensor_list.mask
            assert m is not None
            mask = F.interpolate(m[None].float(), size=x.shape[-2:]).to(torch.bool)[0]
            out[name] = NestedTensor(x, mask)
        return out

@META_ARCH_REGISTRY.register()
class DeformableDetr(nn.Module):
    """
    Implement Deformable Detr
    """

    def __init__(self, cfg):
        super().__init__()
        self.with_image_labels = cfg.WITH_IMAGE_LABELS
        self.weak_weight = cfg.MODEL.DETR.WEAK_WEIGHT

        self.device = torch.device(cfg.MODEL.DEVICE)
        self.test_topk = cfg.TEST.DETECTIONS_PER_IMAGE
        self.num_classes = cfg.MODEL.DETR.NUM_CLASSES
        self.mask_on = cfg.MODEL.MASK_ON
        hidden_dim = cfg.MODEL.DETR.HIDDEN_DIM
        num_queries = cfg.MODEL.DETR.NUM_OBJECT_QUERIES

        # Transformer parameters:
        nheads = cfg.MODEL.DETR.NHEADS
        dropout = cfg.MODEL.DETR.DROPOUT
        dim_feedforward = cfg.MODEL.DETR.DIM_FEEDFORWARD
        enc_layers = cfg.MODEL.DETR.ENC_LAYERS
        dec_layers = cfg.MODEL.DETR.DEC_LAYERS
        num_feature_levels = cfg.MODEL.DETR.NUM_FEATURE_LEVELS
        two_stage = cfg.MODEL.DETR.TWO_STAGE
        with_box_refine = cfg.MODEL.DETR.WITH_BOX_REFINE

        # Loss parameters:
        giou_weight = cfg.MODEL.DETR.GIOU_WEIGHT
        l1_weight = cfg.MODEL.DETR.L1_WEIGHT
        deep_supervision = cfg.MODEL.DETR.DEEP_SUPERVISION
        cls_weight = cfg.MODEL.DETR.CLS_WEIGHT
        focal_alpha = cfg.MODEL.DETR.FOCAL_ALPHA

        N_steps = hidden_dim // 2
        d2_backbone = MaskedBackbone(cfg)
        backbone = Joiner(d2_backbone, PositionEmbeddingSine(N_steps, normalize=True))

        transformer = DeformableTransformer(
            d_model=hidden_dim,
            nhead=nheads,
            num_encoder_layers=enc_layers,
            num_decoder_layers=dec_layers,
            dim_feedforward=dim_feedforward,
            dropout=dropout,
            activation="relu",
            return_intermediate_dec=True,
            num_feature_levels=num_feature_levels,
            dec_n_points=4,
            enc_n_points=4,
            two_stage=two_stage,
            two_stage_num_proposals=num_queries)

        self.detr = DeformableDETR(
            backbone, transformer, num_classes=self.num_classes, 
            num_queries=num_queries,
            num_feature_levels=num_feature_levels,
            aux_loss=deep_supervision,
            with_box_refine=with_box_refine,
            two_stage=two_stage,
        )

        if self.mask_on:
            assert 0, 'Mask is not supported yet :('

        matcher = HungarianMatcher(
            cost_class=cls_weight, cost_bbox=l1_weight, cost_giou=giou_weight)
        weight_dict = {"loss_ce": cls_weight, "loss_bbox": l1_weight}
        weight_dict["loss_giou"] = giou_weight
        if deep_supervision:
            aux_weight_dict = {}
            for i in range(dec_layers - 1):
                aux_weight_dict.update({k + f"_{i}": v for k, v in weight_dict.items()})
            weight_dict.update(aux_weight_dict)
        print('weight_dict', weight_dict)
        losses = ["labels", "boxes", "cardinality"]
        if self.mask_on:
            losses += ["masks"]
        self.criterion = CustomSetCriterion(
            self.num_classes, matcher=matcher, weight_dict=weight_dict, 
            focal_alpha=focal_alpha, 
            losses=losses,
            use_fed_loss=cfg.MODEL.DETR.USE_FED_LOSS
        )
        pixel_mean = torch.Tensor(cfg.MODEL.PIXEL_MEAN).to(self.device).view(3, 1, 1)
        pixel_std = torch.Tensor(cfg.MODEL.PIXEL_STD).to(self.device).view(3, 1, 1)
        self.normalizer = lambda x: (x - pixel_mean) / pixel_std


    def forward(self, batched_inputs):
        """
        Args:
        Returns:
            dict[str: Tensor]:
                mapping from a named loss to a tensor storing the loss. Used during training only.
        """
        images = self.preprocess_image(batched_inputs)
        output = self.detr(images)
        if self.training:
            gt_instances = [x["instances"].to(self.device) for x in batched_inputs]
            targets = self.prepare_targets(gt_instances)
            loss_dict = self.criterion(output, targets)
            weight_dict = self.criterion.weight_dict
            for k in loss_dict.keys():
                if k in weight_dict:
                    loss_dict[k] *= weight_dict[k]
            if self.with_image_labels:
                if batched_inputs[0]['ann_type'] in ['image', 'captiontag']:
                    loss_dict['loss_image'] = self.weak_weight * self._weak_loss(
                        output, batched_inputs)
                else:
                    loss_dict['loss_image'] = images[0].new_zeros(
                        [1], dtype=torch.float32)[0]
                # import pdb; pdb.set_trace()
            return loss_dict
        else:
            image_sizes = output["pred_boxes"].new_tensor(
                [(t["height"], t["width"]) for t in batched_inputs])
            results = self.post_process(output, image_sizes)
            return results


    def prepare_targets(self, targets):
        new_targets = []
        for targets_per_image in targets:
            h, w = targets_per_image.image_size
            image_size_xyxy = torch.as_tensor([w, h, w, h], dtype=torch.float, device=self.device)
            gt_classes = targets_per_image.gt_classes
            gt_boxes = targets_per_image.gt_boxes.tensor / image_size_xyxy
            gt_boxes = box_xyxy_to_cxcywh(gt_boxes)
            new_targets.append({"labels": gt_classes, "boxes": gt_boxes})
            if self.mask_on and hasattr(targets_per_image, 'gt_masks'):
                assert 0, 'Mask is not supported yet :('
                gt_masks = targets_per_image.gt_masks
                gt_masks = convert_coco_poly_to_mask(gt_masks.polygons, h, w)
                new_targets[-1].update({'masks': gt_masks})
        return new_targets


    def post_process(self, outputs, target_sizes):
        """
        """
        out_logits, out_bbox = outputs['pred_logits'], outputs['pred_boxes']
        assert len(out_logits) == len(target_sizes)
        assert target_sizes.shape[1] == 2

        prob = out_logits.sigmoid()
        topk_values, topk_indexes = torch.topk(
            prob.view(out_logits.shape[0], -1), self.test_topk, dim=1)
        scores = topk_values
        topk_boxes = topk_indexes // out_logits.shape[2]
        labels = topk_indexes % out_logits.shape[2]
        boxes = box_cxcywh_to_xyxy(out_bbox)
        boxes = torch.gather(boxes, 1, topk_boxes.unsqueeze(-1).repeat(1,1,4))

        # and from relative [0, 1] to absolute [0, height] coordinates
        img_h, img_w = target_sizes.unbind(1)
        scale_fct = torch.stack([img_w, img_h, img_w, img_h], dim=1)
        boxes = boxes * scale_fct[:, None, :]

        results = []
        for s, l, b, size in zip(scores, labels, boxes, target_sizes):
            r = Instances((size[0], size[1]))
            r.pred_boxes = Boxes(b)
            r.scores = s
            r.pred_classes = l
            results.append({'instances': r})
        return results


    def preprocess_image(self, batched_inputs):
        """
        Normalize, pad and batch the input images.
        """
        images = [self.normalizer(x["image"].to(self.device)) for x in batched_inputs]
        return images


    def _weak_loss(self, outputs, batched_inputs):
        loss = 0
        for b, x in enumerate(batched_inputs):
            labels = x['pos_category_ids']
            pred_logits = [outputs['pred_logits'][b]]
            pred_boxes = [outputs['pred_boxes'][b]]
            for xx in outputs['aux_outputs']:
                pred_logits.append(xx['pred_logits'][b])
                pred_boxes.append(xx['pred_boxes'][b])
            pred_logits = torch.stack(pred_logits, dim=0) # L x N x C
            pred_boxes = torch.stack(pred_boxes, dim=0) # L x N x 4
            for label in labels:
                loss += self._max_size_loss(
                    pred_logits, pred_boxes, label) / len(labels)
        loss = loss / len(batched_inputs)
        return loss


    def _max_size_loss(self, logits, boxes, label):
        '''
        Inputs:
          logits: L x N x C
          boxes: L x N x 4
        '''
        target = logits.new_zeros((logits.shape[0], logits.shape[2]))
        target[:, label] = 1.
        sizes = boxes[..., 2] * boxes[..., 3] # L x N
        ind = sizes.argmax(dim=1) # L
        loss = F.binary_cross_entropy_with_logits(
            logits[range(len(ind)), ind], target, reduction='sum')
        return loss