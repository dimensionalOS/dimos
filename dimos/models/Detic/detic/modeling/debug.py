# Copyright (c) Facebook, Inc. and its affiliates.
import cv2
import numpy as np
import torch
import torch.nn.functional as F
import os

COLORS = ((np.random.rand(1300, 3) * 0.4 + 0.6) * 255).astype(
  np.uint8).reshape(1300, 1, 1, 3)

def _get_color_image(heatmap):
  heatmap = heatmap.reshape(
    heatmap.shape[0], heatmap.shape[1], heatmap.shape[2], 1)
  if heatmap.shape[0] == 1:
      color_map = (heatmap * np.ones((1, 1, 1, 3), np.uint8) * 255).max(
          axis=0).astype(np.uint8) # H, W, 3
  else:
      color_map = (heatmap * COLORS[:heatmap.shape[0]]).max(axis=0).astype(np.uint8) # H, W, 3

  return color_map

def _blend_image(image, color_map, a=0.7):
  color_map = cv2.resize(color_map, (image.shape[1], image.shape[0]))
  ret = np.clip(image * (1 - a) + color_map * a, 0, 255).astype(np.uint8)
  return ret

def _blend_image_heatmaps(image, color_maps, a=0.7):
    merges = np.zeros((image.shape[0], image.shape[1], 3), np.float32)
    for color_map in color_maps:
        color_map = cv2.resize(color_map, (image.shape[1], image.shape[0]))
        merges = np.maximum(merges, color_map)
    ret = np.clip(image * (1 - a) + merges * a, 0, 255).astype(np.uint8)
    return ret

def _decompose_level(x, shapes_per_level, N):
    '''
    x: LNHiWi x C
    '''
    x = x.view(x.shape[0], -1)
    ret = []
    st = 0
    for l in range(len(shapes_per_level)):
        ret.append([])
        h = shapes_per_level[l][0].int().item()
        w = shapes_per_level[l][1].int().item()
        for i in range(N):
            ret[l].append(x[st + h * w * i:st + h * w * (i + 1)].view(
                h, w, -1).permute(2, 0, 1))
        st += h * w * N
    return ret

def _imagelist_to_tensor(images):
    images = [x for x in images]
    image_sizes = [x.shape[-2:] for x in images]
    h = max([size[0] for size in image_sizes])
    w = max([size[1] for size in image_sizes])
    S = 32
    h, w = ((h - 1) // S + 1) * S, ((w - 1) // S + 1) * S
    images = [F.pad(x, (0, w - x.shape[2], 0, h - x.shape[1], 0, 0)) \
        for x in images]
    images = torch.stack(images)
    return images


def _ind2il(ind, shapes_per_level, N):
    r = ind
    l = 0
    S = 0
    while r - S >= N * shapes_per_level[l][0] * shapes_per_level[l][1]:
        S += N * shapes_per_level[l][0] * shapes_per_level[l][1]
        l += 1
    i = (r - S) // (shapes_per_level[l][0] * shapes_per_level[l][1])
    return i, l

def debug_train(
    images, gt_instances, flattened_hms, reg_targets, labels, pos_inds,
    shapes_per_level, locations, strides):
    '''
    images: N x 3 x H x W
    flattened_hms: LNHiWi x C
    shapes_per_level: L x 2 [(H_i, W_i)]
    locations: LNHiWi x 2
    '''
    reg_inds = torch.nonzero(
        reg_targets.max(dim=1)[0] > 0).squeeze(1)
    N = len(images)
    images = _imagelist_to_tensor(images)
    repeated_locations = [torch.cat([loc] * N, dim=0) \
        for loc in locations]
    locations = torch.cat(repeated_locations, dim=0)
    gt_hms = _decompose_level(flattened_hms, shapes_per_level, N)
    masks = flattened_hms.new_zeros((flattened_hms.shape[0], 1))
    masks[pos_inds] = 1
    masks = _decompose_level(masks, shapes_per_level, N)
    for i in range(len(images)):
        image = images[i].detach().cpu().numpy().transpose(1, 2, 0)
        color_maps = []
        for l in range(len(gt_hms)):
            color_map = _get_color_image(
                gt_hms[l][i].detach().cpu().numpy())
            color_maps.append(color_map)
            cv2.imshow('gthm_{}'.format(l), color_map)
        blend = _blend_image_heatmaps(image.copy(), color_maps)
        if gt_instances is not None:
            bboxes = gt_instances[i].gt_boxes.tensor
            for j in range(len(bboxes)):
                bbox = bboxes[j]
                cv2.rectangle(
                    blend, 
                    (int(bbox[0]), int(bbox[1])),
                    (int(bbox[2]), int(bbox[3])),
                    (0, 0, 255), 3, cv2.LINE_AA)
    
        for j in range(len(pos_inds)):
            image_id, l = _ind2il(pos_inds[j], shapes_per_level, N)
            if image_id != i:
                continue
            loc = locations[pos_inds[j]]
            cv2.drawMarker(
                blend, (int(loc[0]), int(loc[1])), (0, 255, 255),
                markerSize=(l + 1) * 16)
        
        for j in range(len(reg_inds)):
            image_id, l = _ind2il(reg_inds[j], shapes_per_level, N)
            if image_id != i:
                continue
            ltrb = reg_targets[reg_inds[j]]
            ltrb *= strides[l]
            loc = locations[reg_inds[j]]
            bbox = [(loc[0] - ltrb[0]), (loc[1] - ltrb[1]),
                    (loc[0] + ltrb[2]), (loc[1] + ltrb[3])]
            cv2.rectangle(
                blend, 
                (int(bbox[0]), int(bbox[1])),
                (int(bbox[2]), int(bbox[3])),
                (255, 0, 0), 1, cv2.LINE_AA)  
            cv2.circle(blend, (int(loc[0]), int(loc[1])), 2, (255, 0, 0), -1)

        cv2.imshow('blend', blend)
        cv2.waitKey()


def debug_test(
    images, logits_pred, reg_pred, agn_hm_pred=[], preds=[], 
    vis_thresh=0.3, debug_show_name=False, mult_agn=False):
    '''
    images: N x 3 x H x W
    class_target: LNHiWi x C
    cat_agn_heatmap: LNHiWi
    shapes_per_level: L x 2 [(H_i, W_i)]
    '''
    N = len(images)
    for i in range(len(images)):
        image = images[i].detach().cpu().numpy().transpose(1, 2, 0)
        result = image.copy().astype(np.uint8)
        pred_image = image.copy().astype(np.uint8)
        color_maps = []
        L = len(logits_pred)
        for l in range(L):
            if logits_pred[0] is not None:
                stride = min(image.shape[0], image.shape[1]) / min(
                    logits_pred[l][i].shape[1], logits_pred[l][i].shape[2])
            else:
                stride = min(image.shape[0], image.shape[1]) / min(
                    agn_hm_pred[l][i].shape[1], agn_hm_pred[l][i].shape[2])
            stride = stride if stride < 60 else 64 if stride < 100 else 128
            if logits_pred[0] is not None:
                if mult_agn:
                    logits_pred[l][i] = logits_pred[l][i] * agn_hm_pred[l][i]
                color_map = _get_color_image(
                    logits_pred[l][i].detach().cpu().numpy())
                color_maps.append(color_map)
                cv2.imshow('predhm_{}'.format(l), color_map)

            if debug_show_name:
                from detectron2.data.datasets.lvis_v1_categories import LVIS_CATEGORIES 
                cat2name = [x['name'] for x in LVIS_CATEGORIES]
            for j in range(len(preds[i].scores) if preds is not None else 0):
                if preds[i].scores[j] > vis_thresh:
                    bbox = preds[i].proposal_boxes[j] \
                        if preds[i].has('proposal_boxes') else \
                        preds[i].pred_boxes[j]
                    bbox = bbox.tensor[0].detach().cpu().numpy().astype(np.int32)
                    cat = int(preds[i].pred_classes[j]) \
                        if preds[i].has('pred_classes') else 0
                    cl = COLORS[cat, 0, 0]
                    cv2.rectangle(
                        pred_image, (int(bbox[0]), int(bbox[1])), 
                        (int(bbox[2]), int(bbox[3])), 
                        (int(cl[0]), int(cl[1]), int(cl[2])), 2, cv2.LINE_AA)
                    if debug_show_name:
                        txt = '{}{:.1f}'.format(
                            cat2name[cat] if cat > 0 else '', 
                            preds[i].scores[j])
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cat_size = cv2.getTextSize(txt, font, 0.5, 2)[0]
                        cv2.rectangle(
                            pred_image,
                            (int(bbox[0]), int(bbox[1] - cat_size[1] - 2)),
                            (int(bbox[0] + cat_size[0]), int(bbox[1] - 2)), 
                            (int(cl[0]), int(cl[1]), int(cl[2])), -1)
                        cv2.putText(
                            pred_image, txt, (int(bbox[0]), int(bbox[1] - 2)), 
                            font, 0.5, (0, 0, 0), thickness=1, lineType=cv2.LINE_AA)


            if agn_hm_pred[l] is not None:
                agn_hm_ = agn_hm_pred[l][i, 0, :, :, None].detach().cpu().numpy()
                agn_hm_ = (agn_hm_ * np.array([255, 255, 255]).reshape(
                    1, 1, 3)).astype(np.uint8)
                cv2.imshow('agn_hm_{}'.format(l), agn_hm_)
        blend = _blend_image_heatmaps(image.copy(), color_maps)
        cv2.imshow('blend', blend)
        cv2.imshow('preds', pred_image)
        cv2.waitKey()

global cnt
cnt = 0

def debug_second_stage(images, instances, proposals=None, vis_thresh=0.3, 
    save_debug=False, debug_show_name=False, image_labels=[],
    save_debug_path='output/save_debug/',
    bgr=False):
    images = _imagelist_to_tensor(images)
    if 'COCO' in save_debug_path:
        from detectron2.data.datasets.builtin_meta import COCO_CATEGORIES
        cat2name = [x['name'] for x in COCO_CATEGORIES]
    else:
        from detectron2.data.datasets.lvis_v1_categories import LVIS_CATEGORIES
        cat2name = ['({}){}'.format(x['frequency'], x['name']) \
            for x in LVIS_CATEGORIES]
    for i in range(len(images)):
        image = images[i].detach().cpu().numpy().transpose(1, 2, 0).astype(np.uint8).copy()
        if bgr:
            image = image[:, :, ::-1].copy()
        if instances[i].has('gt_boxes'):
            bboxes = instances[i].gt_boxes.tensor.cpu().numpy()
            scores = np.ones(bboxes.shape[0])
            cats = instances[i].gt_classes.cpu().numpy()
        else:
            bboxes = instances[i].pred_boxes.tensor.cpu().numpy()
            scores = instances[i].scores.cpu().numpy()
            cats = instances[i].pred_classes.cpu().numpy()
        for j in range(len(bboxes)):
            if scores[j] > vis_thresh:
                bbox = bboxes[j]
                cl = COLORS[cats[j], 0, 0]
                cl = (int(cl[0]), int(cl[1]), int(cl[2]))
                cv2.rectangle(
                    image, 
                    (int(bbox[0]), int(bbox[1])),
                    (int(bbox[2]), int(bbox[3])),
                    cl, 2, cv2.LINE_AA)
                if debug_show_name:
                    cat = cats[j]
                    txt = '{}{:.1f}'.format(
                        cat2name[cat] if cat > 0 else '', 
                        scores[j])
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cat_size = cv2.getTextSize(txt, font, 0.5, 2)[0]
                    cv2.rectangle(
                        image,
                        (int(bbox[0]), int(bbox[1] - cat_size[1] - 2)),
                        (int(bbox[0] + cat_size[0]), int(bbox[1] - 2)), 
                        (int(cl[0]), int(cl[1]), int(cl[2])), -1)
                    cv2.putText(
                        image, txt, (int(bbox[0]), int(bbox[1] - 2)), 
                        font, 0.5, (0, 0, 0), thickness=1, lineType=cv2.LINE_AA)
        if proposals is not None:
            proposal_image = images[i].detach().cpu().numpy().transpose(1, 2, 0).astype(np.uint8).copy()
            if bgr:
                proposal_image = proposal_image.copy()
            else:
                proposal_image = proposal_image[:, :, ::-1].copy()
            bboxes = proposals[i].proposal_boxes.tensor.cpu().numpy()
            if proposals[i].has('scores'):
                scores = proposals[i].scores.detach().cpu().numpy()
            else:
                scores = proposals[i].objectness_logits.detach().cpu().numpy()
            # selected = -1
            # if proposals[i].has('image_loss'):
            #     selected = proposals[i].image_loss.argmin()
            if proposals[i].has('selected'):
                selected = proposals[i].selected
            else:
                selected = [-1 for _ in range(len(bboxes))]
            for j in range(len(bboxes)):
                if scores[j] > vis_thresh or selected[j] >= 0:
                    bbox = bboxes[j]
                    cl = (209, 159, 83)
                    th = 2
                    if selected[j] >= 0:
                        cl = (0, 0, 0xa4)
                        th = 4
                    cv2.rectangle(
                        proposal_image, 
                        (int(bbox[0]), int(bbox[1])),
                        (int(bbox[2]), int(bbox[3])),
                        cl, th, cv2.LINE_AA)
                    if selected[j] >= 0 and debug_show_name:
                        cat = selected[j].item()
                        txt = '{}'.format(cat2name[cat])
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        cat_size = cv2.getTextSize(txt, font, 0.5, 2)[0]
                        cv2.rectangle(
                            proposal_image,
                            (int(bbox[0]), int(bbox[1] - cat_size[1] - 2)),
                            (int(bbox[0] + cat_size[0]), int(bbox[1] - 2)), 
                            (int(cl[0]), int(cl[1]), int(cl[2])), -1)
                        cv2.putText(
                            proposal_image, txt, 
                            (int(bbox[0]), int(bbox[1] - 2)), 
                            font, 0.5, (0, 0, 0), thickness=1, 
                            lineType=cv2.LINE_AA)

        if save_debug:
            global cnt
            cnt = (cnt + 1) % 5000
            if not os.path.exists(save_debug_path):
                os.mkdir(save_debug_path)
            save_name = '{}/{:05d}.jpg'.format(save_debug_path, cnt)
            if i < len(image_labels):
                image_label = image_labels[i]
                save_name = '{}/{:05d}'.format(save_debug_path, cnt)
                for x in image_label:
                    class_name = cat2name[x]
                    save_name = save_name + '|{}'.format(class_name)
                save_name = save_name + '.jpg'
            cv2.imwrite(save_name, proposal_image)
        else:
            cv2.imshow('image', image)
            if proposals is not None:
                cv2.imshow('proposals', proposal_image)
            cv2.waitKey()