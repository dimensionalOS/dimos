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

import os
import sys
import torch
from PIL import Image
import cv2
import numpy as np

# May need to add this back for import to work
# external_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'external', 'Metric3D'))
# if external_path not in sys.path:
#     sys.path.append(external_path)


class Metric3D:
    def __init__(self, gt_depth_scale=256.0):
        #self.conf = get_config("zoedepth", "infer")
        #self.depth_model = build_model(self.conf)
        self.depth_model = torch.hub.load('yvanyin/metric3d', 'metric3d_vit_small', pretrain=True).cuda()
        if torch.cuda.device_count() > 1:
            print(f"Using {torch.cuda.device_count()} GPUs!")
            #self.depth_model = torch.nn.DataParallel(self.depth_model)
        self.depth_model.eval()

        self.intrinsic = [707.0493, 707.0493, 604.0814, 180.5066]  
        self.intrinsic_scaled = None
        self.gt_depth_scale = gt_depth_scale # And this
        self.pad_info = None
        self.rgb_origin = None
    '''
    Input: Single image in RGB format
    Output: Depth map
    '''

    def update_intrinsic(self, intrinsic):
        """
        Update the intrinsic parameters dynamically.
        Ensure that the input intrinsic is valid.
        """
        if len(intrinsic) != 4:
            raise ValueError("Intrinsic must be a list or tuple with 4 values: [fx, fy, cx, cy]")
        self.intrinsic = intrinsic
        print(f"Intrinsics updated to: {self.intrinsic}")

    def infer_depth(self, img, debug=False):
        if debug:
            print(f"Input image: {img}")
        try:
            if isinstance(img, str):
                print(f"Image type string: {type(img)}")
                self.rgb_origin = cv2.imread(img)[:, :, ::-1]
            else:
                # print(f"Image type not string: {type(img)}, cv2 conversion assumed to be handled. If not, this will throw an error")
                self.rgb_origin = img
        except Exception as e:
            print(f"Error parsing into infer_depth: {e}")

        img = self.rescale_input(img, self.rgb_origin)

        with torch.no_grad():
            pred_depth, confidence, output_dict = self.depth_model.inference({'input': img})

        # Convert to PIL format
        depth_image = self.unpad_transform_depth(pred_depth)
        out_16bit_numpy = (depth_image.squeeze().cpu().numpy() * self.gt_depth_scale).astype(np.uint16)
        depth_map_pil = Image.fromarray(out_16bit_numpy)

        return depth_map_pil
    def save_depth(self, pred_depth):
        # Save the depth map to a file
        pred_depth_np = pred_depth.cpu().numpy()
        output_depth_file = 'output_depth_map.png'
        cv2.imwrite(output_depth_file, pred_depth_np)
        print(f"Depth map saved to {output_depth_file}")

    # Adjusts input size to fit pretrained ViT model
    def rescale_input(self, rgb, rgb_origin):
        #### ajust input size to fit pretrained model
        # keep ratio resize
        input_size = (616, 1064)  # for vit model
        # input_size = (544, 1216) # for convnext model
        h, w = rgb_origin.shape[:2]
        scale = min(input_size[0] / h, input_size[1] / w)
        rgb = cv2.resize(rgb_origin, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR)
        # remember to scale intrinsic, hold depth
        self.intrinsic_scaled = [self.intrinsic[0] * scale, self.intrinsic[1] * scale, self.intrinsic[2] * scale, self.intrinsic[3] * scale]
        # padding to input_size
        padding = [123.675, 116.28, 103.53]
        h, w = rgb.shape[:2]
        pad_h = input_size[0] - h
        pad_w = input_size[1] - w
        pad_h_half = pad_h // 2
        pad_w_half = pad_w // 2
        rgb = cv2.copyMakeBorder(rgb, pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half,
                                 cv2.BORDER_CONSTANT, value=padding)
        self.pad_info = [pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half]

        #### normalize
        mean = torch.tensor([123.675, 116.28, 103.53]).float()[:, None, None]
        std = torch.tensor([58.395, 57.12, 57.375]).float()[:, None, None]
        rgb = torch.from_numpy(rgb.transpose((2, 0, 1))).float()
        rgb = torch.div((rgb - mean), std)
        rgb = rgb[None, :, :, :].cuda()
        return rgb
    def unpad_transform_depth(self, pred_depth):
        # un pad
        pred_depth = pred_depth.squeeze()
        pred_depth = pred_depth[self.pad_info[0]: pred_depth.shape[0] - self.pad_info[1],
                     self.pad_info[2]: pred_depth.shape[1] - self.pad_info[3]]

        # upsample to original size
        pred_depth = torch.nn.functional.interpolate(pred_depth[None, None, :, :], self.rgb_origin.shape[:2],
                                                     mode='bilinear').squeeze()
        ###################### canonical camera space ######################

        #### de-canonical transform
        canonical_to_real_scale = self.intrinsic_scaled[0] / 1000.0  # 1000.0 is the focal length of canonical camera
        pred_depth = pred_depth * canonical_to_real_scale  # now the depth is metric
        pred_depth = torch.clamp(pred_depth, 0, 1000)
        return pred_depth


    """Set new intrinsic value."""
    def update_intrinsic(self, intrinsic):
        self.intrinsic = intrinsic

    def eval_predicted_depth(self, depth_file, pred_depth):
        if depth_file is not None:
            gt_depth = cv2.imread(depth_file, -1)
            gt_depth = gt_depth / self.gt_depth_scale
            gt_depth = torch.from_numpy(gt_depth).float().cuda()
            assert gt_depth.shape == pred_depth.shape

            mask = (gt_depth > 1e-8)
            abs_rel_err = (torch.abs(pred_depth[mask] - gt_depth[mask]) / gt_depth[mask]).mean()
            print('abs_rel_err:', abs_rel_err.item())