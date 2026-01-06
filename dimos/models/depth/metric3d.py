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

from dataclasses import dataclass, field
from functools import cached_property
from typing import Any

import cv2
import torch

from dimos.models.base import LocalModel, LocalModelConfig


@dataclass
class Metric3DConfig(LocalModelConfig):
    """Configuration for Metric3D depth estimation model."""

    camera_intrinsics: list[float] = field(default_factory=lambda: [500.0, 500.0, 320.0, 240.0])
    """Camera intrinsics [fx, fy, cx, cy]."""

    gt_depth_scale: float = 256.0
    """Scale factor for ground truth depth."""

    device: str = "cuda" if torch.cuda.is_available() else "cpu"
    """Device to run the model on."""


class Metric3D(LocalModel):
    default_config = Metric3DConfig
    config: Metric3DConfig

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        self.intrinsic = self.config.camera_intrinsics
        self.intrinsic_scaled: list[float] | None = None
        self.gt_depth_scale = self.config.gt_depth_scale
        self.pad_info: list[int] | None = None
        self.rgb_origin: Any = None

    @cached_property
    def _model(self) -> Any:
        model = torch.hub.load(  # type: ignore[no-untyped-call]
            "yvanyin/metric3d", "metric3d_vit_small", pretrain=True
        )
        model = model.to(self.device)
        model.eval()
        return model

    """
    Input: Single image in RGB format
    Output: Depth map
    """

    def update_intrinsic(self, intrinsic):  # type: ignore[no-untyped-def]
        """
        Update the intrinsic parameters dynamically.
        Ensure that the input intrinsic is valid.
        """
        if len(intrinsic) != 4:
            raise ValueError("Intrinsic must be a list or tuple with 4 values: [fx, fy, cx, cy]")
        self.intrinsic = intrinsic
        print(f"Intrinsics updated to: {self.intrinsic}")

    def infer_depth(self, img, debug: bool = False):  # type: ignore[no-untyped-def]
        if debug:
            print(f"Input image: {img}")
        try:
            if isinstance(img, str):
                print(f"Image type string: {type(img)}")
                self.rgb_origin = cv2.imread(img)[:, :, ::-1]  # type: ignore[assignment]
            else:
                # print(f"Image type not string: {type(img)}, cv2 conversion assumed to be handled. If not, this will throw an error")
                self.rgb_origin = img
        except Exception as e:
            print(f"Error parsing into infer_depth: {e}")

        img = self.rescale_input(img, self.rgb_origin)  # type: ignore[no-untyped-call]

        with torch.no_grad():
            pred_depth, confidence, output_dict = self._model.inference({"input": img})

        # Convert to PIL format
        depth_image = self.unpad_transform_depth(pred_depth)  # type: ignore[no-untyped-call]

        return depth_image.cpu().numpy()

    def save_depth(self, pred_depth) -> None:  # type: ignore[no-untyped-def]
        # Save the depth map to a file
        pred_depth_np = pred_depth.cpu().numpy()
        output_depth_file = "output_depth_map.png"
        cv2.imwrite(output_depth_file, pred_depth_np)
        print(f"Depth map saved to {output_depth_file}")

    # Adjusts input size to fit pretrained ViT model
    def rescale_input(self, rgb, rgb_origin):  # type: ignore[no-untyped-def]
        #### ajust input size to fit pretrained model
        # keep ratio resize
        input_size = (616, 1064)  # for vit model
        # input_size = (544, 1216) # for convnext model
        h, w = rgb_origin.shape[:2]
        scale = min(input_size[0] / h, input_size[1] / w)
        rgb = cv2.resize(
            rgb_origin, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR
        )
        # remember to scale intrinsic, hold depth
        self.intrinsic_scaled = [  # type: ignore[assignment]
            self.intrinsic[0] * scale,
            self.intrinsic[1] * scale,
            self.intrinsic[2] * scale,
            self.intrinsic[3] * scale,
        ]
        # padding to input_size
        padding = [123.675, 116.28, 103.53]
        h, w = rgb.shape[:2]
        pad_h = input_size[0] - h
        pad_w = input_size[1] - w
        pad_h_half = pad_h // 2
        pad_w_half = pad_w // 2
        rgb = cv2.copyMakeBorder(
            rgb,
            pad_h_half,
            pad_h - pad_h_half,
            pad_w_half,
            pad_w - pad_w_half,
            cv2.BORDER_CONSTANT,
            value=padding,
        )
        self.pad_info = [pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half]  # type: ignore[assignment]

        #### normalize
        mean = torch.tensor([123.675, 116.28, 103.53]).float()[:, None, None]
        std = torch.tensor([58.395, 57.12, 57.375]).float()[:, None, None]
        rgb = torch.from_numpy(rgb.transpose((2, 0, 1))).float()
        rgb = torch.div((rgb - mean), std)
        rgb = rgb[None, :, :, :].to(self.device)
        return rgb

    def unpad_transform_depth(self, pred_depth):  # type: ignore[no-untyped-def]
        # un pad
        pred_depth = pred_depth.squeeze()
        pred_depth = pred_depth[
            self.pad_info[0] : pred_depth.shape[0] - self.pad_info[1],  # type: ignore[index]
            self.pad_info[2] : pred_depth.shape[1] - self.pad_info[3],  # type: ignore[index]
        ]

        # upsample to original size
        pred_depth = torch.nn.functional.interpolate(
            pred_depth[None, None, :, :],
            self.rgb_origin.shape[:2],
            mode="bilinear",  # type: ignore[attr-defined]
        ).squeeze()
        ###################### canonical camera space ######################

        #### de-canonical transform
        canonical_to_real_scale = (
            self.intrinsic_scaled[0] / 1000.0  # type: ignore[index]
        )  # 1000.0 is the focal length of canonical camera
        pred_depth = pred_depth * canonical_to_real_scale  # now the depth is metric
        pred_depth = torch.clamp(pred_depth, 0, 1000)
        return pred_depth

    def eval_predicted_depth(self, depth_file, pred_depth) -> None:  # type: ignore[no-untyped-def]
        if depth_file is not None:
            gt_depth = cv2.imread(depth_file, -1)
            gt_depth = gt_depth / self.gt_depth_scale
            gt_depth = torch.from_numpy(gt_depth).float().to(self.device)  # type: ignore[assignment]
            assert gt_depth.shape == pred_depth.shape

            mask = gt_depth > 1e-8
            abs_rel_err = (torch.abs(pred_depth[mask] - gt_depth[mask]) / gt_depth[mask]).mean()
            print("abs_rel_err:", abs_rel_err.item())
