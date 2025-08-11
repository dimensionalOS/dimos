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

import onnxruntime as ort
from PIL import Image
import cv2
import numpy as np

# May need to add this back for import to work
# external_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'external', 'Metric3D'))
# if external_path not in sys.path:
#     sys.path.append(external_path)


class Metric3D:
    def __init__(self, onnx_model_path, gt_depth_scale=256.0, intrinsic=None, provider='auto'):
        self.input_size = (616, 1064)  # for vit model; adjust if needed
        self.gt_depth_scale = gt_depth_scale
        self.intrinsic = intrinsic or [707.0493, 707.0493, 604.0814, 180.5066]
        self.intrinsic_scaled = None
        self.pad_info = None
        self.rgb_origin = None

        # Provider selection logic
        if provider == 'auto':
            # Try CUDA, then TensorRT, then CPU
            available_providers = ort.get_available_providers()
            if 'CUDAExecutionProvider' in available_providers:
                providers = [
                    (
                        'CUDAExecutionProvider',
                        {'cudnn_conv_use_max_workspace': '0', 'device_id': '0'}
                    )
                ]
            elif 'TensorrtExecutionProvider' in available_providers:
                providers = [
                    (
                        'TensorrtExecutionProvider',
                        {'trt_engine_cache_enable': True, 'trt_fp16_enable': True, 'device_id': 0, 'trt_dla_enable': False}
                    )
                ]
            else:
                providers = ['CPUExecutionProvider']
        elif provider == 'cuda':
            providers = [
                (
                    'CUDAExecutionProvider',
                    {'cudnn_conv_use_max_workspace': '0', 'device_id': '0'}
                )
            ]
        elif provider == 'tensorrt':
            providers = [
                (
                    'TensorrtExecutionProvider',
                    {'trt_engine_cache_enable': True, 'trt_fp16_enable': True, 'device_id': 0, 'trt_dla_enable': False}
                )
            ]
        else:
            providers = ['CPUExecutionProvider']

        self.session = ort.InferenceSession(onnx_model_path, providers=providers)

    """
    Input: Single image in RGB format
    Output: Depth map
    """

    def update_intrinsic(self, intrinsic):
        """
        Update the intrinsic parameters dynamically.
        Ensure that the input intrinsic is valid.
        """
        if len(intrinsic) != 4:
            raise ValueError("Intrinsic must be a list or tuple with 4 values: [fx, fy, cx, cy]")
        self.intrinsic = intrinsic
        print(f"Intrinsics updated to: {self.intrinsic}")

    def prepare_input(self, rgb_image):
        h, w = rgb_image.shape[:2]
        scale = min(self.input_size[0] / h, self.input_size[1] / w)
        rgb = cv2.resize(
            rgb_image, (int(w * scale), int(h * scale)), interpolation=cv2.INTER_LINEAR
        )
        # Scale intrinsics
        self.intrinsic_scaled = [
            self.intrinsic[0] * scale,
            self.intrinsic[1] * scale,
            self.intrinsic[2] * scale,
            self.intrinsic[3] * scale,
        ]
        # padding to input_size
        padding = [123.675, 116.28, 103.53]
        h, w = rgb.shape[:2]
        pad_h = self.input_size[0] - h
        pad_w = self.input_size[1] - w
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
        self.pad_info = [pad_h_half, pad_h - pad_h_half, pad_w_half, pad_w - pad_w_half]
        onnx_input = {
            "image": np.ascontiguousarray(
                np.transpose(rgb, (2, 0, 1))[None], dtype=np.float32
            ),  # 1, 3, H, W
        }
        return onnx_input, rgb_image.shape[:2]

    def infer_depth(self, img, debug=False):
        if debug:
            print(f"Input image: {img}")
        try:
            if isinstance(img, str):
                print(f"Image type string: {type(img)}")
                self.rgb_origin = cv2.imread(img)[:, :, ::-1]
            else:
                self.rgb_origin = img
        except Exception as e:
            print(f"Error parsing into infer_depth: {e}")
            return None

        onnx_input, original_shape = self.prepare_input(self.rgb_origin)
        outputs = self.session.run(None, onnx_input)
        depth = outputs[0].squeeze()  # [H, W]

        # Remove padding
        pad_info = self.pad_info
        depth = depth[
            pad_info[0] : self.input_size[0] - pad_info[1],
            pad_info[2] : self.input_size[1] - pad_info[3],
        ]
        # Resize to original image size
        depth = cv2.resize(
            depth, (original_shape[1], original_shape[0]), interpolation=cv2.INTER_LINEAR
        )

        # Convert canonical depth to metric using scaled intrinsics
        if self.intrinsic_scaled is not None:
            canonical_to_real_scale = self.intrinsic_scaled[0] / 1000.0
            depth = depth * canonical_to_real_scale

        # Convert to 16-bit and PIL Image
        out_16bit_numpy = (depth * self.gt_depth_scale).astype(np.uint16)
        depth_map_pil = Image.fromarray(out_16bit_numpy)
        return depth_map_pil

    def save_depth(self, pred_depth):
        # Save the depth map to a file
        if isinstance(pred_depth, Image.Image):
            pred_depth_np = np.array(pred_depth)
        else:
            pred_depth_np = pred_depth
        output_depth_file = "output_depth_map.png"
        cv2.imwrite(output_depth_file, pred_depth_np)
        print(f"Depth map saved to {output_depth_file}")

    def eval_predicted_depth(self, depth_file, pred_depth):
        if depth_file is not None:
            gt_depth = cv2.imread(depth_file, -1)
            gt_depth = gt_depth / self.gt_depth_scale
            if isinstance(pred_depth, Image.Image):
                pred_depth = np.array(pred_depth) / self.gt_depth_scale
            else:
                pred_depth = pred_depth / self.gt_depth_scale
            assert gt_depth.shape == pred_depth.shape
            mask = gt_depth > 1e-8
            abs_rel_err = (np.abs(pred_depth[mask] - gt_depth[mask]) / gt_depth[mask]).mean()
            print("abs_rel_err:", abs_rel_err)
