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

from dimos.models.depth.metric3d import Metric3D
import os
import pickle
import argparse
import pandas as pd
from PIL import Image
from io import BytesIO
import torch
import sys
import cv2
import tarfile
import logging
import time
import tempfile
import gc
import io
import csv
import numpy as np
from dimos.types.depth_map import DepthMapType

class DepthProcessor:
    def __init__(self, debug=False):
        self.debug = debug
        self.metric_3d = Metric3D()
        self.depth_count = 0
        self.valid_depth_count = 0
        self.logger = logging.getLogger(__name__)
        self.intrinsic = [707.0493, 707.0493, 604.0814, 180.5066]  # Default intrinsic

        print("DepthProcessor initialized")

        if debug:
            print("Running in debug mode")
            self.logger.info("Running in debug mode")


    def process(self, frame: Image.Image, intrinsics=None):
        """Process a frame to generate a depth map.
        
        Args:
            frame: PIL Image to process
            intrinsics: Optional camera intrinsics parameters
        
        Returns:
            DepthMapType containing the depth map
        """
        if intrinsics:
            self.metric_3d.update_intrinsic(intrinsics)
        else:
            self.metric_3d.update_intrinsic(self.intrinsic)

        # Convert frame to numpy array suitable for processing
        if isinstance(frame, Image.Image):
            image = frame.convert('RGB')
        elif isinstance(frame, np.ndarray):
            image = Image.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        else:
            raise ValueError("Unsupported frame format. Must be PIL Image or numpy array.")

        image_np = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        image_np = resize_image_for_vit(image_np)

        # Process image and run depth via Metric3D
        try:
            with torch.no_grad():
                depth_map = self.metric_3d.infer_depth(image_np)

            self.depth_count += 1

            # Validate depth map
            if is_depth_map_valid(np.array(depth_map)):
                self.valid_depth_count += 1
            else:
                self.logger.error(f"Invalid depth map for the provided frame.")
                print("Invalid depth map for the provided frame.")
                return None

            if self.debug:
                # Save depth map locally or to S3 as needed
                pass  # Implement saving logic if required

            return DepthMapType(depth_data=depth_map, metadata={"intrinsics": intrinsics})

        except Exception as e:
            self.logger.error(f"Error processing frame: {e}")
            return None