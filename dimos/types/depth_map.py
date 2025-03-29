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

from typing import Any
import numpy as np

class DepthMapType:
    def __init__(self, depth_data: np.ndarray, metadata: Any = None):
        """
        Initializes a standardized depth map type.

        Args:
            depth_data (np.ndarray): The depth map data as a numpy array.
            metadata (Any, optional): Additional metadata related to the depth map.
        """
        self.depth_data = depth_data
        self.metadata = metadata

    def normalize(self):
        """Normalize the depth data to a 0-1 range."""
        min_val = np.min(self.depth_data)
        max_val = np.max(self.depth_data)
        self.depth_data = (self.depth_data - min_val) / (max_val - min_val)

    def save_to_file(self, filepath: str):
        """Save the depth map to a file."""
        np.save(filepath, self.depth_data) 