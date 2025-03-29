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

from typing import List, Any
import numpy as np

class SegmentationType:
    def __init__(self, masks: List[np.ndarray], metadata: Any = None):
        """
        Initializes a standardized segmentation type.

        Args:
            masks (List[np.ndarray]): A list of binary masks for segmentation.
            metadata (Any, optional): Additional metadata related to the segmentations.
        """
        self.masks = masks
        self.metadata = metadata 

    def combine_masks(self):
        """Combine all masks into a single mask."""
        combined_mask = np.zeros_like(self.masks[0])
        for mask in self.masks:
            combined_mask = np.logical_or(combined_mask, mask)
        return combined_mask

    def save_masks(self, directory: str):
        """Save each mask to a separate file."""
        import os
        os.makedirs(directory, exist_ok=True)
        for i, mask in enumerate(self.masks):
            np.save(os.path.join(directory, f"mask_{i}.npy"), mask) 