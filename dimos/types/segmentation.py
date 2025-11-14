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