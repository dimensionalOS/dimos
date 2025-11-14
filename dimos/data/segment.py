import cv2
import numpy as np
from PIL import Image
import logging
from dimos.models.segmentation.segment_utils import sample_points_from_heatmap
from dimos.models.segmentation.sam import SAM
from dimos.models.segmentation.clipseg import CLIPSeg

# Setup logging
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


class SegmentProcessor:
    def __init__(self, device='cuda'):
        # Initialize CLIPSeg and SAM models
        self.clipseg = CLIPSeg(model_name="CIDAS/clipseg-rd64-refined", device=device)
        self.sam = SAM(model_name="facebook/sam-vit-huge", device=device)
        self.logger = logger

    def process_frame(self, image, captions):
        """
        Process a single image and return segmentation masks.

        Args:
            image (PIL.Image.Image or np.ndarray): The input image to process.
            captions (list of str): A list of captions for segmentation.

        Returns:
            list of np.ndarray: A list of segmentation masks corresponding to the captions.
        """
        try:
            self.logger.info("STARTING PROCESSING IMAGE ---------------------------------------")
            self.logger.info(f"Processing image with captions: {captions}")

            # Convert image to PIL.Image if it's a numpy array
            if isinstance(image, np.ndarray):
                image = Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))

            preds = self.clipseg.run_inference(image, captions)
            sampled_points = []
            sam_masks = []

            original_size = image.size  # (width, height)

            for idx in range(preds.shape[0]):
                points = sample_points_from_heatmap(preds[idx][0], original_size, num_points=10)
                if points:
                    sampled_points.append(points)
                else:
                    self.logger.info(f"No points sampled for prediction index {idx}")
                    sampled_points.append([])

            for idx in range(preds.shape[0]):
                if sampled_points[idx]:
                    mask_tensor = self.sam.run_inference_from_points(image, [sampled_points[idx]])
                    if mask_tensor:
                        # Convert mask tensor to a numpy array
                        mask = (255 * mask_tensor[0].numpy().squeeze()).astype(np.uint8)
                        sam_masks.append(mask)
                    else:
                        self.logger.info(f"No mask tensor returned for sampled points at index {idx}")
                        sam_masks.append(np.zeros((original_size[1], original_size[0]), dtype=np.uint8))
                else:
                    self.logger.info(f"No sampled points for prediction index {idx}, skipping mask inference")
                    sam_masks.append(np.zeros((original_size[1], original_size[0]), dtype=np.uint8))

            self.logger.info("DONE PROCESSING IMAGE ---------------------------------------")
            return sam_masks
        except Exception as e:
            self.logger.error(f"Error processing image: {e}")
            return []