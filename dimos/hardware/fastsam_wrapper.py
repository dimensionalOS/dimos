"""
FastSAM wrapper with native multi-point support
"""
import numpy as np
import torch
import cv2
from PIL import Image
import os
import sys

# Add the FastSAM directory to path for imports
fastsam_path = os.path.join(os.path.dirname(__file__), 'FastSAM')
if os.path.exists(fastsam_path):
    sys.path.insert(0, fastsam_path)
else:
    raise RuntimeError(f"FastSAM directory not found at {fastsam_path}")

# Import from the cloned FastSAM repository
from fastsam import FastSAM, FastSAMPrompt
print("Native FastSAM loaded successfully")


class FastSAMWrapper:
    """Wrapper for FastSAM with multi-point prompt support"""
    
    def __init__(self, model_path):
        """
        Initialize FastSAM model

        Args:
            model_path: Path to FastSAM model weights
        """
        self.model_path = model_path
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'

        # Load with weights_only=False for PyTorch 2.6 compatibility
        # This is safe as we're loading our own trusted model
        # Temporarily override torch.load to disable weights_only check
        original_load = torch.load

        def custom_load(*args, **kwargs):
            kwargs['weights_only'] = False
            return original_load(*args, **kwargs)

        torch.load = custom_load
        try:
            self.fastsam_model = FastSAM(model_path)
        finally:
            torch.load = original_load  # Always restore original function

        print(f"FastSAM initialized with model: {model_path}, device: {self.device}")
        
    def segment_with_points(self, image, points, point_labels, conf=0.25, iou=0.7):
        """
        Segment object using multiple points with labels
        
        Args:
            image: Input image (numpy array or PIL Image)
            points: List of (x, y) coordinates
            point_labels: List of labels (1=foreground, 0=background)
            conf: Confidence threshold (lowered for better detection)
            iou: IOU threshold
            
        Returns:
            Binary mask of the segmented object
        """
        # Save image temporarily for FastSAM processing
        temp_img_path = "temp_fastsam_frame.jpg"
        if isinstance(image, np.ndarray):
            cv2.imwrite(temp_img_path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        else:
            image.save(temp_img_path)
        
        # Run FastSAM everything mode
        everything_results = self.fastsam_model(
            temp_img_path,
            device=self.device,
            retina_masks=True,
            imgsz=1024,
            conf=conf,
            iou=iou
        )
        
        # Process with point prompt
        prompt_process = FastSAMPrompt(temp_img_path, everything_results, device=self.device)
        
        # For multi-point, we need to use a custom approach since FastSAM's native multi-point
        # has issues with background points
        print(f"Using {len(points)} points: {len([l for l in point_labels if l==1])} foreground, {len([l for l in point_labels if l==0])} background")
        
        if len([l for l in point_labels if l == 0]) > 0:
            # Has background points - use custom scoring
            masks = prompt_process._format_results(everything_results[0], 0)
            if not masks:
                print("No masks found")
                ann = None
            else:
                h = masks[0]['segmentation'].shape[0] 
                w = masks[0]['segmentation'].shape[1]
                target_height = prompt_process.img.shape[0]
                target_width = prompt_process.img.shape[1]
                
                # Scale points if needed
                if h != target_height or w != target_width:
                    scaled_points = [[int(point[0] * w / target_width), int(point[1] * h / target_height)] for point in points]
                else:
                    scaled_points = points
                
                best_mask = None
                best_score = -float('inf')
                
                for mask_dict in masks:
                    mask = mask_dict['segmentation']
                    score = 0
                    
                    # Check each point
                    for i, point in enumerate(scaled_points):
                        px, py = point
                        if point_labels[i] == 1:  # Foreground
                            if mask[py, px]:
                                score += 10  # Big reward for containing foreground
                            else:
                                score -= 100  # Big penalty for missing foreground
                        else:  # Background  
                            if mask[py, px]:
                                score -= 1  # Penalty for containing background
                            else:
                                score += 0.1  # Small reward for excluding background
                    
                    if score > best_score:
                        best_score = score
                        best_mask = mask
                
                print(f"Best mask score: {best_score}")
                ann = np.array([best_mask]) if best_mask is not None else None
        else:
            # No background points - use native FastSAM
            ann = prompt_process.point_prompt(points=points, pointlabel=point_labels)
        
        # Debug: Check what FastSAM returns
        print(f"FastSAM returned type: {type(ann)}")
        if ann is not None:
            if isinstance(ann, np.ndarray):
                print(f"Direct array shape: {ann.shape}, dtype: {ann.dtype}, unique values: {np.unique(ann)[:10]}")
            elif isinstance(ann, list):
                print(f"List with {len(ann)} items")
                if len(ann) > 0:
                    print(f"First item type: {type(ann[0])}")
                    if isinstance(ann[0], dict):
                        print(f"First item keys: {ann[0].keys()}")
        
        # Clean up temp file
        if os.path.exists(temp_img_path):
            os.remove(temp_img_path)
        
        # Convert annotation to binary mask
        if ann is not None:
            if isinstance(ann, np.ndarray):
                # Direct numpy array result
                if ann.ndim == 2:
                    return (ann > 0).astype(np.uint8) * 255
                elif ann.ndim == 3 and ann.shape[0] == 1:
                    # Squeeze out the batch dimension
                    return (ann[0] > 0).astype(np.uint8) * 255
            elif isinstance(ann, list) and len(ann) > 0:
                # List of annotations
                if isinstance(ann[0], dict) and 'segmentation' in ann[0]:
                    mask = ann[0]['segmentation']
                else:
                    mask = ann[0]
                
                if isinstance(mask, np.ndarray):
                    return (mask > 0).astype(np.uint8) * 255
        
        print("Warning: Could not extract mask from FastSAM result")
        return None
    
    def segment_with_box(self, image, bbox, conf=0.25, iou=0.7):
        """
        Segment object within the given bounding box
        
        Args:
            image: Input image (numpy array or PIL Image)
            bbox: Bounding box [x1, y1, x2, y2]
            conf: Confidence threshold
            iou: IOU threshold
            
        Returns:
            Binary mask of the segmented object
        """
        # Save image temporarily for FastSAM processing
        temp_img_path = "temp_fastsam_frame.jpg"
        if isinstance(image, np.ndarray):
            cv2.imwrite(temp_img_path, cv2.cvtColor(image, cv2.COLOR_RGB2BGR))
        else:
            image.save(temp_img_path)
        
        # Run FastSAM everything mode
        everything_results = self.fastsam_model(
            temp_img_path,
            device=self.device,
            retina_masks=True,
            imgsz=1024,
            conf=conf,
            iou=iou
        )
        
        # Process with box prompt
        prompt_process = FastSAMPrompt(temp_img_path, everything_results, device=self.device)
        
        # Use box prompt
        print(f"Using box prompt: [{bbox[0]}, {bbox[1]}, {bbox[2]}, {bbox[3]}]")
        ann = prompt_process.box_prompt(bboxes=[bbox])
        
        # Clean up temp file
        if os.path.exists(temp_img_path):
            os.remove(temp_img_path)
        
        # Convert annotation to binary mask
        if ann is not None:
            if isinstance(ann, np.ndarray):
                # Direct numpy array result
                if ann.ndim == 2:
                    return (ann > 0).astype(np.uint8) * 255
                elif ann.ndim == 3 and ann.shape[0] == 1:
                    # Squeeze out the batch dimension
                    return (ann[0] > 0).astype(np.uint8) * 255
            elif isinstance(ann, list) and len(ann) > 0:
                # List of annotations
                if isinstance(ann[0], dict) and 'segmentation' in ann[0]:
                    mask = ann[0]['segmentation']
                else:
                    mask = ann[0]
                
                if isinstance(mask, np.ndarray):
                    return (mask > 0).astype(np.uint8) * 255
        
        return None
    
    def segment_with_point(self, image, point, conf=0.25, iou=0.7):
        """
        Segment object at the given point (single point version)
        
        Args:
            image: Input image (numpy array or PIL Image)
            point: (x, y) coordinate
            conf: Confidence threshold
            iou: IOU threshold
            
        Returns:
            Binary mask of the segmented object
        """
        # Use multi-point method with a single foreground point
        return self.segment_with_points(image, [point], [1], conf, iou)
    
