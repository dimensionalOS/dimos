import numpy as np
import cv2
import torch
import time


class SimpleTracker:
    def __init__(self, history_size=100, min_count=10, count_window=20):
        """
        Simple temporal tracker that counts appearances in a fixed window.
        :param history_size: Number of past frames to remember
        :param min_count: Minimum number of appearances required
        :param count_window: Number of latest frames to consider for counting
        """
        self.history = []
        self.history_size = history_size
        self.min_count = min_count
        self.count_window = count_window
        self.total_counts = {}

    def update(self, track_ids):
        # Add new frame's track IDs to history
        self.history.append(track_ids)
        if len(self.history) > self.history_size:
            self.history.pop(0)

        # Consider only the latest `count_window` frames for counting
        recent_history = self.history[-self.count_window:]
        all_tracks = np.concatenate(recent_history) if recent_history else np.array([])
        
        # Compute occurrences efficiently using numpy
        unique_ids, counts = np.unique(all_tracks, return_counts=True)
        id_counts = dict(zip(unique_ids, counts))
        
        # Update total counts but ensure it only contains IDs within the history size
        total_tracked_ids = np.concatenate(self.history) if self.history else np.array([])
        unique_total_ids, total_counts = np.unique(total_tracked_ids, return_counts=True)
        self.total_counts = dict(zip(unique_total_ids, total_counts))
        
        # Return IDs that appear often enough
        return [track_id for track_id, count in id_counts.items() if count >= self.min_count]
    
    def get_total_counts(self):
        """Returns the total count of each tracking ID seen over time, limited to history size."""
        return self.total_counts


def extract_masks_bboxes_probs_names(result, max_size=0.7):
    """
    Extracts masks, bounding boxes, probabilities, and class names from one Ultralytics result object.
    
    Parameters:
    result: Ultralytics result object
    max_size: float, maximum allowed size of object relative to image (0-1)
    
    Returns:
    tuple: (masks, bboxes, track_ids, probs, names, areas)
    """
    masks = []
    bboxes = []
    track_ids = []
    probs = []
    names = []
    areas = []

    if result.masks is None:
        return masks, bboxes, track_ids, probs, names, areas
    
    total_area = result.masks.orig_shape[0] * result.masks.orig_shape[1]

    for box, mask_data in zip(result.boxes, result.masks.data):
        mask_numpy = mask_data

        # Extract bounding box
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        
        # Extract track_id if available
        track_id = -1  # default if no tracking
        if hasattr(box, 'id') and box.id is not None:
            track_id = int(box.id[0].item())
        
        # Extract probability and class index
        conf = float(box.conf[0])
        cls_idx = int(box.cls[0])
        area = (x2 - x1) * (y2 - y1)

        if area / total_area > max_size:
            continue

        masks.append(mask_numpy)
        bboxes.append([x1, y1, x2, y2])
        track_ids.append(track_id)
        probs.append(conf)
        names.append(result.names[cls_idx])
        areas.append(area)

    return masks, bboxes, track_ids, probs, names, areas

def compute_texture_map(frame, blur_size=3):
    """
    Compute texture map using gradient statistics.
    Returns high values for textured regions and low values for smooth regions.
    
    Parameters:
    frame: BGR image
    blur_size: Size of Gaussian blur kernel for pre-processing
    
    Returns:
    numpy array: Texture map with values normalized to [0,1]
    """
    # Convert to grayscale
    if len(frame.shape) == 3:
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    else:
        gray = frame
        
    # Pre-process with slight blur to reduce noise
    if blur_size > 0:
        gray = cv2.GaussianBlur(gray, (blur_size, blur_size), 0)
    
    # Compute gradients in x and y directions
    grad_x = cv2.Sobel(gray, cv2.CV_32F, 1, 0, ksize=3)
    grad_y = cv2.Sobel(gray, cv2.CV_32F, 0, 1, ksize=3)
    
    # Compute gradient magnitude and direction
    magnitude = np.sqrt(grad_x**2 + grad_y**2)
    
    # Compute local standard deviation of gradient magnitude
    texture_map = cv2.GaussianBlur(magnitude, (15, 15), 0)
    
    # Normalize to [0,1]
    texture_map = (texture_map - texture_map.min()) / (texture_map.max() - texture_map.min() + 1e-8)
    
    return texture_map


def filter_segmentation_results(frame, masks, bboxes, track_ids, probs, names, areas, texture_threshold=0.07, size_filter=800):
    """
    Filters segmentation results using both overlap and saliency detection.
    Uses mask_sum tensor for efficient overlap detection.
    
    Parameters:
    masks: list of torch.Tensor containing mask data
    bboxes: list of bounding boxes [x1, y1, x2, y2]
    track_ids: list of tracking IDs
    probs: list of confidence scores
    names: list of class names
    areas: list of object areas
    frame: BGR image for computing saliency
    texture_threshold: Average texture value required for mask to be kept
    size_filter: Minimum size of the object to be kept
    
    Returns:
    tuple: (filtered_masks, filtered_bboxes, filtered_track_ids, filtered_probs, filtered_names, filtered_texture_values, texture_map)
    """
    if len(masks) <= 1:
        return masks, bboxes, track_ids, probs, names, []
        
    # Compute texture map once and convert to tensor
    texture_map = compute_texture_map(frame)
    
    # Sort by area (smallest to largest)
    sorted_indices = torch.tensor(areas).argsort(descending=False)

    device = masks[0].device  # Get the device of the first mask
    
    # Create mask_sum tensor where each pixel stores the index of the mask that claims it
    mask_sum = torch.zeros_like(masks[0], dtype=torch.int32)
    
    texture_map = torch.from_numpy(texture_map).to(device)  # Convert texture_map to tensor and move to device
    
    filtered_texture_values = []  # List to store texture values of filtered masks
    
    for i, idx in enumerate(sorted_indices):
        mask = masks[idx]
        # Compute average texture value within mask
        texture_value = torch.mean(texture_map[mask > 0]) if torch.any(mask > 0) else 0
        
        # Only claim pixels if mask passes texture threshold
        if texture_value >= texture_threshold:
            mask_sum[mask > 0] = i
            filtered_texture_values.append(texture_value.item())  # Store the texture value as a Python float
    
    # Get indices that appear in mask_sum (these are the masks we want to keep)
    keep_indices, counts = torch.unique(mask_sum[mask_sum > 0], return_counts=True)
    size_indices = counts > size_filter
    keep_indices = keep_indices[size_indices]

    sorted_indices = sorted_indices.cpu()
    keep_indices = keep_indices.cpu()
    
    # Map back to original indices and filter
    final_indices = sorted_indices[keep_indices].tolist()
    
    filtered_masks = [masks[i] for i in final_indices]
    filtered_bboxes = [bboxes[i] for i in final_indices]
    filtered_track_ids = [track_ids[i] for i in final_indices]
    filtered_probs = [probs[i] for i in final_indices]
    filtered_names = [names[i] for i in final_indices]

    return filtered_masks, filtered_bboxes, filtered_track_ids, filtered_probs, filtered_names, filtered_texture_values


def plot_results(image, masks, bboxes, track_ids, probs, names, alpha=0.5):
    """
    Draws bounding boxes, masks, and labels on the given image with enhanced visualization.
    Includes object names in the overlay and improved text visibility.
    """
    h, w = image.shape[:2]
    overlay = image.copy()

    for mask, bbox, track_id, prob, name in zip(masks, bboxes, track_ids, probs, names):
        # Convert mask tensor to numpy if needed
        if isinstance(mask, torch.Tensor):
            mask = mask.cpu().numpy()

        mask_resized = cv2.resize(mask, (w, h), interpolation=cv2.INTER_LINEAR)
        
        # Generate consistent color based on track_id
        if track_id != -1:
            np.random.seed(track_id)
            color = np.random.randint(0, 255, (3,), dtype=np.uint8)
            np.random.seed(None)
        else:
            color = np.random.randint(0, 255, (3,), dtype=np.uint8)
            
        # Apply mask color
        overlay[mask_resized > 0.5] = color

        # Draw bounding box
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(overlay, (x1, y1), (x2, y2), color.tolist(), 2)

        # Prepare label text
        label = f"ID:{track_id} {prob:.2f}"
        if name:  # Add object name if available
            label += f" {name}"

        # Calculate text size for background rectangle
        (text_w, text_h), _ = cv2.getTextSize(
            label, 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            1
        )

        # Draw background rectangle for text
        cv2.rectangle(
            overlay, 
            (x1, y1-text_h-8), 
            (x1+text_w+4, y1), 
            color.tolist(), 
            -1
        )

        # Draw text with white color for better visibility
        cv2.putText(
            overlay,
            label,
            (x1+2, y1-5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),  # White text
            1
        )

    # Blend overlay with original image
    result = cv2.addWeighted(overlay, alpha, image, 1 - alpha, 0)
    return result


def crop_images_from_bboxes(image, bboxes, buffer=0):
    """
    Crops regions from an image based on bounding boxes with an optional buffer.

    Parameters:
    image (numpy array): Input image.
    bboxes (list of lists): List of bounding boxes [x1, y1, x2, y2].
    buffer (int): Number of pixels to expand each bounding box.

    Returns:
    list of numpy arrays: Cropped image regions.
    """
    height, width, _ = image.shape
    cropped_images = []

    for bbox in bboxes:
        x1, y1, x2, y2 = bbox

        # Apply buffer
        x1 = max(0, x1 - buffer)
        y1 = max(0, y1 - buffer)
        x2 = min(width, x2 + buffer)
        y2 = min(height, y2 + buffer)

        cropped_image = image[int(y1):int(y2), int(x1):int(x2)]
        cropped_images.append(cropped_image)

    return cropped_images
