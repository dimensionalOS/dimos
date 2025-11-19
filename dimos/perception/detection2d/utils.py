import numpy as np
import cv2

def filter_detections(bboxes, track_ids, class_ids, confidences, names,
                    class_filter=None, name_filter=None, track_id_filter=None):
    """
    Filter detection results based on class IDs, names, and/or tracking IDs.
    
    Args:
        bboxes: List of bounding boxes [x1, y1, x2, y2]
        track_ids: List of tracking IDs
        class_ids: List of class indices
        confidences: List of detection confidences
        names: List of class names
        class_filter: List/set of class IDs to keep, or None to keep all
        name_filter: List/set of class names to keep, or None to keep all
        track_id_filter: List/set of track IDs to keep, or None to keep all
        
    Returns:
        tuple: (filtered_bboxes, filtered_track_ids, filtered_class_ids, 
                filtered_confidences, filtered_names)
    """
    # Convert filters to sets for efficient lookup
    if class_filter is not None:
        class_filter = set(class_filter)
    if name_filter is not None:
        name_filter = set(name_filter)
    if track_id_filter is not None:
        track_id_filter = set(track_id_filter)
    
    # Initialize lists for filtered results
    filtered_bboxes = []
    filtered_track_ids = []
    filtered_class_ids = []
    filtered_confidences = []
    filtered_names = []
    
    # Filter detections
    for bbox, track_id, class_id, conf, name in zip(
        bboxes, track_ids, class_ids, confidences, names):
        
        # Check if detection passes all specified filters
        keep = True
        
        if class_filter is not None:
            keep = keep and (class_id in class_filter)
            
        if name_filter is not None:
            keep = keep and (name in name_filter)
            
        if track_id_filter is not None:
            keep = keep and (track_id in track_id_filter)
            
        # If detection passes all filters, add it to results
        if keep:
            filtered_bboxes.append(bbox)
            filtered_track_ids.append(track_id)
            filtered_class_ids.append(class_id)
            filtered_confidences.append(conf)
            filtered_names.append(name)
    
    return (filtered_bboxes, filtered_track_ids, filtered_class_ids, 
            filtered_confidences, filtered_names)

def extract_detection_results(result, class_filter=None, name_filter=None, track_id_filter=None):
    """
    Extract and optionally filter detection information from a YOLO result object.
    
    Args:
        result: Ultralytics result object
        class_filter: List/set of class IDs to keep, or None to keep all
        name_filter: List/set of class names to keep, or None to keep all
        track_id_filter: List/set of track IDs to keep, or None to keep all
        
    Returns:
        tuple: (bboxes, track_ids, class_ids, confidences, names)
            - bboxes: list of [x1, y1, x2, y2] coordinates
            - track_ids: list of tracking IDs
            - class_ids: list of class indices
            - confidences: list of detection confidences
            - names: list of class names
    """
    bboxes = []
    track_ids = []
    class_ids = []
    confidences = []
    names = []

    if result.boxes is None:
        return bboxes, track_ids, class_ids, confidences, names

    for box in result.boxes:
        # Extract bounding box coordinates
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        
        # Extract tracking ID if available
        track_id = -1
        if hasattr(box, 'id') and box.id is not None:
            track_id = int(box.id[0].item())
            
        # Extract class information
        cls_idx = int(box.cls[0])
        name = result.names[cls_idx]
        
        # Extract confidence
        conf = float(box.conf[0])
        
        # Check filters before adding to results
        keep = True
        if class_filter is not None:
            keep = keep and (cls_idx in class_filter)
        if name_filter is not None:
            keep = keep and (name in name_filter)
        if track_id_filter is not None:
            keep = keep and (track_id in track_id_filter)
            
        if keep:
            bboxes.append([x1, y1, x2, y2])
            track_ids.append(track_id)
            class_ids.append(cls_idx)
            confidences.append(conf)
            names.append(name)

    return bboxes, track_ids, class_ids, confidences, names


def plot_results(image, bboxes, track_ids, class_ids, confidences, names, alpha=0.5):
    """
    Draw bounding boxes and labels on the image.
    
    Args:
        image: Original input image
        bboxes: List of bounding boxes [x1, y1, x2, y2]
        track_ids: List of tracking IDs
        class_ids: List of class indices
        confidences: List of detection confidences
        names: List of class names
        alpha: Transparency of the overlay
        
    Returns:
        Image with visualized detections
    """
    vis_img = image.copy()

    for bbox, track_id, conf, name in zip(bboxes, track_ids, confidences, names):
        # Generate consistent color based on track_id or class name
        if track_id != -1:
            np.random.seed(track_id)
        else:
            np.random.seed(hash(name) % 100000)
        color = np.random.randint(0, 255, (3,), dtype=np.uint8)
        np.random.seed(None)
            
        # Draw bounding box
        x1, y1, x2, y2 = map(int, bbox)
        cv2.rectangle(vis_img, (x1, y1), (x2, y2), color.tolist(), 2)

        # Prepare label text
        if track_id != -1:
            label = f"ID:{track_id} {name} {conf:.2f}"
        else:
            label = f"{name} {conf:.2f}"

        # Calculate text size for background rectangle
        (text_w, text_h), _ = cv2.getTextSize(
            label, 
            cv2.FONT_HERSHEY_SIMPLEX, 
            0.5, 
            1
        )

        # Draw background rectangle for text
        cv2.rectangle(
            vis_img, 
            (x1, y1-text_h-8), 
            (x1+text_w+4, y1), 
            color.tolist(), 
            -1
        )

        # Draw text with white color for better visibility
        cv2.putText(
            vis_img,
            label,
            (x1+2, y1-5),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (255, 255, 255),
            1
        )

    return vis_img