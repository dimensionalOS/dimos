import cv2
import numpy as np
from ultralytics import YOLO
from dimos.perception.detection2d.utils import extract_detection_results, plot_results, filter_detections
import os


class Yolo2DDetector:
    def __init__(self, model_path="models/yolo11n.engine", device="cuda"):
        """
        Initialize the YOLO detector.
        
        Args:
            model_path (str): Path to the YOLO model weights
            device (str): Device to run inference on ('cuda' or 'cpu')
        """
        self.device = device
        self.model = YOLO(model_path)

        module_dir = os.path.dirname(__file__)
        self.tracker_config = os.path.join(module_dir, 'config', 'custom_tracker.yaml')

    def process_image(self, image):
        """
        Process an image and return detection results.
        
        Args:
            image: Input image in BGR format (OpenCV)
            
        Returns:
            tuple: (bboxes, track_ids, class_ids, confidences, names)
                - bboxes: list of [x1, y1, x2, y2] coordinates
                - track_ids: list of tracking IDs (or -1 if no tracking)
                - class_ids: list of class indices
                - confidences: list of detection confidences
                - names: list of class names
        """
        results = self.model.track(
            source=image,
            device=self.device,
            conf=0.5,
            iou=0.6,
            persist=True,
            verbose=False,
            tracker=self.tracker_config
        )

        if len(results) > 0:
            # Extract detection results
            bboxes, track_ids, class_ids, confidences, names = extract_detection_results(results[0])
            return bboxes, track_ids, class_ids, confidences, names
            
        return [], [], [], [], []

    def visualize_results(self, image, bboxes, track_ids, class_ids, confidences, names):
        """
        Generate visualization of detection results.
        
        Args:
            image: Original input image
            bboxes: List of bounding boxes
            track_ids: List of tracking IDs
            class_ids: List of class indices
            confidences: List of detection confidences
            names: List of class names
            
        Returns:
            Image with visualized detections
        """
        return plot_results(image, bboxes, track_ids, class_ids, confidences, names)


def main():
    """Example usage of the Yolo2DDetector class."""
    # Initialize video capture
    cap = cv2.VideoCapture(0)
    
    # Initialize detector
    detector = Yolo2DDetector()

    enable_person_filter = True

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            # Process frame
            bboxes, track_ids, class_ids, confidences, names = detector.process_image(frame)
            
            # Apply person filtering if enabled
            if enable_person_filter and len(bboxes) > 0:
                # Person is class_id 0 in COCO dataset
                bboxes, track_ids, class_ids, confidences, names = filter_detections(
                    bboxes, track_ids, class_ids, confidences, names,
                    class_filter=[0],  # 0 is the class_id for person
                    name_filter=['person']
                )
            
            # Visualize results
            if len(bboxes) > 0:
                frame = detector.visualize_results(
                    frame, 
                    bboxes, 
                    track_ids, 
                    class_ids, 
                    confidences, 
                    names
                )

            # Display results
            cv2.imshow("YOLO Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()