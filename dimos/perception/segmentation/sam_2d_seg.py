import cv2
import numpy as np
import time
from ultralytics import FastSAM
from dimos.perception.segmentation.utils import extract_masks_bboxes_probs_names, \
                                                filter_segmentation_results, \
                                                plot_results, \
                                                crop_images_from_bboxes
from dimos.perception.common.detection2d_tracker import target2dTracker, get_tracked_results
from dimos.perception.segmentation.image_analyzer import ImageAnalyzer
import os
from collections import deque
from concurrent.futures import ThreadPoolExecutor

class Sam2DSegmenter:
    def __init__(self, model_path="FastSAM-s.pt", device="cuda", 
                 min_analysis_interval=5.0, use_tracker=True, use_analyzer=True,
                 use_rich_labeling=False):
        # Core components
        self.device = device
        self.model = FastSAM(model_path)
        self.use_tracker = use_tracker
        self.use_analyzer = use_analyzer
        self.use_rich_labeling = use_rich_labeling

        module_dir = os.path.dirname(__file__)
        self.tracker_config = os.path.join(module_dir, 'config', 'custom_tracker.yaml')

        # Initialize tracker if enabled
        if self.use_tracker:
            self.tracker = target2dTracker(
                history_size=80,
                score_threshold_start=0.7,
                score_threshold_stop=0.05,
                min_frame_count=10,
                max_missed_frames=50,
                min_area_ratio=0.05,
                max_area_ratio=0.4,
                texture_range=(0.0, 0.35),
                border_safe_distance=100,
                weights={"prob": 1.0, "temporal": 3.0, "texture": 2.0, "border": 3.0, "size": 1.0}
            )
        
        # Initialize analyzer components if enabled
        if self.use_analyzer:
            self.image_analyzer = ImageAnalyzer()
            self.min_analysis_interval = min_analysis_interval
            self.last_analysis_time = 0
            self.to_be_analyzed = deque()
            self.object_names = {}
            self.analysis_executor = ThreadPoolExecutor(max_workers=1)
            self.current_future = None
            self.current_queue_ids = None

    def process_image(self, image):
        """Process an image and return segmentation results."""
        results = self.model.track(
            source=image,
            device=self.device,
            retina_masks=True,
            conf=0.6,
            iou=0.9,
            persist=True,
            verbose=False,
            tracker=self.tracker_config,
        )

        if len(results) > 0:
            # Get initial segmentation results
            masks, bboxes, track_ids, probs, names, areas = extract_masks_bboxes_probs_names(results[0])
            
            # Filter results
            filtered_masks, filtered_bboxes, filtered_track_ids, filtered_probs, filtered_names, filtered_texture_values = \
                filter_segmentation_results(image, masks, bboxes, track_ids, probs, names, areas)
            
            if self.use_tracker:
                # Update tracker with filtered results
                tracked_targets = self.tracker.update(
                    image,
                    filtered_masks,
                    filtered_bboxes,
                    filtered_track_ids,
                    filtered_probs,
                    filtered_names,
                    filtered_texture_values,
                )
                
                # Get tracked results
                tracked_masks, tracked_bboxes, tracked_target_ids, tracked_probs, tracked_names = get_tracked_results(tracked_targets)
                
                if self.use_analyzer:
                    # Update analysis queue with tracked IDs
                    target_id_set = set(tracked_target_ids)

                    # Remove untracked objects from object_names
                    all_target_ids = list(self.tracker.targets.keys())
                    self.object_names = {track_id: name for track_id, name in self.object_names.items()
                                        if track_id in all_target_ids}
                    
                    # Remove untracked objects from queue and results
                    self.to_be_analyzed = deque([track_id for track_id in self.to_be_analyzed 
                                           if track_id in target_id_set])
                    
                    # Filter out any IDs being analyzed from the to_be_analyzed queue
                    if self.current_queue_ids:
                        self.to_be_analyzed = deque([tid for tid in self.to_be_analyzed 
                                               if tid not in self.current_queue_ids])
                    
                    # Add new track_ids to analysis queue
                    for track_id in tracked_target_ids:
                        if track_id not in self.object_names and track_id not in self.to_be_analyzed:
                            self.to_be_analyzed.append(track_id)
                
                return tracked_masks, tracked_bboxes, tracked_target_ids, tracked_probs, tracked_names
            else:
                # Return filtered results directly if tracker is disabled
                return filtered_masks, filtered_bboxes, filtered_track_ids, filtered_probs, filtered_names
        return [], [], [], [], []

    def check_analysis_status(self, tracked_target_ids):
        """Check if analysis is complete and prepare new queue if needed."""
        if not self.use_analyzer:
            return None, None

        current_time = time.time()
        
        # Check if current queue analysis is complete
        if self.current_future and self.current_future.done():
            try:
                results = self.current_future.result()
                if results is not None:
                    # Map results to track IDs
                    object_list = eval(results)
                    for track_id, result in zip(self.current_queue_ids, object_list):
                        self.object_names[track_id] = result
            except Exception as e:
                print(f"Queue analysis failed: {e}")
            self.current_future = None
            self.current_queue_ids = None
            self.last_analysis_time = current_time

        # If enough time has passed and we have items to analyze, start new analysis
        if (not self.current_future and self.to_be_analyzed and 
            current_time - self.last_analysis_time >= self.min_analysis_interval):

            queue_indices = []
            queue_ids = []
            
            # Collect all valid track IDs from the queue
            while self.to_be_analyzed:
                track_id = self.to_be_analyzed[0]
                if track_id in tracked_target_ids:
                    bbox_idx = tracked_target_ids.index(track_id)
                    queue_indices.append(bbox_idx)
                    queue_ids.append(track_id)
                self.to_be_analyzed.popleft()
                
            if queue_indices:
                return queue_indices, queue_ids
        return None, None

    def run_analysis(self, frame, tracked_bboxes, tracked_target_ids):
        """Run queue image analysis in background."""
        if not self.use_analyzer:
            return

        queue_indices, queue_ids = self.check_analysis_status(tracked_target_ids)
        if queue_indices:
            selected_bboxes = [tracked_bboxes[i] for i in queue_indices]
            cropped_images = crop_images_from_bboxes(frame, selected_bboxes)
            if cropped_images:
                self.current_queue_ids = queue_ids
                print(f"Analyzing objects with track_ids: {queue_ids}")

                if self.use_rich_labeling:
                    prompt_type = "rich"
                    cropped_images.append(frame)
                else:
                    prompt_type = "normal"
                    
                self.current_future = self.analysis_executor.submit(
                    self.image_analyzer.analyze_images,
                    cropped_images,
                    prompt_type=prompt_type
                )

    def get_object_names(self, track_ids, tracked_names):
        """Get object names for the given track IDs, falling back to tracked names."""
        if not self.use_analyzer:
            return tracked_names
            
        return [self.object_names.get(track_id, tracked_name) 
                for track_id, tracked_name in zip(track_ids, tracked_names)]

    def visualize_results(self, image, masks, bboxes, track_ids, probs, names):
        """Generate an overlay visualization with segmentation results and object names."""
        return plot_results(image, masks, bboxes, track_ids, probs, names)

    def cleanup(self):
        """Cleanup resources."""
        if self.use_analyzer:
            self.analysis_executor.shutdown()


def main():
    # Example usage with different configurations
    cap = cv2.VideoCapture(0)
    
    # Example 1: Full functionality with rich labeling
    segmenter = Sam2DSegmenter(
        min_analysis_interval=4.0, 
        use_tracker=True, 
        use_analyzer=True,
        use_rich_labeling=True  # Enable rich labeling
    )
    
    # Example 2: Full functionality with normal labeling
    # segmenter = Sam2DSegmenter(min_analysis_interval=4.0, use_tracker=True, use_analyzer=True)
    
    # Example 3: Tracker only (analyzer disabled)
    # segmenter = Sam2DSegmenter(use_analyzer=False)
    
    # Example 4: Basic segmentation only (both tracker and analyzer disabled)
    # segmenter = Sam2DSegmenter(use_tracker=False, use_analyzer=False)

    try:
        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break

            start_time = time.time()
            
            # Process image and get results
            masks, bboxes, target_ids, probs, names = segmenter.process_image(frame)
            
            # Run analysis if enabled
            if segmenter.use_tracker and segmenter.use_analyzer:
                segmenter.run_analysis(frame, bboxes, target_ids)
                names = segmenter.get_object_names(target_ids, names)

            #processing_time = time.time() - start_time
            #print(f"Processing time: {processing_time:.2f}s")

            overlay = segmenter.visualize_results(
                frame, 
                masks, 
                bboxes, 
                target_ids, 
                probs, 
                names
            )

            cv2.imshow("Segmentation", overlay)
            key = cv2.waitKey(1)
            if key & 0xFF == ord("q"):
                break

    finally:
        segmenter.cleanup()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
