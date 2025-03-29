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


from dimos.stream.videostream import VideoStream

import warnings
from concurrent.futures import ProcessPoolExecutor, as_completed
from collections import deque
from dimos.types.depth_map import DepthMapType
from dimos.types.label import LabelType
from dimos.types.pointcloud import PointCloudType
from dimos.types.segmentation import SegmentationType
import os

class DataPipeline:
    def __init__(self, video_stream: VideoStream,
                 run_depth: bool = False,
                 run_labels: bool = False,
                 run_pointclouds: bool = False,
                 run_segmentations: bool = False,
                 max_workers: int = 4):
        self.video_stream = video_stream
        self.run_depth = run_depth
        self.run_labels = run_labels
        self.run_pointclouds = run_pointclouds
        self.run_segmentations = run_segmentations
        self.max_workers = max_workers

        # Validate pipeline configuration
        self._validate_pipeline()

        # Initialize the pipeline
        self._initialize_pipeline()

        # Storage for processed data
        self.generated_depth_maps = deque()
        self.generated_labels = deque()
        self.generated_pointclouds = deque()
        self.generated_segmentations = deque()

    def _validate_pipeline(self):
        """Validate the pipeline configuration based on dependencies."""
        if self.run_pointclouds and not self.run_depth:
            raise ValueError("PointClouds generation requires Depth maps. "
                             "Enable run_depth=True to use run_pointclouds=True.")

        if self.run_segmentations and not self.run_labels:
            raise ValueError("Segmentations generation requires Labels. "
                             "Enable run_labels=True to use run_segmentations=True.")

        if not any([self.run_depth, self.run_labels, self.run_pointclouds, self.run_segmentations]):
            warnings.warn("No pipeline layers selected to run. The DataPipeline will be initialized without any processing.")

    def _initialize_pipeline(self):
        """Initialize necessary components based on selected pipeline layers."""
        if self.run_depth:
            from .depth import DepthProcessor
            self.depth_processor = DepthProcessor(debug=True)
            print("Depth map generation enabled.")
        else:
            self.depth_processor = None

        if self.run_labels:
            from .labels import LabelProcessor
            self.labels_processor = LabelProcessor(debug=True)
            print("Label generation enabled.")
        else:
            self.labels_processor = None

        if self.run_pointclouds:
            from .pointcloud import PointCloudProcessor
            self.pointcloud_processor = PointCloudProcessor(debug=True)
            print("PointCloud generation enabled.")
        else:
            self.pointcloud_processor = None

        if self.run_segmentations:
            from .segment import SegmentProcessor
            self.segmentation_processor = SegmentProcessor(debug=True)
            print("Segmentation generation enabled.")
        else:
            self.segmentation_processor = None

    def run(self):
        """Execute the selected pipeline layers."""
        try:
            for frame in self.video_stream:
                result = self._process_frame(frame)
                depth_map, label, pointcloud, segmentation = result

                if depth_map is not None:
                    self.generated_depth_maps.append(depth_map)
                if label is not None:
                    self.generated_labels.append(label)
                if pointcloud is not None:
                    self.generated_pointclouds.append(pointcloud)
                if segmentation is not None:
                    self.generated_segmentations.append(segmentation)
        except KeyboardInterrupt:
            print("Pipeline interrupted by user.")

    def _process_frame(self, frame):
        """Process a single frame and return results."""
        depth_map = None
        label = None
        pointcloud = None
        segmentation = None

        if self.run_depth:
            depth_map = self.depth_processor.process(frame)

        if self.run_labels:
            label = self.labels_processor.caption_image_data(frame)

        if self.run_pointclouds and isinstance(depth_map, DepthMapType) and self.pointcloud_processor:
            pointcloud = self.pointcloud_processor.process_frame(frame, depth_map.depth_data)

        if self.run_segmentations and isinstance(label, LabelType) and self.segmentation_processor:
            segmentation = self.segmentation_processor.process_frame(frame, label.labels)

        return depth_map, label, pointcloud, segmentation

    def save_all_processed_data(self, directory: str):
        """Save all processed data to files in the specified directory."""
        os.makedirs(directory, exist_ok=True)

        for i, depth_map in enumerate(self.generated_depth_maps):
            if isinstance(depth_map, DepthMapType):
                depth_map.save_to_file(os.path.join(directory, f"depth_map_{i}.npy"))

        for i, label in enumerate(self.generated_labels):
            if isinstance(label, LabelType):
                label.save_to_json(os.path.join(directory, f"labels_{i}.json"))

        for i, pointcloud in enumerate(self.generated_pointclouds):
            if isinstance(pointcloud, PointCloudType):
                pointcloud.save_to_file(os.path.join(directory, f"pointcloud_{i}.pcd"))

        for i, segmentation in enumerate(self.generated_segmentations):
            if isinstance(segmentation, SegmentationType):
                segmentation.save_masks(os.path.join(directory, f"segmentation_{i}"))
