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

# UNDER DEVELOPMENT 🚧🚧🚧

import cv2
import pycolmap
from pathlib import Path
from dimos.environment.environment import Environment

class COLMAPEnvironment(Environment):
    def initialize_from_images(self, image_dir):
        """Initialize the environment from a set of image frames or video."""
        image_dir = Path(image_dir)
        output_path = Path("colmap_output")
        output_path.mkdir(exist_ok=True)
        mvs_path = output_path / "mvs"
        database_path = output_path / "database.db"

        # Step 1: Feature extraction
        pycolmap.extract_features(database_path, image_dir)

        # Step 2: Feature matching
        pycolmap.match_exhaustive(database_path)

        # Step 3: Sparse reconstruction
        maps = pycolmap.incremental_mapping(database_path, image_dir, output_path)
        maps[0].write(output_path)

        # Step 4: Dense reconstruction (optional)
        pycolmap.undistort_images(mvs_path, output_path, image_dir)
        pycolmap.patch_match_stereo(mvs_path)  # Requires compilation with CUDA
        pycolmap.stereo_fusion(mvs_path / "dense.ply", mvs_path)

        return maps

    def initialize_from_video(self, video_path, frame_output_dir):
        """Extract frames from a video and initialize the environment."""
        video_path = Path(video_path)
        frame_output_dir = Path(frame_output_dir)
        frame_output_dir.mkdir(exist_ok=True)

        # Extract frames from the video
        self._extract_frames_from_video(video_path, frame_output_dir)

        # Initialize from the extracted frames
        return self.initialize_from_images(frame_output_dir)

    def _extract_frames_from_video(self, video_path, frame_output_dir):
        """Extract frames from a video and save them to a directory."""
        cap = cv2.VideoCapture(str(video_path))
        frame_count = 0

        while cap.isOpened():
            ret, frame = cap.read()
            if not ret:
                break
            frame_filename = frame_output_dir / f"frame_{frame_count:04d}.jpg"
            cv2.imwrite(str(frame_filename), frame)
            frame_count += 1

        cap.release()

    def label_objects(self):
        pass

    def get_visualization(self, format_type):
        pass

    def get_segmentations(self):
        pass

    def get_point_cloud(self, object_id=None):
        pass

    def get_depth_map(self):
        pass
