# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Best-effort episode video using the pinned VLN-CE visualization utilities."""

import json
import os
import tempfile

from habitat.utils.visualizations.utils import images_to_video, observations_to_image

FPS = 10


class NativeEpisodeVideo:
    def __init__(self, output_path, metadata_path, environment):
        self.output_path = str(output_path)
        self.metadata_path = str(metadata_path)
        self.environment = environment
        self.frames = []
        self.control_count = 0
        self.terminal_reason = None
        self.diagnostic = None

    def capture_initial(self):
        self._capture(repeats=FPS)

    def capture_control(self):
        self.control_count += 1
        self._capture()

    def capture_terminal(self, reason):
        self.terminal_reason = str(reason)
        self._capture(repeats=FPS)

    def close(self):
        if self.diagnostic is None:
            try:
                directory = os.path.dirname(self.output_path)
                with tempfile.TemporaryDirectory(dir=directory) as temporary:
                    images_to_video(
                        self.frames,
                        temporary,
                        "native-render",
                        fps=FPS,
                    )
                    os.replace(os.path.join(temporary, "native-render.mp4"), self.output_path)
            except Exception as error:
                self._fail(error)
        payload = {
            "schema_version": "native-render.v1",
            "status": "failed" if self.diagnostic else "completed",
            "frame_count": len(self.frames),
            "control_frame_count": self.control_count,
            "fps": FPS,
            "composition": "upstream_habitat_rgb_depth_and_non_oracle_top_down_map",
            "terminal_reason": self.terminal_reason,
        }
        if self.diagnostic:
            payload["diagnostic"] = self.diagnostic
            if os.path.exists(self.output_path):
                os.unlink(self.output_path)
        _write_json_atomic(self.metadata_path, payload)

    def _capture(self, repeats=1):
        if self.diagnostic is not None:
            return
        try:
            observations = dict(self.environment.observations)
            rgb_height, rgb_width = observations["rgb"].shape[:2]
            if observations["depth"].shape[:2] != (rgb_height, rgb_width):
                import cv2

                depth = cv2.resize(observations["depth"], (rgb_width, rgb_height))
                observations["depth"] = depth[..., None]
            frame = observations_to_image(
                observations,
                self.environment.visualization_info(),
            )
            self.frames.extend([frame] * repeats)
        except Exception as error:
            self._fail(error)

    def _fail(self, error):
        if self.diagnostic is None:
            detail = str(error).replace(self.output_path, "<render-output>")
            self.diagnostic = f"{type(error).__name__}: {detail}"[:512]


def _write_json_atomic(path, payload):
    directory = os.path.dirname(path)
    descriptor, temporary = tempfile.mkstemp(prefix=".render-", dir=directory)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as handle:
            json.dump(payload, handle, sort_keys=True, separators=(",", ":"))
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
