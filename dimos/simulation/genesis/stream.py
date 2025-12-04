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

from pathlib import Path
import time

import cv2
import numpy as np

from ..base.stream_base import AnnotatorType, StreamBase, TransportType


class GenesisStream(StreamBase):
    """Genesis stream implementation."""

    def __init__(  # type: ignore[no-untyped-def]
        self,
        simulator,
        width: int = 1920,
        height: int = 1080,
        fps: int = 60,
        camera_path: str = "/camera",
        annotator_type: AnnotatorType = "rgb",
        transport: TransportType = "tcp",
        rtsp_url: str = "rtsp://mediamtx:8554/stream",
        usd_path: str | Path | None = None,
    ) -> None:
        """Initialize the Genesis stream."""
        super().__init__(
            simulator=simulator,
            width=width,
            height=height,
            fps=fps,
            camera_path=camera_path,
            annotator_type=annotator_type,
            transport=transport,
            rtsp_url=rtsp_url,
            usd_path=usd_path,
        )

        self.scene = simulator.get_stage()

        # Initialize components
        if usd_path:
            self._load_stage(usd_path)
        self._setup_camera()
        self._setup_ffmpeg()
        self._setup_annotator()

        # Build scene after camera is set up
        simulator.build()

    def _load_stage(self, usd_path: str | Path) -> None:
        """Load stage from file."""
        # Genesis handles stage loading through simulator
        pass

    def _setup_camera(self) -> None:
        """Setup and validate camera."""
        self.camera = self.scene.add_camera(
            res=(self.width, self.height),
            pos=(3.5, 0.0, 2.5),
            lookat=(0, 0, 0.5),
            fov=30,
            GUI=False,
        )

    def _setup_annotator(self) -> None:
        """Setup the specified annotator."""
        # Genesis handles different render types through camera.render()
        pass

    def stream(self) -> None:
        """Start the streaming loop."""
        try:
            print("[Stream] Starting Genesis camera stream...")
            frame_count = 0
            start_time = time.time()

            while True:
                frame_start = time.time()

                # Step simulation and get frame
                step_start = time.time()
                self.scene.step()
                step_time = time.time() - step_start
                print(f"[Stream] Simulation step took {step_time * 1000:.2f}ms")

                # Get frame based on annotator type
                if self.annotator_type == "rgb":
                    frame, _, _, _ = self.camera.render(rgb=True)
                elif self.annotator_type == "normals":
                    _, _, _, frame = self.camera.render(normal=True)
                else:
                    frame, _, _, _ = self.camera.render(rgb=True)  # Default to RGB

                # Convert frame format if needed
                if isinstance(frame, np.ndarray):
                    frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)

                # Write to FFmpeg
                self.proc.stdin.write(frame.tobytes())  # type: ignore[attr-defined]
                self.proc.stdin.flush()  # type: ignore[attr-defined]

                # Log metrics
                frame_time = time.time() - frame_start
                print(f"[Stream] Total frame processing took {frame_time * 1000:.2f}ms")
                frame_count += 1

                if frame_count % 100 == 0:
                    elapsed_time = time.time() - start_time
                    current_fps = frame_count / elapsed_time
                    print(
                        f"[Stream] Processed {frame_count} frames | Current FPS: {current_fps:.2f}"
                    )

        except KeyboardInterrupt:
            print("\n[Stream] Received keyboard interrupt, stopping stream...")
        finally:
            self.cleanup()

    def cleanup(self) -> None:
        """Cleanup resources."""
        print("[Cleanup] Stopping FFmpeg process...")
        if hasattr(self, "proc"):
            self.proc.stdin.close()  # type: ignore[attr-defined]
            self.proc.wait()  # type: ignore[attr-defined]
        print("[Cleanup] Closing simulation...")
        try:
            self.simulator.close()
        except AttributeError:
            print("[Cleanup] Warning: Could not close simulator properly")
        print("[Cleanup] Successfully cleaned up resources")
