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

from ..base.stream_base import AnnotatorType, StreamBase, TransportType


class IsaacStream(StreamBase):
    """Isaac Sim stream implementation."""

    def __init__(  # type: ignore[no-untyped-def]
        self,
        simulator,
        width: int = 1920,
        height: int = 1080,
        fps: int = 60,
        camera_path: str = "/World/alfred_parent_prim/alfred_base_descr/chest_cam_rgb_camera_frame/chest_cam",
        annotator_type: AnnotatorType = "rgb",
        transport: TransportType = "tcp",
        rtsp_url: str = "rtsp://mediamtx:8554/stream",
        usd_path: str | Path | None = None,
    ) -> None:
        """Initialize the Isaac Sim stream."""
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

        # Import omni.replicator after SimulationApp initialization
        import omni.replicator.core as rep  # type: ignore[import-not-found]

        self.rep = rep

        # Initialize components
        if usd_path:
            self._load_stage(usd_path)
        self._setup_camera()  # type: ignore[no-untyped-call]
        self._setup_ffmpeg()
        self._setup_annotator()

    def _load_stage(self, usd_path: str | Path):  # type: ignore[no-untyped-def]
        """Load USD stage from file."""
        import omni.usd  # type: ignore[import-not-found]

        abs_path = str(Path(usd_path).resolve())
        omni.usd.get_context().open_stage(abs_path)
        self.stage = self.simulator.get_stage()
        if not self.stage:
            raise RuntimeError(f"Failed to load stage: {abs_path}")

    def _setup_camera(self):  # type: ignore[no-untyped-def]
        """Setup and validate camera."""
        self.stage = self.simulator.get_stage()
        camera_prim = self.stage.GetPrimAtPath(self.camera_path)
        if not camera_prim:
            raise RuntimeError(f"Failed to find camera at path: {self.camera_path}")

        self.render_product = self.rep.create.render_product(
            self.camera_path, resolution=(self.width, self.height)
        )

    def _setup_annotator(self) -> None:
        """Setup the specified annotator."""
        self.annotator = self.rep.AnnotatorRegistry.get_annotator(self.annotator_type)
        self.annotator.attach(self.render_product)

    def stream(self) -> None:
        """Start the streaming loop."""
        try:
            print("[Stream] Starting camera stream loop...")
            frame_count = 0
            start_time = time.time()

            while True:
                frame_start = time.time()

                # Step simulation and get frame
                step_start = time.time()
                self.rep.orchestrator.step()
                step_time = time.time() - step_start
                print(f"[Stream] Simulation step took {step_time * 1000:.2f}ms")

                frame = self.annotator.get_data()
                frame = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)

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
        self.simulator.close()
        print("[Cleanup] Successfully cleaned up resources")
