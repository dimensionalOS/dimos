# Copyright 2026 Dimensional Inc.
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

"""Best-effort private video rendering for one official VLN-CE episode."""

import json
import math
import os
import tempfile

import cv2
import numpy as np

FPS = 10.0
PANEL_SIZE = 448
FRAME_WIDTH = PANEL_SIZE * 2
FRAME_HEIGHT = PANEL_SIZE
HOLD_FRAMES = int(FPS)


class NativeEpisodeRenderer:
    """Record exact RGB observations beside a private navmesh trajectory view."""

    def __init__(self, output_path, metadata_path, environment):
        self.output_path = str(output_path)
        self.metadata_path = str(metadata_path)
        self.environment = environment
        self._writer = None
        self._occupancy = None
        self._frame_count = 0
        self._control_frame_count = 0
        self._terminal_reason = None
        self._diagnostic = None
        self._closed = False

    def capture_initial(self):
        """Hold the reset observation long enough to inspect it."""

        self._capture(HOLD_FRAMES)

    def capture_control(self):
        """Record one accepted benchmark control period."""

        self._control_frame_count += 1
        self._capture(1)

    def capture_terminal(self, reason):
        """Hold the terminal state and label its non-oracle reason."""

        self._terminal_reason = str(reason)
        self._capture(HOLD_FRAMES)

    def close(self):
        """Finalize video and atomically publish sanitized renderer metadata."""

        if self._closed:
            return
        self._closed = True
        if self._writer is not None:
            try:
                self._writer.release()
            except Exception as error:  # OpenCV backend exceptions vary by codec build.
                self._fail(error)
        if self._diagnostic is None and self._frame_count == 0:
            self._diagnostic = "RuntimeError: renderer captured no frames"
        payload = {
            "schema_version": "native-render.v1",
            "status": "failed" if self._diagnostic is not None else "completed",
            "frame_count": self._frame_count,
            "control_frame_count": self._control_frame_count,
            "fps": FPS,
            "width": FRAME_WIDTH,
            "height": FRAME_HEIGHT,
            "composition": "rgb_and_initial_floor_navmesh",
            "timing": "simulated_action_time",
            "terminal_reason": self._terminal_reason,
        }
        if self._diagnostic is not None:
            payload["diagnostic"] = self._diagnostic
        try:
            _write_json_atomic(self.metadata_path, payload)
        except Exception:
            # The supervisor synthesizes failed metadata when presentation output
            # is unavailable. Never let that alter Habitat's terminal result.
            pass

    def _capture(self, repeats):
        if self._diagnostic is not None or self._closed:
            return
        try:
            if self._occupancy is None:
                self._occupancy = self.environment.static_occupancy()
            frame = compose_native_frame(
                self.environment.observations["rgb"],
                self._occupancy,
                self.environment.trajectory,
                self.environment.pose,
                self._control_frame_count,
                self._terminal_reason,
            )
            if self._writer is None:
                fourcc = cv2.VideoWriter_fourcc(*"mp4v")
                self._writer = cv2.VideoWriter(
                    self.output_path,
                    fourcc,
                    FPS,
                    (FRAME_WIDTH, FRAME_HEIGHT),
                )
                if not self._writer.isOpened():
                    raise RuntimeError("OpenCV could not open the native MP4 writer")
            for _ in range(repeats):
                self._writer.write(frame)
                self._frame_count += 1
        except Exception as error:  # Rendering must never invalidate official scoring.
            self._fail(error)

    def _fail(self, error):
        if self._diagnostic is None:
            detail = str(error).replace(self.output_path, "<render-output>")
            self._diagnostic = f"{type(error).__name__}: {detail}"[:512]


def compose_native_frame(rgb, occupancy, trajectory, pose, control_count, terminal_reason=None):
    """Compose one deterministic BGR frame without any benchmark oracle overlays."""

    rgb_array = np.ascontiguousarray(rgb, dtype=np.uint8)
    if rgb_array.shape != (224, 224, 3):
        raise ValueError("native renderer requires the official 224x224 RGB observation")
    camera = cv2.cvtColor(rgb_array, cv2.COLOR_RGB2BGR)
    camera = cv2.resize(camera, (PANEL_SIZE, PANEL_SIZE), interpolation=cv2.INTER_LINEAR)
    map_panel = _map_panel(occupancy, trajectory, pose)
    frame = np.concatenate((camera, map_panel), axis=1)
    cv2.putText(
        frame,
        "Agent RGB",
        (16, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    label = f"Top-down trajectory | controls {control_count}"
    cv2.putText(
        frame,
        label,
        (PANEL_SIZE + 16, 30),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.62,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    if terminal_reason is not None:
        cv2.putText(
            frame,
            f"Terminal: {terminal_reason}",
            (16, FRAME_HEIGHT - 18),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.65,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
    return np.ascontiguousarray(frame, dtype=np.uint8)


def _map_panel(occupancy, trajectory, pose):
    traversability = np.asarray(occupancy["traversability"], dtype=np.uint8)
    expected = (int(occupancy["height"]), int(occupancy["width"]))
    if traversability.shape != expected:
        raise ValueError("native renderer received inconsistent occupancy dimensions")
    image = np.full((*expected, 3), 24, dtype=np.uint8)
    image[traversability != 0] = (205, 205, 205)
    points = [_grid_point(point, occupancy) for point in trajectory]
    if len(points) >= 2:
        cv2.polylines(
            image,
            [np.asarray(points, dtype=np.int32)],
            False,
            (255, 170, 0),
            max(2, int(min(expected) * 0.006)),
            cv2.LINE_AA,
        )
    center = _grid_point(pose.position, occupancy)
    forward_x, forward_z = _forward_xz(pose.rotation)
    arrow_length = max(8, int(min(expected) * 0.025))
    tip = (
        round(center[0] + forward_x * arrow_length),
        round(center[1] + forward_z * arrow_length),
    )
    cv2.arrowedLine(
        image,
        center,
        tip,
        (0, 230, 255),
        max(2, int(min(expected) * 0.008)),
        cv2.LINE_AA,
        tipLength=0.45,
    )
    return _fit_panel(image)


def _grid_point(position, occupancy):
    width = int(occupancy["width"])
    height = int(occupancy["height"])
    x_span = float(occupancy["upper_x"]) - float(occupancy["origin_x"])
    z_span = float(occupancy["upper_z"]) - float(occupancy["origin_z"])
    if x_span <= 0.0 or z_span <= 0.0:
        raise ValueError("native renderer requires positive navmesh bounds")
    column = int((float(position[0]) - float(occupancy["origin_x"])) / x_span * width)
    row = int((float(position[2]) - float(occupancy["origin_z"])) / z_span * height)
    return (
        min(max(column, 0), width - 1),
        min(max(row, 0), height - 1),
    )


def _forward_xz(rotation):
    x = float(rotation.x)
    y = float(rotation.y)
    z = float(rotation.z)
    w = float(rotation.w)
    forward_x = -(2.0 * x * z + 2.0 * w * y)
    forward_z = -(1.0 - 2.0 * x * x - 2.0 * y * y)
    length = math.hypot(forward_x, forward_z)
    if length == 0.0:
        return 0.0, -1.0
    return forward_x / length, forward_z / length


def _fit_panel(image):
    height, width = image.shape[:2]
    scale = min(float(PANEL_SIZE) / width, float(PANEL_SIZE) / height)
    resized_width = max(1, round(width * scale))
    resized_height = max(1, round(height * scale))
    resized = cv2.resize(
        image,
        (resized_width, resized_height),
        interpolation=cv2.INTER_NEAREST,
    )
    panel = np.full((PANEL_SIZE, PANEL_SIZE, 3), 12, dtype=np.uint8)
    x_offset = (PANEL_SIZE - resized_width) // 2
    y_offset = (PANEL_SIZE - resized_height) // 2
    panel[
        y_offset : y_offset + resized_height,
        x_offset : x_offset + resized_width,
    ] = resized
    return panel


def _write_json_atomic(path, payload):
    directory = os.path.dirname(path)
    descriptor, temporary = tempfile.mkstemp(prefix=".native-render-", dir=directory)
    try:
        with os.fdopen(descriptor, "w") as handle:
            json.dump(payload, handle, sort_keys=True, separators=(",", ":"))
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)
