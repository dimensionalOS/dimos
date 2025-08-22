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

import time
from dataclasses import dataclass, field
from datetime import timedelta
from enum import Enum
from typing import Optional, Tuple

import cv2
import numpy as np
import reactivex as rx

# Import LCM types
from dimos_lcm.sensor_msgs.Image import Image as LCMImage
from dimos_lcm.std_msgs.Header import Header
from reactivex import operators as ops
from reactivex.observable import Observable
from reactivex.scheduler import ThreadPoolScheduler

from dimos.types.timestamped import Timestamped, TimestampedBufferCollection


class ImageFormat(Enum):
    """Supported image formats for internal representation."""

    BGR = "BGR"  # 8-bit Blue-Green-Red color
    RGB = "RGB"  # 8-bit Red-Green-Blue color
    RGBA = "RGBA"  # 8-bit RGB with Alpha
    BGRA = "BGRA"  # 8-bit BGR with Alpha
    GRAY = "GRAY"  # 8-bit Grayscale
    GRAY16 = "GRAY16"  # 16-bit Grayscale
    DEPTH = "DEPTH"  # 32-bit Float Depth


@dataclass
class Image(Timestamped):
    """Standardized image type with LCM integration."""

    msg_name = "sensor_msgs.Image"
    data: np.ndarray
    format: ImageFormat = field(default=ImageFormat.BGR)
    frame_id: str = field(default="")
    ts: float = field(default_factory=time.time)

    def __post_init__(self):
        """Validate image data and format."""
        if self.data is None:
            raise ValueError("Image data cannot be None")

        if not isinstance(self.data, np.ndarray):
            raise ValueError("Image data must be a numpy array")

        if len(self.data.shape) < 2:
            raise ValueError("Image data must be at least 2D")

        # Ensure data is contiguous for efficient operations
        if not self.data.flags["C_CONTIGUOUS"]:
            self.data = np.ascontiguousarray(self.data)

    @property
    def height(self) -> int:
        """Get image height."""
        return self.data.shape[0]

    @property
    def width(self) -> int:
        """Get image width."""
        return self.data.shape[1]

    @property
    def channels(self) -> int:
        """Get number of channels."""
        if len(self.data.shape) == 2:
            return 1
        elif len(self.data.shape) == 3:
            return self.data.shape[2]
        else:
            raise ValueError("Invalid image dimensions")

    @property
    def shape(self) -> Tuple[int, ...]:
        """Get image shape."""
        return self.data.shape

    @property
    def dtype(self) -> np.dtype:
        """Get image data type."""
        return self.data.dtype

    def copy(self) -> "Image":
        """Create a deep copy of the image."""
        return self.__class__(
            data=self.data.copy(),
            format=self.format,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    @classmethod
    def from_opencv(
        cls, cv_image: np.ndarray, format: ImageFormat = ImageFormat.BGR, **kwargs
    ) -> "Image":
        """Create Image from OpenCV image array."""
        return cls(data=cv_image, format=format, **kwargs)

    @classmethod
    def from_numpy(
        cls, np_image: np.ndarray, format: ImageFormat = ImageFormat.BGR, **kwargs
    ) -> "Image":
        """Create Image from numpy array."""
        return cls(data=np_image, format=format, **kwargs)

    @classmethod
    def from_file(cls, filepath: str, format: ImageFormat = ImageFormat.BGR) -> "Image":
        """Load image from file."""
        # OpenCV loads as BGR by default
        cv_image = cv2.imread(filepath, cv2.IMREAD_UNCHANGED)
        if cv_image is None:
            raise ValueError(f"Could not load image from {filepath}")

        # Detect format based on channels and data type
        if len(cv_image.shape) == 2:
            if cv_image.dtype == np.uint16:
                detected_format = ImageFormat.GRAY16
            else:
                detected_format = ImageFormat.GRAY
        elif cv_image.shape[2] == 3:
            detected_format = ImageFormat.BGR  # OpenCV default
        elif cv_image.shape[2] == 4:
            detected_format = ImageFormat.BGRA
        else:
            detected_format = format

        return cls(data=cv_image, format=detected_format)

    @classmethod
    def from_depth(cls, depth_data: np.ndarray, frame_id: str = "", ts: float = None) -> "Image":
        """Create Image from depth data (float32 array)."""
        if depth_data.dtype != np.float32:
            depth_data = depth_data.astype(np.float32)

        return cls(
            data=depth_data,
            format=ImageFormat.DEPTH,
            frame_id=frame_id,
            ts=ts if ts is not None else time.time(),
        )

    def to_opencv(self) -> np.ndarray:
        """Convert to OpenCV-compatible array (BGR format)."""
        if self.format == ImageFormat.BGR:
            return self.data
        elif self.format == ImageFormat.RGB:
            return cv2.cvtColor(self.data, cv2.COLOR_RGB2BGR)
        elif self.format == ImageFormat.RGBA:
            return cv2.cvtColor(self.data, cv2.COLOR_RGBA2BGR)
        elif self.format == ImageFormat.BGRA:
            return cv2.cvtColor(self.data, cv2.COLOR_BGRA2BGR)
        elif self.format == ImageFormat.GRAY:
            return self.data
        elif self.format == ImageFormat.GRAY16:
            return self.data
        elif self.format == ImageFormat.DEPTH:
            return self.data  # Depth images are already in the correct format
        else:
            raise ValueError(f"Unsupported format conversion: {self.format}")

    def to_rgb(self) -> "Image":
        """Convert image to RGB format."""
        if self.format == ImageFormat.RGB:
            return self.copy()
        elif self.format == ImageFormat.BGR:
            rgb_data = cv2.cvtColor(self.data, cv2.COLOR_BGR2RGB)
        elif self.format == ImageFormat.RGBA:
            return self.copy()  # Already RGB with alpha
        elif self.format == ImageFormat.BGRA:
            rgb_data = cv2.cvtColor(self.data, cv2.COLOR_BGRA2RGBA)
        elif self.format == ImageFormat.GRAY:
            rgb_data = cv2.cvtColor(self.data, cv2.COLOR_GRAY2RGB)
        elif self.format == ImageFormat.GRAY16:
            # Convert 16-bit grayscale to 8-bit then to RGB
            gray8 = (self.data / 256).astype(np.uint8)
            rgb_data = cv2.cvtColor(gray8, cv2.COLOR_GRAY2RGB)
        else:
            raise ValueError(f"Unsupported format conversion from {self.format} to RGB")

        return self.__class__(
            data=rgb_data,
            format=ImageFormat.RGB if self.format != ImageFormat.BGRA else ImageFormat.RGBA,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def to_bgr(self) -> "Image":
        """Convert image to BGR format."""
        if self.format == ImageFormat.BGR:
            return self.copy()
        elif self.format == ImageFormat.RGB:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_RGB2BGR)
        elif self.format == ImageFormat.RGBA:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_RGBA2BGR)
        elif self.format == ImageFormat.BGRA:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_BGRA2BGR)
        elif self.format == ImageFormat.GRAY:
            bgr_data = cv2.cvtColor(self.data, cv2.COLOR_GRAY2BGR)
        elif self.format == ImageFormat.GRAY16:
            # Convert 16-bit grayscale to 8-bit then to BGR
            gray8 = (self.data / 256).astype(np.uint8)
            bgr_data = cv2.cvtColor(gray8, cv2.COLOR_GRAY2BGR)
        else:
            raise ValueError(f"Unsupported format conversion from {self.format} to BGR")

        return self.__class__(
            data=bgr_data,
            format=ImageFormat.BGR,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def to_grayscale(self) -> "Image":
        """Convert image to grayscale."""
        if self.format == ImageFormat.GRAY:
            return self.copy()
        elif self.format == ImageFormat.GRAY16:
            return self.copy()
        elif self.format == ImageFormat.BGR:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_BGR2GRAY)
        elif self.format == ImageFormat.RGB:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_RGB2GRAY)
        elif self.format == ImageFormat.RGBA:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_RGBA2GRAY)
        elif self.format == ImageFormat.BGRA:
            gray_data = cv2.cvtColor(self.data, cv2.COLOR_BGRA2GRAY)
        else:
            raise ValueError(f"Unsupported format conversion from {self.format} to grayscale")

        return self.__class__(
            data=gray_data,
            format=ImageFormat.GRAY,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def resize(self, width: int, height: int, interpolation: int = cv2.INTER_LINEAR) -> "Image":
        """Resize the image to the specified dimensions."""
        resized_data = cv2.resize(self.data, (width, height), interpolation=interpolation)

        return self.__class__(
            data=resized_data,
            format=self.format,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def crop(self, x: int, y: int, width: int, height: int) -> "Image":
        """Crop the image to the specified region."""
        # Ensure crop region is within image bounds
        x = max(0, min(x, self.width))
        y = max(0, min(y, self.height))
        x2 = min(x + width, self.width)
        y2 = min(y + height, self.height)

        cropped_data = self.data[y:y2, x:x2]

        return self.__class__(
            data=cropped_data,
            format=self.format,
            frame_id=self.frame_id,
            ts=self.ts,
        )

    def frame_goodness(self, debug: bool = False):
        """
        Stateless per-frame 'goodness' in [0,1]. No rolling state.
        Lenient on cheap/noisy cameras: center-ROI, light smoothing, stricter gating.
        Returns:
          {
            "score": float in [0,1],
            "metrics": {...},
            "reasons": [..]
          }
        """

        # ----------------- helpers -----------------
        def _to_gray_f32(x):
            if x.ndim == 3 and x.shape[2] == 3:
                x = 0.299 * x[..., 0] + 0.587 * x[..., 1] + 0.114 * x[..., 2]
            return np.asarray(x, np.float32)

        def _resize_if_needed(x, target_w=960):
            h, w = x.shape[:2]
            if w > target_w:
                s = target_w / float(w)
                x = cv2.resize(x, (int(w * s), int(h * s)), interpolation=cv2.INTER_AREA)
            return x

        def _center_crop(x, frac=0.85):
            """Take central frac of the frame for glitch checks."""
            h, w = x.shape
            ch, cw = int(h * frac), int(w * frac)
            y0 = (h - ch) // 2
            x0 = (w - cw) // 2
            return x[y0 : y0 + ch, x0 : x0 + cw]

        def _sobel5_mag(x):
            sx = cv2.Sobel(x, cv2.CV_32F, 1, 0, ksize=5)
            sy = cv2.Sobel(x, cv2.CV_32F, 0, 1, ksize=5)
            return cv2.magnitude(sx, sy)

        def _lap_var(x):
            L = cv2.Laplacian(x, cv2.CV_32F, ksize=3)
            return float(L.var())

        def _noise_sigma_mad(x, grad_mag, flat_pct=40.0):
            gth = np.percentile(grad_mag, flat_pct)
            flat = grad_mag <= gth
            low = cv2.medianBlur(x, 3)
            resid = x - low
            r = resid[flat] if np.any(flat) else resid
            med = np.median(r)
            mad = np.median(np.abs(r - med))
            return float(max(1.4826 * mad, 0.5))  # small floor

        def _clip_tails(x):
            return float((x <= 2).mean()), float((x >= 253).mean())

        def _robust_step_z_of_means(img_for_rows_cols):
            r = img_for_rows_cols.mean(axis=1)
            c = img_for_rows_cols.mean(axis=0)

            # Light 1D smoothing by blurring the image beforehand (done outside)
            def z_of(v):
                d = np.abs(np.diff(v))
                if d.size == 0:
                    return 0.0
                m = np.median(d)
                mad = np.median(np.abs(d - m)) + 1e-6
                return float((np.max(np.abs(d - m))) / (1.4826 * mad + 1e-6))

            return z_of(r), z_of(c)

        def _stripe_score_db(gray_roi):
            """
            Robust banding score (0..~30 dB) on center ROI.
            Detrend + Hann + local-median background; drop very low bins.
            """

            def proj_score(v):
                v = v.astype(np.float32)
                N = v.size
                if N < 32:
                    return 0.0
                t = np.arange(N, dtype=np.float32)
                A = np.stack([t, np.ones_like(t)], axis=1)
                m, b = np.linalg.lstsq(A, v, rcond=None)[0]
                v = v - (m * t + b)
                w = np.hanning(N).astype(np.float32)
                V = np.fft.rfft(v * w)
                P = (V.real**2 + V.imag**2) / (np.dot(w, w) + 1e-12)
                low_bins = max(3, int(0.01 * P.size))
                if P.size <= low_bins + 2:
                    return 0.0
                P = P[low_bins:]
                k = max(9, (P.size // 64) * 2 + 1)
                pad = k // 2
                Ppad = np.pad(P, (pad, pad), mode="edge")
                B = np.empty_like(P)
                for i in range(P.size):
                    B[i] = np.median(Ppad[i : i + k])
                B = np.maximum(B, np.percentile(P, 10) * 0.5 + 1e-12)
                R = P / B
                idx = int(np.argmax(R))
                win = max(3, k // 3)
                mask = np.ones_like(R, dtype=bool)
                mask[max(0, idx - win) : min(R.size, idx + win + 1)] = False
                second = R[mask].max() if mask.any() else 1.0
                iso = R[idx] / (second + 1e-6)
                score_db = 10.0 * np.log10(R[idx] + 1e-12)
                if iso < 1.2:
                    score_db = 0.0
                return float(np.clip(score_db, 0.0, 30.0))

            r = gray_roi.mean(axis=1)
            c = gray_roi.mean(axis=0)
            # Use the 90th percentile of row/col, not the max (less trigger-happy)
            return float(np.percentile([proj_score(r), proj_score(c)], 90))

        def _blockiness8(gray_roi):
            dif = np.abs(np.diff(gray_roi, axis=1))
            if dif.shape[1] == 0:
                return 1.0
            cols = np.arange(dif.shape[1])
            grid = dif[:, (cols + 1) % 8 == 0]
            interior = dif[:, (cols + 1) % 8 != 0]
            gm = grid.mean() if grid.size else 0.0
            im = interior.mean() if interior.size else 1e-6
            return float(gm / (im + 1e-6))

        def _clip01(v):
            return float(max(0.0, min(1.0, v)))

        # ----------------- preprocess -----------------
        x = _to_gray_f32(self.to_grayscale().data)
        x = _resize_if_needed(x, 960)

        # ----------------- core metrics (full frame) -----------------
        gmag = _sobel5_mag(x)
        ten = float(gmag.mean())
        lapv = _lap_var(x)
        sigma = _noise_sigma_mad(x, gmag)
        snr = float(ten / (sigma * sigma + 1e-6))
        dclip, bclip = _clip_tails(x)
        dyn = float(x.max() - x.min())

        # ----------------- glitch metrics (center ROI, smoothed) -----------------
        roi = _center_crop(x, 0.85)
        roi_s = cv2.GaussianBlur(roi, (0, 0), 0.5)  # light smoothing to reduce noise-induced steps
        # Tight texture gate: only test if image likely has structure
        ten_log = np.log10(ten + 1.0)
        sharp_q = _clip01((ten_log - 1.7) / (3.7 - 1.7 + 1e-6))
        is_textured = (sharp_q > 0.45) and (lapv > 100.0) and (dyn >= 15.0)

        if is_textured:
            rowz, colz = _robust_step_z_of_means(roi_s)
            stripe_db = _stripe_score_db(roi_s)
            block = _blockiness8(roi_s)
        else:
            rowz = colz = stripe_db = 0.0
            block = 1.0

        # ----------------- stateless fusion -----------------
        # Sharpness base
        ten_lo, ten_hi = 1.7, 3.7
        sharp_q = _clip01((ten_log - ten_lo) / (ten_hi - ten_lo + 1e-6))

        # Penalties (lenient defaults)
        p_noise = _clip01((sigma - 12.0) / 24.0)  # 0 at 12, 1 at 36
        p_clip_dark = _clip01((dclip - 0.10) / 0.40)  # ignore small tails
        p_clip_bright = _clip01((bclip - 0.10) / 0.40)
        p_low_dyn = _clip01((12.0 - dyn) / 12.0)
        p_expo = max(p_clip_dark, p_clip_bright, p_low_dyn)

        # Glitch votes (lenient thresholds)
        step_bad = (rowz > 16.0) or (colz > 16.0)
        band_bad = stripe_db > 22.0  # dB
        block_bad = block > 2.6
        votes = int(step_bad) + int(band_bad) + int(block_bad) if is_textured else 0

        # Demote single-vote; require consensus for strong penalty
        p_glitch = 0.0
        if votes >= 2:
            p_glitch = 0.45
        elif votes == 1:
            p_glitch = 0.15  # tiny penalty; don't add "possible glitch" to reasons

        score = sharp_q - 0.25 * p_noise - 0.25 * p_expo - p_glitch
        score = _clip01(score)

        # ----------------- reasons (quiet on single-vote glitch) -----------------
        reasons = []
        if sharp_q < 0.30:
            reasons.append("blur/low-sharpness")
        if p_noise > 0.5:
            reasons.append("grain/SNR")
        if p_expo > 0.5:
            reasons.append("exposure/clipping")
        if votes >= 2:
            reasons.append("glitch")

        metrics = {
            "tenengrad_mean": ten,
            "ten_log": ten_log,
            "sharp_q": sharp_q,
            "laplacian_var": lapv,
            "noise_sigma": sigma,
            "snr": snr,
            "clip_dark": dclip,
            "clip_bright": bclip,
            "row_step_z": rowz,
            "col_step_z": colz,
            "stripe_db": stripe_db,
            "blockiness8": block,
            "dyn_range": dyn,
            "votes": votes,
        }

        report = {"score": score, "metrics": metrics, "reasons": reasons or ["ok"]}
        if debug:
            print(f"[frame_goodness lenient] {report}")
        return report

    def save(self, filepath: str) -> bool:
        """Save image to file."""
        # Convert to OpenCV format for saving
        cv_image = self.to_opencv()
        return cv2.imwrite(filepath, cv_image)

    def lcm_encode(self, frame_id: Optional[str] = None) -> LCMImage:
        """Convert to LCM Image message."""
        msg = LCMImage()

        # Header
        msg.header = Header()
        msg.header.seq = 0  # Initialize sequence number
        msg.header.frame_id = frame_id or self.frame_id

        # Set timestamp properly as Time object
        if self.ts is not None:
            msg.header.stamp.sec = int(self.ts)
            msg.header.stamp.nsec = int((self.ts - int(self.ts)) * 1e9)
        else:
            current_time = time.time()
            msg.header.stamp.sec = int(current_time)
            msg.header.stamp.nsec = int((current_time - int(current_time)) * 1e9)

        # Image properties
        msg.height = self.height
        msg.width = self.width
        msg.encoding = self._get_lcm_encoding()  # Convert format to LCM encoding
        msg.is_bigendian = False  # Use little endian
        msg.step = self._get_row_step()

        # Image data
        image_bytes = self.data.tobytes()
        msg.data_length = len(image_bytes)
        msg.data = image_bytes

        return msg.lcm_encode()

    @classmethod
    def lcm_decode(cls, data: bytes, **kwargs) -> "Image":
        """Create Image from LCM Image message."""
        # Parse encoding to determine format and data type
        msg = LCMImage.lcm_decode(data)
        format_info = cls._parse_encoding(msg.encoding)

        # Convert bytes back to numpy array
        data = np.frombuffer(msg.data, dtype=format_info["dtype"])

        # Reshape to image dimensions
        if format_info["channels"] == 1:
            data = data.reshape((msg.height, msg.width))
        else:
            data = data.reshape((msg.height, msg.width, format_info["channels"]))

        return cls(
            data=data,
            format=format_info["format"],
            frame_id=msg.header.frame_id if hasattr(msg, "header") else "",
            ts=msg.header.stamp.sec + msg.header.stamp.nsec / 1e9
            if hasattr(msg, "header") and msg.header.stamp.sec > 0
            else time.time(),
            **kwargs,
        )

    def _get_row_step(self) -> int:
        """Calculate row step (bytes per row)."""
        bytes_per_pixel = self._get_bytes_per_pixel()
        return self.width * bytes_per_pixel

    def _get_bytes_per_pixel(self) -> int:
        """Calculate bytes per pixel based on format and data type."""
        bytes_per_element = self.data.dtype.itemsize
        return self.channels * bytes_per_element

    def _get_lcm_encoding(self) -> str:
        """Get LCM encoding string from internal format and data type."""
        # Map internal format to LCM encoding based on format and dtype
        if self.format == ImageFormat.GRAY:
            if self.dtype == np.uint8:
                return "mono8"
            elif self.dtype == np.uint16:
                return "mono16"
        elif self.format == ImageFormat.GRAY16:
            return "mono16"
        elif self.format == ImageFormat.RGB:
            return "rgb8"
        elif self.format == ImageFormat.RGBA:
            return "rgba8"
        elif self.format == ImageFormat.BGR:
            return "bgr8"
        elif self.format == ImageFormat.BGRA:
            return "bgra8"
        elif self.format == ImageFormat.DEPTH:
            if self.dtype == np.float32:
                return "32FC1"
            elif self.dtype == np.float64:
                return "64FC1"

        raise ValueError(
            f"Cannot determine LCM encoding for format={self.format}, dtype={self.dtype}"
        )

    @staticmethod
    def _parse_encoding(encoding: str) -> dict:
        """Parse LCM image encoding string to determine format and data type."""
        # Standard encodings
        encoding_map = {
            "mono8": {"format": ImageFormat.GRAY, "dtype": np.uint8, "channels": 1},
            "mono16": {"format": ImageFormat.GRAY16, "dtype": np.uint16, "channels": 1},
            "rgb8": {"format": ImageFormat.RGB, "dtype": np.uint8, "channels": 3},
            "rgba8": {"format": ImageFormat.RGBA, "dtype": np.uint8, "channels": 4},
            "bgr8": {"format": ImageFormat.BGR, "dtype": np.uint8, "channels": 3},
            "bgra8": {"format": ImageFormat.BGRA, "dtype": np.uint8, "channels": 4},
            # Depth/float encodings
            "32FC1": {"format": ImageFormat.DEPTH, "dtype": np.float32, "channels": 1},
            "32FC3": {"format": ImageFormat.RGB, "dtype": np.float32, "channels": 3},
            "64FC1": {"format": ImageFormat.DEPTH, "dtype": np.float64, "channels": 1},
        }

        if encoding not in encoding_map:
            raise ValueError(f"Unsupported encoding: {encoding}")

        return encoding_map[encoding]

    def __repr__(self) -> str:
        """String representation."""
        return (
            f"Image(shape={self.shape}, format={self.format.value}, "
            f"dtype={self.dtype}, frame_id='{self.frame_id}', ts={self.ts})"
        )

    def __eq__(self, other) -> bool:
        """Check equality with another Image."""
        if not isinstance(other, Image):
            return False

        return (
            np.array_equal(self.data, other.data)
            and self.format == other.format
            and self.frame_id == other.frame_id
            and abs(self.ts - other.ts) < 1e-6
        )

    def __len__(self) -> int:
        """Return total number of pixels."""
        return self.height * self.width

    def agent_encode(self) -> str:
        """Encode image to base64 JPEG format for agent processing.

        Returns:
            Base64 encoded JPEG string suitable for LLM/agent consumption.
        """
        bgr_image = self.to_bgr()

        # Encode as JPEG
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 95]  # 95% quality
        success, buffer = cv2.imencode(".jpg", bgr_image.data, encode_param)

        if not success:
            raise ValueError("Failed to encode image as JPEG")

        # Convert to base64
        import base64

        jpeg_bytes = buffer.tobytes()
        base64_str = base64.b64encode(jpeg_bytes).decode("utf-8")

        return base64_str


def frame_goodness_window(target_frequency: float, source: Observable[Image]) -> Observable[Image]:
    window = TimestampedBufferCollection(1.0 / target_frequency)
    source.subscribe(window.add)

    thread_scheduler = ThreadPoolScheduler(max_workers=1)

    def find_best(*argv):
        if not window._items:
            return None
        return max(window._items, key=lambda x: x.frame_goodness()["score"])

    return rx.interval(1.0 / target_frequency).pipe(
        ops.observe_on(thread_scheduler), ops.map(find_best)
    )
