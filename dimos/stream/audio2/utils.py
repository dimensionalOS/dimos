#!/usr/bin/env python3
# Copyright 2025-2026 Dimensional Inc.
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

"""Utility functions for audio pipeline operations."""

import time
from typing import Optional, Tuple

import gi
import numpy as np

gi.require_version("Gst", "1.0")
from gi.repository import Gst

from dimos.stream.audio2.types import (
    AudioEvent,
    AudioFormat,
    AudioSpec,
    CompressedAudioEvent,
    RawAudioEvent,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger("dimos.stream.audio2.utils")

# Map AudioFormat to numpy dtype
NUMPY_DTYPE_MAP = {
    AudioFormat.PCM_S16LE: np.int16,
    AudioFormat.PCM_S32LE: np.int32,
    AudioFormat.PCM_F32LE: np.float32,
    AudioFormat.PCM_F64LE: np.float64,
}


def get_numpy_dtype_for_format(format: AudioFormat):
    """Get numpy dtype for an AudioFormat.

    Args:
        format: Audio format enum

    Returns:
        Corresponding numpy dtype

    Raises:
        ValueError: If format is not a raw PCM format
    """
    if not format.is_raw:
        raise ValueError(f"Cannot get numpy dtype for compressed format: {format}")

    dtype = NUMPY_DTYPE_MAP.get(format)
    if dtype is None:
        raise ValueError(f"No numpy dtype mapping for format: {format}")

    return dtype


def create_audio_caps(spec: AudioSpec) -> str:
    """Create a GStreamer caps string from an AudioSpec.

    Args:
        spec: Audio specification

    Returns:
        GStreamer caps string
    """
    return spec.to_gst_caps_string()


def parse_caps_to_spec(caps: Gst.Caps) -> Tuple[Optional[int], Optional[int], Optional[str]]:
    """Parse GStreamer caps to extract audio parameters.

    Args:
        caps: GStreamer caps object

    Returns:
        Tuple of (sample_rate, channels, format_str)
    """
    if not caps or caps.is_empty():
        return None, None, None

    structure = caps.get_structure(0)
    if not structure:
        return None, None, None

    sample_rate = structure.get_value("rate")
    channels = structure.get_value("channels")
    format_str = structure.get_value("format")

    return sample_rate, channels, format_str


def buffer_to_raw_audio_event(
    buffer: Gst.Buffer,
    sample_rate: int,
    channels: int,
    format: AudioFormat,
    timestamp: Optional[float] = None,
) -> RawAudioEvent:
    """Convert a GStreamer buffer to a RawAudioEvent.

    Args:
        buffer: GStreamer buffer containing audio data
        sample_rate: Sample rate in Hz
        channels: Number of audio channels
        format: Audio format (must be raw PCM)
        timestamp: Optional timestamp, defaults to current time

    Returns:
        RawAudioEvent with the audio data

    Raises:
        ValueError: If format is not raw or buffer mapping fails
    """
    if not format.is_raw:
        raise ValueError(f"Cannot create RawAudioEvent from compressed format: {format}")

    # Map buffer to access data
    success, map_info = buffer.map(Gst.MapFlags.READ)
    if not success:
        raise ValueError("Failed to map GStreamer buffer")

    try:
        # Get numpy dtype
        dtype = get_numpy_dtype_for_format(format)

        # Convert buffer data to numpy array
        data = np.frombuffer(map_info.data, dtype=dtype)

        # Reshape for multi-channel audio
        if channels > 1:
            data = data.reshape(-1, channels)

        # Use provided timestamp or current time
        if timestamp is None:
            timestamp = time.time()

        return RawAudioEvent(
            data=data.copy(),  # Copy to ensure data persists after unmap
            sample_rate=sample_rate,
            channels=channels,
            timestamp=timestamp,
        )

    finally:
        buffer.unmap(map_info)


def buffer_to_compressed_audio_event(
    buffer: Gst.Buffer,
    sample_rate: int,
    channels: int,
    format: AudioFormat,
    timestamp: Optional[float] = None,
) -> CompressedAudioEvent:
    """Convert a GStreamer buffer to a CompressedAudioEvent.

    Args:
        buffer: GStreamer buffer containing compressed audio
        sample_rate: Sample rate in Hz
        channels: Number of audio channels
        format: Audio format (must be compressed)
        timestamp: Optional timestamp, defaults to current time

    Returns:
        CompressedAudioEvent with the audio data

    Raises:
        ValueError: If format is not compressed or buffer mapping fails
    """
    if not format.is_compressed:
        raise ValueError(f"Cannot create CompressedAudioEvent from raw format: {format}")

    # Map buffer to access data
    success, map_info = buffer.map(Gst.MapFlags.READ)
    if not success:
        raise ValueError("Failed to map GStreamer buffer")

    try:
        # Use provided timestamp or current time
        if timestamp is None:
            timestamp = time.time()

        # Calculate duration if available
        duration = buffer.duration / Gst.SECOND if buffer.duration != Gst.CLOCK_TIME_NONE else None

        return CompressedAudioEvent(
            data=bytes(map_info.data),  # Convert to bytes
            format_type=format,
            sample_rate=sample_rate,
            channels=channels,
            timestamp=timestamp,
            duration=duration,
        )

    finally:
        buffer.unmap(map_info)


def buffer_to_audio_event(
    buffer: Gst.Buffer,
    spec: AudioSpec,
    detected_rate: Optional[int] = None,
    detected_channels: Optional[int] = None,
    timestamp: Optional[float] = None,
) -> AudioEvent:
    """Convert a GStreamer buffer to appropriate AudioEvent type.

    Args:
        buffer: GStreamer buffer
        spec: Expected audio specification
        detected_rate: Detected sample rate from caps (overrides spec if provided)
        detected_channels: Detected channels from caps (overrides spec if provided)
        timestamp: Optional timestamp

    Returns:
        Either RawAudioEvent or CompressedAudioEvent based on format
    """
    # Use detected values if available, otherwise fall back to spec
    sample_rate = detected_rate or spec.sample_rate or 44100
    channels = detected_channels or spec.channels or 1

    if spec.format.is_raw:
        return buffer_to_raw_audio_event(
            buffer=buffer,
            sample_rate=sample_rate,
            channels=channels,
            format=spec.format,
            timestamp=timestamp,
        )
    else:
        return buffer_to_compressed_audio_event(
            buffer=buffer,
            sample_rate=sample_rate,
            channels=channels,
            format=spec.format,
            timestamp=timestamp,
        )


def validate_pipeline_element(pipeline: Gst.Pipeline, element_name: str) -> Gst.Element:
    """Get and validate a pipeline element by name.

    Args:
        pipeline: GStreamer pipeline
        element_name: Name of the element to retrieve

    Returns:
        The element if found

    Raises:
        RuntimeError: If element not found
    """
    element = pipeline.get_by_name(element_name)
    if not element:
        raise RuntimeError(f"Failed to get '{element_name}' from pipeline")
    return element


def apply_gstreamer_properties(
    element: Gst.Element,
    properties: dict,
    skip_properties: Optional[set] = None,
    element_name: str = "element",
) -> None:
    """Apply properties to a GStreamer element with error handling.

    Args:
        element: GStreamer element to configure
        properties: Dictionary of property name to value
        skip_properties: Set of property names to skip (already set)
        element_name: Name of element for logging
    """
    skip = skip_properties or set()

    for prop, value in properties.items():
        if prop in skip:
            continue
        try:
            element.set_property(prop, value)
            logger.debug(f"Set {element_name} property {prop}={value}")
        except Exception as e:
            logger.warning(f"Failed to set {element_name} property {prop}: {e}")
