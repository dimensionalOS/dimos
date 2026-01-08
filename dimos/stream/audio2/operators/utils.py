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

"""Volume calculation utilities for audio processing."""

import numpy as np


def calculate_rms_volume(audio_data: np.ndarray) -> float:
    """Calculate RMS (Root Mean Square) volume of audio data.

    Args:
        audio_data: Audio data as numpy array (mono or multi-channel)

    Returns:
        RMS volume as a float between 0.0 and 1.0
    """
    # For multi-channel audio, calculate RMS across all channels
    if len(audio_data.shape) > 1 and audio_data.shape[1] > 1:
        # Flatten all channels
        audio_data = audio_data.flatten()

    # Calculate RMS
    rms = np.sqrt(np.mean(np.square(audio_data)))

    # For int16 data, normalize to [0, 1]
    if audio_data.dtype == np.int16:
        rms = rms / 32768.0
    elif audio_data.dtype == np.int32:
        rms = rms / 2147483648.0

    return float(rms)


def calculate_peak_volume(audio_data: np.ndarray) -> float:
    """Calculate peak volume of audio data.

    Args:
        audio_data: Audio data as numpy array (mono or multi-channel)

    Returns:
        Peak volume as a float between 0.0 and 1.0
    """
    # For multi-channel audio, find max across all channels
    if len(audio_data.shape) > 1 and audio_data.shape[1] > 1:
        # Flatten all channels
        audio_data = audio_data.flatten()

    # Find absolute peak value
    peak = np.max(np.abs(audio_data))

    # For int16 data, normalize to [0, 1]
    if audio_data.dtype == np.int16:
        peak = peak / 32768.0
    elif audio_data.dtype == np.int32:
        peak = peak / 2147483648.0

    return float(peak)
