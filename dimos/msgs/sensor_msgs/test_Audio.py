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

import time
from typing import Any

import numpy as np
import pytest

from dimos.msgs.sensor_msgs.Audio import Audio


def test_lcm_encode_decode() -> None:
    """Test that Audio messages can be encoded to LCM bytes and decoded back correctly."""
    # Create a dummy audio message
    sample_rate = 16000
    channels = 1
    duration = 1.0  # seconds
    t = np.linspace(0, duration, int(sample_rate * duration), endpoint=False)
    # Generate a 440Hz sine wave
    data = 0.5 * np.sin(2 * np.pi * 440 * t)
    data = data.astype(np.float32)

    msg = Audio(data=data, sample_rate=sample_rate, channels=channels)

    # Encode
    encoded = msg.lcm_encode()
    assert isinstance(encoded, bytes)
    # Header is 20 bytes (double + 3 ints), payload is 16000 * 4 bytes
    expected_len = 20 + len(data) * 4
    assert len(encoded) == expected_len

    # Decode
    decoded = Audio.lcm_decode(encoded)

    # Verify
    assert decoded.sample_rate == sample_rate
    assert decoded.channels == channels
    assert abs(decoded.ts - msg.ts) < 1e-6
    assert len(decoded.data) == len(data)
    assert decoded.data.dtype == np.float32

    # Check data integrity
    np.testing.assert_allclose(decoded.data, data, rtol=1e-6, atol=1e-6)


def test_lcm_encode_decode_empty() -> None:
    """Test encoding and decoding an empty Audio message."""
    msg = Audio(data=np.array([], dtype=np.float32))
    encoded = msg.lcm_encode()
    decoded = Audio.lcm_decode(encoded)

    assert len(decoded.data) == 0
    assert decoded.data.shape == (0,)


def test_lcm_decode_invalid_length() -> None:
    """Test that decoding raises ValueError if data length doesn't match header."""
    msg = Audio(data=np.zeros(100, dtype=np.float32))
    encoded = msg.lcm_encode()

    # Truncate the encoded data
    truncated = encoded[:-4]
    with pytest.raises(ValueError, match="Decoded data length"):
        Audio.lcm_decode(truncated)


def test_benchmark_serialization(capsys: pytest.CaptureFixture) -> None:
    """Benchmark the encoding and decoding of standard audio chunks."""
    # Benchmark 1s of 16kHz audio (standard chunk size)
    sample_rate = 16000
    channels = 1
    data = np.random.uniform(-1, 1, sample_rate).astype(np.float32)
    msg = Audio(data=data, sample_rate=sample_rate, channels=channels)

    iterations = 1000

    # Warmup
    for _ in range(10):
        e = msg.lcm_encode()
        Audio.lcm_decode(e)

    # Encode benchmark
    start_time = time.time()
    for _ in range(iterations):
        encoded = msg.lcm_encode()
    end_time = time.time()
    encode_avg = (end_time - start_time) / iterations

    # Decode benchmark
    start_time = time.time()
    for _ in range(iterations):
        _ = Audio.lcm_decode(encoded)
    end_time = time.time()
    decode_avg = (end_time - start_time) / iterations

    # Print results to stdout (use -s to see this with pytest)
    with capsys.disabled():
        print(f"\nBenchmark Results (1s audio @ 16kHz, {iterations} iters):")
        print(f"Encode avg: {encode_avg * 1000:.4f} ms")
        print(f"Decode avg: {decode_avg * 1000:.4f} ms")

    # Performance expectations (loose bounds to avoid flakiness on CI)
    # Typically this should be sub-millisecond or low single-digit ms
    assert encode_avg < 0.005  # < 5ms
    assert decode_avg < 0.005  # < 5ms


if __name__ == "__main__":
    # Fallback to manual execution if pytest is not used
    try:
        test_lcm_encode_decode()
        test_lcm_encode_decode_empty()
        print("Tests passed (manual run)")
    except Exception as e:
        print(f"Tests failed: {e}")
        exit(1)
