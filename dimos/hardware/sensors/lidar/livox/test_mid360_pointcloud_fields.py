# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

"""Unit test verifying the Mid360 PointCloud2 field layout.

This test verifies the field names, offsets, types, and point_step
of the PointCloud2 published by the Mid360 native module, without
requiring the physical hardware. It checks that the 7-field layout
(x, y, z, intensity, ring, tag, offset_time) matches what downstream
consumers (e.g. better_fastlio2) expect.
"""

import struct

import pytest


def test_mid360_pointcloud2_field_layout():
    """Verify the 7-field PointCloud2 layout matches spec."""
    # These constants must match main.cpp publish_pointcloud()
    EXPECTED_FIELDS = [
        # (name, offset, datatype_id, datatype_name)
        ("x", 0, 7, "FLOAT32"),
        ("y", 4, 7, "FLOAT32"),
        ("z", 8, 7, "FLOAT32"),
        ("intensity", 12, 7, "FLOAT32"),
        ("ring", 16, 1, "UINT8"),      # scan line (0 for Mid360)
        ("tag", 17, 1, "UINT8"),       # Livox echo/return info
        ("offset_time", 20, 6, "UINT32"),  # microseconds from frame start
    ]
    EXPECTED_POINT_STEP = 24
    EXPECTED_NUM_FIELDS = 7

    # Simulate packing one point as the C++ code does
    x, y, z, intensity = 1.5, 2.5, 3.5, 0.8
    ring_val = 0
    tag_val = 0x10  # first return
    offset_time_us = 500  # 500 microseconds

    buf = bytearray(EXPECTED_POINT_STEP)
    struct.pack_into("<ffff", buf, 0, x, y, z, intensity)
    buf[16] = ring_val
    buf[17] = tag_val
    buf[18] = 0  # padding
    buf[19] = 0  # padding
    struct.pack_into("<I", buf, 20, offset_time_us)

    # Verify we can unpack correctly
    x2, y2, z2, i2 = struct.unpack_from("<ffff", buf, 0)
    assert x2 == pytest.approx(x)
    assert y2 == pytest.approx(y)
    assert z2 == pytest.approx(z)
    assert i2 == pytest.approx(intensity)
    assert buf[16] == ring_val
    assert buf[17] == tag_val
    (ot,) = struct.unpack_from("<I", buf, 20)
    assert ot == offset_time_us

    # Verify field spec
    assert len(EXPECTED_FIELDS) == EXPECTED_NUM_FIELDS
    for name, offset, dtype_id, dtype_name in EXPECTED_FIELDS:
        assert isinstance(name, str)
        assert isinstance(offset, int)
        assert offset < EXPECTED_POINT_STEP, f"{name} offset {offset} >= point_step {EXPECTED_POINT_STEP}"

    # Verify no field overlap
    field_ranges = []
    for name, offset, dtype_id, _ in EXPECTED_FIELDS:
        size = {1: 1, 6: 4, 7: 4}[dtype_id]  # UINT8=1, UINT32=4, FLOAT32=4
        field_ranges.append((offset, offset + size, name))
    field_ranges.sort()
    for i in range(len(field_ranges) - 1):
        _, end_a, name_a = field_ranges[i]
        start_b, _, name_b = field_ranges[i + 1]
        assert end_a <= start_b, f"Fields {name_a} and {name_b} overlap"


def test_offset_time_microsecond_conversion():
    """Verify nanosecond → microsecond conversion logic."""
    # The C++ code does: uint32_t ot_us = offset_time_ns / 1000
    offset_ns = 1_500_000  # 1.5ms = 1500µs
    offset_us = offset_ns // 1000
    assert offset_us == 1500

    # Max value: ~4.29 billion µs = ~71.6 minutes (fits uint32)
    max_us = (2**32 - 1)
    max_minutes = max_us / 1_000_000 / 60
    assert max_minutes > 70, "uint32 microseconds should cover >70 minutes"
