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

"""Show safe native buffer borrowing and report a reproducible local baseline."""

import gc
from statistics import median
from time import perf_counter_ns

from dimos_generated.sensor_msgs.msg import Image, PointCloud2, PointField
import numpy as np


def main() -> None:
    pixels = np.arange(640 * 480 * 3, dtype=np.uint8)
    points = np.arange(100_000 * 3, dtype=np.float32).view(np.uint8)
    fixtures = {
        "640x480 RGB image": Image(
            height=480, width=640, step=640 * 3, encoding="rgb8", data=pixels
        ),
        "100,000-point buffer": PointCloud2(
            height=1,
            width=100_000,
            point_step=12,
            row_step=1_200_000,
            data=points,
            fields=[
                PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
                for index, name in enumerate(("x", "y", "z"))
            ],
            is_dense=True,
        ),
    }
    for label, message in fixtures.items():
        view = message.data.view()
        assert np.shares_memory(view, message.data.view())
        assert not view.flags.writeable
        try:
            message.data.append(0)
        except BufferError:
            print(f"{label}: {view.nbytes:,} bytes, shared read-only view; resize rejected")
        else:
            raise AssertionError("Borrowed buffer allowed resizing")
        copied = message.data.copy()
        assert not np.shares_memory(view, copied)
        assert copied.flags.writeable
        del view
        durations = {"borrow": [], "copy": [], "encode": [], "decode": []}
        encoded = message.encode()
        for _ in range(30):
            start = perf_counter_ns()
            borrowed = message.data.view()
            durations["borrow"].append(perf_counter_ns() - start)
            del borrowed
            start = perf_counter_ns()
            copied = message.data.copy()
            durations["copy"].append(perf_counter_ns() - start)
            start = perf_counter_ns()
            encoded = message.encode()
            durations["encode"].append(perf_counter_ns() - start)
            start = perf_counter_ns()
            decoded = type(message).decode(encoded)
            durations["decode"].append(perf_counter_ns() - start)
        assert decoded.encode() == encoded
        print(
            "  local median (microseconds): "
            + ", ".join(f"{name}={median(values) / 1000:.1f}" for name, values in durations.items())
        )
    last_view = fixtures["640x480 RGB image"].data.view()
    fixtures.clear()
    gc.collect()
    np.testing.assert_array_equal(last_view, pixels)
    print("Borrowed image remains readable after its message variable is released.")


if __name__ == "__main__":
    main()
