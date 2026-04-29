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

"""Hardware round-trip test for multiple Arduino message types.

Sends Bool, Int32, Vector3, and Quaternion messages to a physical Arduino
and verifies the echoed values. Includes float64->float32 precision checks
for types with double fields (Vector3, Quaternion).

Requires:
    - Arduino Uno connected via USB
    - nix on PATH

Run:
    uv run python dimos/hardware/arduino/examples/arduino_multi_echo/test_multi_echo_hardware.py
"""

from __future__ import annotations

import struct
import sys
import threading
import time
from typing import Any

from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.module_coordinator import ModuleCoordinator
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.hardware.arduino.examples.arduino_multi_echo.module import MultiEcho
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.std_msgs.Bool import Bool
from dimos.msgs.std_msgs.Int32 import Int32
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# Float64 values chosen to exercise precision loss on AVR (double = float32).
# These have more than 7 significant digits, so truncation to float32 is
# guaranteed but should remain within float32 representable range.
F64_TEST_VALUES = [
    3.141592653589793,  # pi
    2.718281828459045,  # e
    -0.123456789012345,  # negative, many decimals
    1.0000001192092896,  # just above 1.0 + float32 epsilon
    100000.015625,  # large integer part + fractional
    1e-7,  # small magnitude
]

BOOL_TESTS = [True, False, True]
INT32_TESTS = [0, 1, -1, 42, 2147483647, -2147483648]


class TestHarnessConfig(ModuleConfig):
    pass


class TestHarness(Module):
    """Sends test messages and collects echoes via internal thread."""

    config: TestHarnessConfig

    bool_out: Out[Bool]
    bool_in: In[Bool]
    int32_out: Out[Int32]
    int32_in: In[Int32]
    vec3_out: Out[Vector3]
    vec3_in: In[Vector3]
    quat_out: Out[Quaternion]
    quat_in: In[Quaternion]

    _bool_echoes: list[bool]
    _int32_echoes: list[int]
    _vec3_echoes: list[tuple[float, float, float]]
    _quat_echoes: list[tuple[float, float, float, float]]
    _lock: Any  # threading.Lock
    _done: bool

    @rpc
    def start(self) -> None:
        super().start()
        self._bool_echoes = []
        self._int32_echoes = []
        self._vec3_echoes = []
        self._quat_echoes = []
        self._lock = threading.Lock()
        self._done = False

        self.bool_in.subscribe(self._on_bool)
        self.int32_in.subscribe(self._on_int32)
        self.vec3_in.subscribe(self._on_vec3)
        self.quat_in.subscribe(self._on_quat)

        # Start sending in a background thread
        threading.Thread(target=self._send_all, daemon=True).start()

    def _on_bool(self, msg: Bool) -> None:
        with self._lock:
            self._bool_echoes.append(msg.data)

    def _on_int32(self, msg: Int32) -> None:
        with self._lock:
            self._int32_echoes.append(msg.data)

    def _on_vec3(self, msg: Vector3) -> None:
        with self._lock:
            self._vec3_echoes.append((msg.x, msg.y, msg.z))

    def _on_quat(self, msg: Quaternion) -> None:
        with self._lock:
            self._quat_echoes.append((msg.x, msg.y, msg.z, msg.w))

    def _send_all(self) -> None:
        """Send all test messages with small gaps."""
        # Wait for bridge to be ready
        time.sleep(2)

        for val in BOOL_TESTS:
            self.bool_out.publish(Bool(data=val))
            time.sleep(0.2)

        for val in INT32_TESTS:
            self.int32_out.publish(Int32(data=val))
            time.sleep(0.2)

        vec3_tests = [
            Vector3(F64_TEST_VALUES[0], F64_TEST_VALUES[1], F64_TEST_VALUES[2]),
            Vector3(F64_TEST_VALUES[3], F64_TEST_VALUES[4], F64_TEST_VALUES[5]),
            Vector3(0.0, 0.0, 0.0),
        ]
        for vec in vec3_tests:
            self.vec3_out.publish(vec)
            time.sleep(0.2)

        quat_tests = [
            Quaternion(
                F64_TEST_VALUES[0],
                F64_TEST_VALUES[1],
                F64_TEST_VALUES[2],
                F64_TEST_VALUES[3],
            ),
            Quaternion(0.0, 0.0, 0.0, 1.0),
            Quaternion(
                F64_TEST_VALUES[4],
                F64_TEST_VALUES[5],
                -F64_TEST_VALUES[0],
                F64_TEST_VALUES[1],
            ),
        ]
        for q in quat_tests:
            self.quat_out.publish(q)
            time.sleep(0.2)

        time.sleep(2)
        with self._lock:
            self._done = True
        logger.info("All test messages sent")

    @rpc
    def get_results(self) -> dict[str, Any]:
        """Return collected echo data for validation."""
        with self._lock:
            return {
                "done": self._done,
                "bool_echoes": list(self._bool_echoes),
                "int32_echoes": list(self._int32_echoes),
                "vec3_echoes": list(self._vec3_echoes),
                "quat_echoes": list(self._quat_echoes),
            }


def float64_to_float32(val: float) -> float:
    """Round-trip a float64 through float32 representation."""
    return struct.unpack("f", struct.pack("f", val))[0]


def validate_results(results: dict[str, Any]) -> bool:
    passed = True

    # Bool checks
    print(f"\n{'=' * 60}")
    print("BOOL ECHO TEST")
    print(f"{'=' * 60}")
    bool_echoes = results["bool_echoes"]
    if len(bool_echoes) >= len(BOOL_TESTS):
        for sent, got in zip(BOOL_TESTS, bool_echoes, strict=False):
            status = "OK" if sent == got else "FAIL"
            print(f"  [{status}] sent={sent} got={got}")
            if sent != got:
                passed = False
    else:
        print(f"  [FAIL] Expected {len(BOOL_TESTS)} echoes, got {len(bool_echoes)}")
        passed = False

    # Int32 checks
    print(f"\n{'=' * 60}")
    print("INT32 ECHO TEST")
    print(f"{'=' * 60}")
    int32_echoes = results["int32_echoes"]
    if len(int32_echoes) >= len(INT32_TESTS):
        for sent, got in zip(INT32_TESTS, int32_echoes, strict=False):
            status = "OK" if sent == got else "FAIL"
            print(f"  [{status}] sent={sent} got={got}")
            if sent != got:
                passed = False
    else:
        print(f"  [FAIL] Expected {len(INT32_TESTS)} echoes, got {len(int32_echoes)}")
        passed = False

    # Vector3 checks
    print(f"\n{'=' * 60}")
    print("VECTOR3 ECHO TEST (float64 -> float32 precision)")
    print(f"{'=' * 60}")
    vec3_tests = [
        (F64_TEST_VALUES[0], F64_TEST_VALUES[1], F64_TEST_VALUES[2]),
        (F64_TEST_VALUES[3], F64_TEST_VALUES[4], F64_TEST_VALUES[5]),
        (0.0, 0.0, 0.0),
    ]
    vec3_echoes = results["vec3_echoes"]
    if len(vec3_echoes) >= len(vec3_tests):
        for i, (sent, got) in enumerate(zip(vec3_tests, vec3_echoes, strict=False)):
            for axis, (s, g) in zip("xyz", zip(sent, got, strict=False), strict=False):
                expected_f32 = float64_to_float32(s)
                abs_err = abs(g - expected_f32)
                tol = max(abs(expected_f32) * 1.2e-7, 1e-45)
                status = "OK" if abs_err <= tol else "FAIL"
                precision_lost = abs(s - g)
                print(
                    f"  [{status}] vec[{i}].{axis}: "
                    f"sent_f64={s:.15g}  expected_f32={expected_f32:.8g}  "
                    f"got={g:.8g}  err_vs_f32={abs_err:.2e}  "
                    f"total_precision_loss={precision_lost:.2e}"
                )
                if abs_err > tol:
                    passed = False
    else:
        print(f"  [FAIL] Expected {len(vec3_tests)} echoes, got {len(vec3_echoes)}")
        passed = False

    # Quaternion checks
    print(f"\n{'=' * 60}")
    print("QUATERNION ECHO TEST (float64 -> float32 precision)")
    print(f"{'=' * 60}")
    quat_tests = [
        (F64_TEST_VALUES[0], F64_TEST_VALUES[1], F64_TEST_VALUES[2], F64_TEST_VALUES[3]),
        (0.0, 0.0, 0.0, 1.0),
        (F64_TEST_VALUES[4], F64_TEST_VALUES[5], -F64_TEST_VALUES[0], F64_TEST_VALUES[1]),
    ]
    quat_echoes = results["quat_echoes"]
    if len(quat_echoes) >= len(quat_tests):
        for i, (sent, got) in enumerate(zip(quat_tests, quat_echoes, strict=False)):
            for axis, (s, g) in zip("xyzw", zip(sent, got, strict=False), strict=False):
                expected_f32 = float64_to_float32(s)
                abs_err = abs(g - expected_f32)
                tol = max(abs(expected_f32) * 1.2e-7, 1e-45)
                status = "OK" if abs_err <= tol else "FAIL"
                precision_lost = abs(s - g)
                print(
                    f"  [{status}] quat[{i}].{axis}: "
                    f"sent_f64={s:.15g}  expected_f32={expected_f32:.8g}  "
                    f"got={g:.8g}  err_vs_f32={abs_err:.2e}  "
                    f"total_precision_loss={precision_lost:.2e}"
                )
                if abs_err > tol:
                    passed = False
    else:
        print(f"  [FAIL] Expected {len(quat_tests)} echoes, got {len(quat_echoes)}")
        passed = False

    print(f"\n{'=' * 60}")
    if passed:
        print("ALL TESTS PASSED")
    else:
        print("SOME TESTS FAILED")
    print(f"{'=' * 60}")
    return passed


def main() -> None:
    bp = (
        autoconnect(
            TestHarness.blueprint(),
            MultiEcho.blueprint(virtual=False),
        )
        .remappings(
            [
                (TestHarness, "bool_out", "bool_cmd"),
                (MultiEcho, "bool_in", "bool_cmd"),
                (MultiEcho, "bool_out", "bool_echo"),
                (TestHarness, "bool_in", "bool_echo"),
                (TestHarness, "int32_out", "int32_cmd"),
                (MultiEcho, "int32_in", "int32_cmd"),
                (MultiEcho, "int32_out", "int32_echo"),
                (TestHarness, "int32_in", "int32_echo"),
                (TestHarness, "vec3_out", "vec3_cmd"),
                (MultiEcho, "vec3_in", "vec3_cmd"),
                (MultiEcho, "vec3_out", "vec3_echo"),
                (TestHarness, "vec3_in", "vec3_echo"),
                (TestHarness, "quat_out", "quat_cmd"),
                (MultiEcho, "quat_in", "quat_cmd"),
                (MultiEcho, "quat_out", "quat_echo"),
                (TestHarness, "quat_in", "quat_echo"),
            ]
        )
        .global_config(n_workers=2)
    )

    coord = ModuleCoordinator.build(bp)

    harness = coord.get_instance(TestHarness)
    assert harness is not None, "TestHarness not found in coordinator"

    # Poll until the harness says it's done
    deadline = time.time() + 60
    results: dict[str, Any] = {}
    while time.time() < deadline:
        results = harness.get_results()
        if results.get("done"):
            break
        time.sleep(1)

    coord.stop()

    if not results.get("done"):
        print("FAIL: Test harness did not finish within 30 seconds")
        sys.exit(1)

    ok = validate_results(results)
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
