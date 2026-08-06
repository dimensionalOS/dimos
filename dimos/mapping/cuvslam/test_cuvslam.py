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

"""Smoke tests for the cuVSLAM native module.

Split by what they need, so the cheap ones stay in the default suite:

* Declaration tests import the module and check its streams, config and CLI wiring.
  No GPU, no binary, no recording -- these catch a rename or a dropped stream, which is
  the failure this module actually suffered (it was wired into no blueprint and nothing
  noticed).
* ``self_hosted`` tests additionally run the built binary. They are skipped unless the
  nix build output is present, because the SDK is not redistributable.

Deliberately *not* covered: tracking accuracy. That is what the benchmark harness is for,
and cuVSLAM is not deterministic run to run, so an accuracy assertion here would be flaky.
"""

from __future__ import annotations

from pathlib import Path
import subprocess

import pytest

from dimos.core.native_module import NativeModule
from dimos.mapping.cuvslam.cuvslam import CuvslamConfig, CuvslamOdometry
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.sensor_msgs.Image import Image
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.msgs.tf2_msgs.TFMessage import TFMessage

BINARY = Path(__file__).parent / "result" / "bin" / "cuvslam_odometry"

needs_binary = pytest.mark.skipif(not BINARY.exists(), reason=f"native binary not built: {BINARY}")


def test_is_a_native_module() -> None:
    assert issubclass(CuvslamOdometry, NativeModule)


def test_declares_the_expected_streams() -> None:
    """A dropped or renamed stream silently unwires the module from its blueprint."""
    annotations = CuvslamOdometry.__annotations__
    expected = {
        "image_left": Image,
        "image_right": Image,
        "camera_info": CameraInfo,
        "imu": Imu,
        "odometry": Odometry,
        "landmarks": PointCloud2,
        "corrected_odometry": Odometry,
        "map_tf": Odometry,
        "tf": TFMessage,
    }
    for name, payload in expected.items():
        assert name in annotations, f"stream {name} disappeared"
        assert payload.__name__ in str(annotations[name]), (
            f"stream {name} changed payload type: {annotations[name]}"
        )


def test_config_is_constructible_with_defaults() -> None:
    config = CuvslamConfig()
    assert config is not None


def test_blueprint_factory_exists() -> None:
    assert hasattr(CuvslamOdometry, "blueprint")


@needs_binary
def test_binary_is_executable() -> None:
    import os

    assert os.access(BINARY, os.X_OK), f"{BINARY} is not executable"


@pytest.mark.self_hosted
@needs_binary
def test_binary_starts_and_reports_usage() -> None:
    """The binary should at least parse its own arguments and exit deliberately.

    Run with no arguments it must not hang and must not segfault; either a usage message
    or a clean non-zero exit is fine. A crash here means the SDK failed to load, which is
    the most common way this module breaks on a fresh machine.
    """
    completed = subprocess.run(
        [str(BINARY)], capture_output=True, text=True, timeout=60, check=False
    )
    assert completed.returncode >= 0, (
        f"binary died on signal {-completed.returncode} with no arguments; "
        "usually a missing or mismatched cuVSLAM SDK"
    )
