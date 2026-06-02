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

"""
Usage::

    from dimos.hardware.sensors.lidar.fastlio2_rust.module import FastLio2Rust
    from dimos.hardware.sensors.lidar.livox.module import Mid360
    from dimos.core.coordination.blueprints import autoconnect

    from dimos.core.coordination.module_coordinator import ModuleCoordinator
    ModuleCoordinator.build(autoconnect(
        Mid360.blueprint(host_ip="192.168.1.5"),
        FastLio2Rust.blueprint(),
    )).loop()
"""

from __future__ import annotations

from pathlib import Path
import time
from typing import TYPE_CHECKING, Annotated, Any

from pydantic.experimental.pipeline import validate_as
from reactivex.disposable import Disposable

from dimos.core.core import rpc
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import In, Out
from dimos.msgs.geometry_msgs.Quaternion import Quaternion
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.nav_msgs.Odometry import Odometry
from dimos.msgs.sensor_msgs.Imu import Imu
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2
from dimos.navigation.nav_stack.frames import FRAME_BODY, FRAME_ODOM
from dimos.spec import perception

# Reuse the shared FAST-LIO YAML configs that the C++ FastLio2 module reads.
_CONFIG_DIR = Path(__file__).resolve().parent.parent / "fastlio2" / "config"


class FastLio2RustConfig(NativeModuleConfig):
    cwd: str | None = "rust"
    executable: str = "result/bin/fastlio2_rust_native"
    build_command: str | None = "nix build .#fastlio2_rust_native"
    stdin_config: bool = True

    debug: bool = False

    # Output message frames.
    frame_id: str = FRAME_ODOM
    child_frame_id: str = FRAME_BODY

    # VERY IMPORTANT
    # this is used to prevent catestrophic divergence
    # go2 dog should set this to 3.1 m/s
    # it needs some buffer room (dog can't actually move that fast)
    # but other than that buffer room, tigher=less chance of catestrophic divergence
    max_velocity: float = 100  # ~200 mph

    # FAST-LIO downsample voxel sizes (the Rust analog of the C++ FastLio2
    # voxel_size / map_voxel_size). None keeps whatever the YAML sets.
    filter_size_surf: float | None = None
    filter_size_map: float | None = None

    # Output publish gating (like the C++ map_freq / odom_freq):
    # < 0 disabled, 0 every scan (default), > 0 throttled to N Hz.
    map_freq: float = 0.0
    odom_freq: float = 0.0

    # publish odom-frame lidar points
    registered_scan_freq: float = 0.0

    # Standard FAST-LIO YAML config (shared with the C++ FastLio2 module).
    # Relative paths resolve against fastlio2/config/. The fastlio_rs crate
    # parses this YAML itself (Config::from_yaml_path) into the pipeline params;
    # we just hand it the resolved path as ``config_path``.
    config: Annotated[
        Path, validate_as(...).transform(lambda p: p if p.is_absolute() else _CONFIG_DIR / p)
    ] = Path("mid360.yaml")

    def to_config_dict(self) -> dict[str, Any]:
        # frame_id lives on the base NativeModuleConfig, so the default
        # to_config_dict() drops it; the Rust binary still needs it.
        config = super().to_config_dict()
        # Hand the binary the YAML path (the crate reads it), not the Path obj.
        config.pop("config", None)
        config["frame_id"] = self.frame_id
        # The transform only resolves explicitly-passed paths; resolve the
        # default (relative) path here too.
        config_path = self.config if self.config.is_absolute() else _CONFIG_DIR / self.config
        config["config_path"] = str(config_path.resolve())
        return config


class FastLio2Rust(NativeModule, perception.Odometry):
    config: FastLio2RustConfig

    lidar: In[PointCloud2]
    imu: In[Imu]
    odometry: Out[Odometry]
    global_map: Out[PointCloud2]
    registered_scan: Out[PointCloud2]

    @rpc
    def start(self) -> None:
        super().start()
        self.register_disposable(
            Disposable(self.odometry.transport.subscribe(self._on_odom_for_tf, self.odometry))
        )

    def _on_odom_for_tf(self, msg: Odometry) -> None:
        self.tf.publish(
            Transform(
                frame_id=self.config.frame_id,
                child_frame_id=self.config.child_frame_id,
                translation=Vector3(
                    msg.pose.position.x,
                    msg.pose.position.y,
                    msg.pose.position.z,
                ),
                rotation=Quaternion(
                    msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w,
                ),
                ts=msg.ts or time.time(),
            )
        )


# Verify protocol port compliance (mypy will flag missing ports)
if TYPE_CHECKING:
    FastLio2Rust()
