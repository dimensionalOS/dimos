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

"""UrdfManager in Rust: the same ports and config, so it drops into a blueprint in its place."""

from __future__ import annotations

from typing import Any

from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.files.File import File
from dimos.msgs.sensor_msgs.JointState import JointState
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.robot.urdf_manager import UrdfManagerConfig


class UrdfManagerNativeConfig(NativeModuleConfig, UrdfManagerConfig):
    source_dir: str | None = "dimos/robot/urdf_manager_native/rust"
    executable: str = "result/bin/urdf_manager"
    build_command: str | None = "nix build -L path:."
    stdin_config: bool = True

    def to_cli_args(self) -> list[str]:
        """Nothing: the Rust module reads its config from stdin, and a URDF does not belong on a command line."""
        return []

    def to_config_dict(self) -> dict[str, Any]:
        """Exactly the Rust module's Config: the model goes over materialized, as the URDF UrdfManager would load."""
        return {
            "urdf_xml": self.model.load().xml,
            "publish_joint_tf": self.publish_joint_tf,
            "joint_tf_hz": self.joint_tf_hz,
            "unchanged_tf_hz": self.unchanged_tf_hz,
        }


class UrdfManagerNative(NativeModule):
    """Poses the URDF from the joint state onto tf, and serves the URDF and the files it names to renderers."""

    config: UrdfManagerNativeConfig

    # IO: the joints it poses are published on it.
    tf: IO[TFMessage]
    coordinator_joint_state: In[JointState]
    # A request for a *.urdf gets the URDF; one for a file the URDF names (mesh, texture, splat, ...) by its full path
    # gets that file. Either goes out on file_info named as requested.
    render_file_request: In[String]
    file_info: Out[File]
