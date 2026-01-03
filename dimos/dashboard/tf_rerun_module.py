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

"""TF Rerun Module - Automatically visualize all transforms in Rerun.

This module subscribes to the /tf LCM topic and logs ALL transforms
to Rerun, providing automatic visualization of the robot's TF tree.

Usage:
    # In blueprints:
    from dimos.dashboard.tf_rerun_module import tf_rerun

    def my_robot():
        return (
            robot_connection()
            + tf_rerun()  # Add TF visualization
            + other_modules()
        )
"""

from typing import Any

import rerun as rr

from dimos.core import Module, rpc
from dimos.core.global_config import GlobalConfig
from dimos.dashboard.rerun_init import connect_rerun
from dimos.msgs.tf2_msgs import TFMessage
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TFRerunModule(Module):
    """Subscribes to /tf LCM topic and logs all transforms to Rerun.

    This module automatically visualizes the TF tree in Rerun by:
    - Subscribing to the /tf LCM topic (captures ALL transforms in the system)
    - Logging each transform to its derived entity path (world/{child_frame_id})
    """

    _global_config: GlobalConfig
    _lcm: LCM | None = None
    _unsubscribe: Any = None

    def __init__(
        self,
        global_config: GlobalConfig | None = None,
        **kwargs: Any,
    ) -> None:
        """Initialize TFRerunModule.

        Args:
            global_config: Optional global configuration for viewer backend settings
            **kwargs: Additional arguments passed to parent Module
        """
        super().__init__(**kwargs)
        self._global_config = global_config or GlobalConfig()

    @rpc
    def start(self) -> None:
        """Start the TF visualization module."""
        super().start()

        # Only connect if Rerun backend is selected
        if self._global_config.viewer_backend.startswith("rerun"):
            connect_rerun(global_config=self._global_config)

            # Subscribe directly to LCM /tf topic (captures ALL transforms)
            self._lcm = LCM()
            self._lcm.start()
            topic = Topic("/tf", TFMessage)
            self._unsubscribe = self._lcm.subscribe(topic, self._on_tf_message)
            logger.info("TFRerunModule: subscribed to /tf, logging all transforms to Rerun")

    def _on_tf_message(self, msg: TFMessage, topic: Topic) -> None:
        """Log all transforms in TFMessage to Rerun.

        Args:
            msg: TFMessage containing transforms to visualize
            topic: The LCM topic (unused but required by callback signature)
        """
        for entity_path, transform in msg.to_rerun():  # type: ignore[no-untyped-call]
            rr.log(entity_path, transform)

    @rpc
    def stop(self) -> None:
        """Stop the TF visualization module and cleanup LCM subscription."""
        if self._unsubscribe:
            self._unsubscribe()
            self._unsubscribe = None

        if self._lcm:
            self._lcm.stop()
            self._lcm = None

        super().stop()


tf_rerun = TFRerunModule.blueprint
