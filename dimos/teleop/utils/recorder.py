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

"""Generic teleop stream recorder, shared across all teleop variants.

One recorder for quest, phone, and hosted teleop. It declares the *superset*
of teleop output ports; autoconnect wires whichever the composed blueprint
actually produces (VR controller poses + buttons for arm teleop, or
``cmd_vel_stamped`` for mobile-base/keyboard teleop). Ports the blueprint
doesn't drive simply stay empty in the DB.

Compose at the CLI::

    dimos run teleop-quest-xarm7  teleop-recorder
    dimos run teleop-hosted-go2   teleop-recorder
"""

from datetime import datetime
from pathlib import Path

from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.msgs.sensor_msgs.VideoStats import VideoStats
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.utils.report import generate_report
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TeleopRecorderConfig(RecorderConfig):
    # Default path is a stem — TeleopRecorder.start() appends a per-run
    # timestamp so successive runs don't clobber each other. Pass an absolute
    # path with ``.db`` to opt out of timestamping.
    db_path: str | Path = "recording_teleop.db"

    # If True (default), generate a transport-stats report next to the .db on
    # stop. Set False for a pure recording-only run (skips matplotlib import +
    # report formatting).
    generate_report: bool = True


class TeleopRecorder(Recorder):
    """Records teleop streams to a .db + (optionally) a transport-stats report.

    Superset of ports across arm (pose + buttons), mobile-base
    (``cmd_vel_stamped``), and hosted-teleop video stats. Unconnected ports stay
    empty in the DB. Each run lands in its own ``<stem>_<YYYYmmdd_HHMMSS>.db``
    so runs don't clobber. On stop, if ``generate_report=True``, also writes
    ``report.md`` + ``latency.png`` + ``jitter.png`` next to the .db.
    """

    left_controller_output: In[PoseStamped]
    right_controller_output: In[PoseStamped]
    buttons: In[Buttons]
    cmd_vel_stamped: In[TwistStamped]
    video_stats: In[VideoStats]
    config: TeleopRecorderConfig

    @rpc
    def start(self) -> None:
        # Append per-run timestamp to the stem so each run is its own file.
        base = Path(self.config.db_path)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.config.db_path = base.with_name(f"{base.stem}_{timestamp}{base.suffix}")
        # SqliteStore (sqlite3.connect) won't create the parent dir — ensure it.
        self.config.db_path.parent.mkdir(parents=True, exist_ok=True)
        super().start()

    @rpc
    def stop(self) -> None:
        # Snapshot the db_path before super().stop() closes the store — once
        # closed, we still want to point the report writer at the same file.
        db_path = Path(self.config.db_path) if self.config.generate_report else None
        super().stop()
        if db_path is not None:
            try:
                generate_report(db_path)
            except Exception:
                logger.exception("generate_report failed for %s", db_path)


__all__ = ["TeleopRecorder", "TeleopRecorderConfig"]
