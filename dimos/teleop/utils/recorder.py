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

from dimos.constants import STATE_DIR
from dimos.core.core import rpc
from dimos.core.stream import In
from dimos.memory2.module import Recorder, RecorderConfig
from dimos.memory2.store.sqlite import SqliteStore
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.geometry_msgs.TwistStamped import TwistStamped
from dimos.teleop.quest.quest_types import Buttons
from dimos.teleop.utils.report import generate_report
from dimos.teleop.utils.video_stats import VideoStats
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


class TeleopRecorderConfig(RecorderConfig):
    db_path: str | Path = STATE_DIR / "teleop_recordings" / "recording_teleop.db"
    generate_report: bool = True


class TeleopRecorder(Recorder):
    """Records teleop streams to a .db + (optionally) a transport-stats report.

    Superset of ports across arm (pose + buttons), mobile-base
    (``cmd_vel_stamped``), and hosted-teleop stats (``video_stats``,
    ``robot_telemetry``). Unconnected ports stay empty in the DB. Each run lands
    in its own ``<stem>_<YYYYmmdd_HHMMSS>.db``
    so runs don't clobber. On stop, if ``generate_report=True``, also writes
    ``report.json`` next to the .db.
    """

    left_controller_output: In[PoseStamped]
    right_controller_output: In[PoseStamped]
    teleop_buttons: In[Buttons]
    cmd_vel_stamped: In[TwistStamped]
    video_stats: In[VideoStats]
    robot_telemetry: In[bytes]  # hosted-teleop periodic telemetry (cmd stats + soc + state)
    config: TeleopRecorderConfig
    # Per-run path (stem + timestamp), held here so we don't mutate config.
    _db_path: Path | None = None

    @rpc
    def start(self) -> None:
        base = Path(self.config.db_path)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self._db_path = base.with_name(f"{base.stem}_{timestamp}{base.suffix}")
        self._db_path.parent.mkdir(parents=True, exist_ok=True)
        # Open the store ourselves so the base Recorder records into our path.
        self._store = self.register_disposable(SqliteStore(path=str(self._db_path)))
        self._store.start()
        super().start()

    @rpc
    def stop(self) -> None:
        db_path = self._db_path if self.config.generate_report else None
        super().stop()
        if db_path is not None:
            try:
                generate_report(db_path)
            except Exception:
                logger.exception("generate_report failed for %s", db_path)
