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

"""Optional audit trail for agent-facing message encodings."""

from __future__ import annotations

import base64
import fcntl
import hashlib
import json
import os
from pathlib import Path
import time
from typing import Any

AGENT_ACTIVITY_DIR_ENV = "DIMOS_AGENT_ACTIVITY_DIR"


def record_agent_encode(message_type: str, encoded: Any) -> None:
    """Record an encoding when an eval agent enabled activity auditing."""
    configured = os.environ.get(AGENT_ACTIVITY_DIR_ENV)
    if not configured:
        return

    root = Path(configured)
    media_dir = root / "media"

    def externalize(value: Any) -> Any:
        if isinstance(value, dict):
            return {key: externalize(child) for key, child in value.items()}
        if isinstance(value, list):
            return [externalize(child) for child in value]
        if not isinstance(value, str) or not value.startswith("data:"):
            return value
        header, separator, payload = value.partition(",")
        if not separator or ";base64" not in header:
            return value
        data = base64.b64decode(payload)
        digest = hashlib.sha256(data).hexdigest()
        media_type = header.removeprefix("data:").split(";", 1)[0]
        suffix = {"image/jpeg": ".jpg", "image/png": ".png"}.get(media_type, ".bin")
        media_dir.mkdir(parents=True, exist_ok=True)
        media_path = media_dir / f"{digest}{suffix}"
        try:
            with media_path.open("xb") as output:
                output.write(data)
        except FileExistsError:
            pass
        return {
            "activity_media": media_path.name,
            "media_type": media_type,
            "bytes": len(data),
            "sha256": digest,
        }

    root.mkdir(parents=True, exist_ok=True)
    event = {
        "event": "agent_encode",
        "wall_time_s": time.time(),
        "pid": os.getpid(),
        "message_type": message_type,
        "output": externalize(encoded),
    }
    line = json.dumps(event, separators=(",", ":")) + "\n"
    with (root / "events.jsonl").open("a") as output:
        fcntl.flock(output, fcntl.LOCK_EX)
        output.write(line)
        output.flush()
        fcntl.flock(output, fcntl.LOCK_UN)
