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

from __future__ import annotations

import os
from pathlib import Path
import re
import sys
import time
from typing import Annotated

from fastapi import Path as PathParam, Query
from fastapi.responses import (
    StreamingResponse,
)
from pydantic import BeforeValidator

LIST_TTL_S = 60.0
ASSETS = Path(__file__).parents[1] / "assets"
VIEW_DIR = ASSETS / "blueprint_view"
VIEW_FILE = re.compile("[a-z_]+\\.(js|css)")
INTROSPECT_TTL_S = 600.0


def _started_from() -> tuple[str | None, int | None]:
    """The program this gateway runs (the python running it) and its modification time (Unix s), as it started."""
    try:
        return (sys.executable, int(os.stat(sys.executable).st_mtime))
    except (OSError, ValueError):
        return (sys.executable or None, None)


STARTED_FROM = _started_from()
STARTED_AT = int(time.time())


class EventStreamResponse(StreamingResponse):
    media_type = "text/event-stream"


FreshQuery = Annotated[
    bool,
    BeforeValidator(lambda value: True if value == "" else value),
    Query(description="check again now instead of answering from the cache"),
]
BlueprintParam = Annotated[
    str,
    PathParam(description="blueprint name, e.g. unitree-go2-basic", examples=["unitree-go2-basic"]),
]
UploadIdParam = Annotated[str, PathParam(description="upload id, e.g. u3", examples=["u3"])]


class ApiError(Exception):
    def __init__(self, status: int, message: str) -> None:
        super().__init__(message)
        self.status = status
