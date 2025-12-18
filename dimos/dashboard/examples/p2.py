#!/usr/bin/env python3
# Copyright 2025 Dimensional Inc.
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

import dataclasses
import logging
import os
from random import choices, random, sample
import sys
from typing import Optional

from reactivex.disposable import Disposable
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

default_rerun_grpc_port = 9876


@dataclasses.dataclass
class RerunInfo:
    logging_id: str = os.environ.get("RERUN_ID", "dimos_main_rerun")
    grpc_port: int = int(os.environ.get("RERUN_GRPC_PORT", default_rerun_grpc_port))
    server_memory_limit: str = os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "25%")
    url: str = os.environ.get(
        "RERUN_URL",
        f"rerun+http://localhost:{os.environ.get('RERUN_GRPC_PORT', default_rerun_grpc_port)!s}/proxy",
    )


rerun_info = RerunInfo()

import multiprocessing as mp


class RerunConnection:
    def __init__(self) -> None:
        self.init_id = mp.current_process().pid
        self.stream = rr.RecordingStream(rerun_info.logging_id, recording_id=rerun_info.logging_id)
        self.stream.connect_grpc(rerun_info.url)

    def log(self, msg: str, value, **kwargs) -> None:
        if self.init_id != mp.current_process().pid:
            raise Exception(
                """Looks like you are somehow using RerunConnection to log data to rerun. However, the process/thread where you init RerunConnection is different from where you are logging. A RerunConnection object needs to be created once per process/thread."""
            )

        self.stream.log(msg, value, **kwargs)


rc = RerunConnection()

import os
from pathlib import Path
import sys
import threading
import time

import yaml

file_path = Path("./dimos/dashboard/support/color_image.yaml")
if not file_path.exists():
    print(f"""[DataReplay] file {file_path} does not exist""", file=sys.stderr)
    exit(1)

with file_path.open("r", encoding="utf-8") as f:
    for line_number, line in enumerate(f):
        if not line.strip():
            continue
        try:
            parsed = yaml.unsafe_load(line) or []
        except Exception as error:
            print(f"""warning: line:{line_number} could not be parsed: {error}""")
            continue

        try:
            print("logging:", parsed)
            rc.log("/color_image", parsed[0].to_rerun(), strict=True)
        except Exception as error:
            print(f"error: {error}")
