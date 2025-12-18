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
from dimos.dashboard.server import start_dashboard_server_thread


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


# there's basically 3 parts to rerun
# 1. some kind of python init that does local message aggregation
# 2. the actual (separate process) grpc message aggregator
# 3. the viewer/renderer
# init starts part 1 (needed before rr.log or rr.send_blueprint)
# we manually start the gprc here (part 2)
# we serve our own viewer via a webserver (part 3) which is why spawn=False (we don't want it to spawn its own viewer, although we could)
print("""[Dashboard] calling rr.init""")
rr.init(rerun_info.logging_id, spawn=False, recording_id=rerun_info.logging_id)
# send (basically) an empty blueprint to at least show the user that something is happening
default_blueprint = rrb.Blueprint(
    rrb.Tabs(
        rrb.Spatial3DView(
            name="Spatial3D",
            origin="/",
            line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
        ),
    )
)
print("[Dashboard] sending empty blueprint")
rr.send_blueprint(default_blueprint)
# get the rrd_url if it wasn't provided
print("[Dashboard] starting rerun grpc if needed")
if not os.environ.get("RERUN_URL", None):
    rr.serve_grpc(
        grpc_port=rerun_info.grpc_port,
        default_blueprint=default_blueprint,
        server_memory_limit=rerun_info.server_memory_limit,
    )
thread = start_dashboard_server_thread(
    **{
        "auto_open": True,
        "terminal_commands": {"agent-spy": "htop", "lcm-spy": "dimos lcmspy"},
        "worker": 1,
    },
    rrd_url=rerun_info.url,
    keep_alive=True,
)

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


thread.join(timeout=3)
