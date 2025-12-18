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

import logging
import os
from random import choices, random, sample
import sys
from typing import Optional

from reactivex.disposable import Disposable
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

from dimos.core import In, Module, rpc
from dimos.dashboard.server import env_bool, start_dashboard_server_thread


# there can only be one dashboard at a time (e.g. global dashboard_config is alright)
class Dashboard(Module):
    """
    Internals Note:
        The Dashboard handles rendering the terminals (Zellij) and the viewer (Rerun).
        The Layout (elsewhere) handles the layout of rerun.
        The start_dashboard_server_thread mostly handles the logic for Zellij, with only an iframe for rerun.
    """

    # the following just get passed directly to start_dashboard_server_thread
    port: int = int(os.environ.get("DASHBOARD_PORT", "4000"))
    dashboard_host: str = os.environ.get("DASHBOARD_HOST", "localhost")
    terminal_commands: dict[str, str] | None = None
    https_enabled: bool = env_bool("HTTPS_ENABLED", False)
    zellij_host: str = os.environ.get("ZELLIJ_HOST", "127.0.0.1")
    zellij_port: int = int(os.environ.get("ZELLIJ_PORT", "8083"))
    zellij_token: str | None = os.environ.get("ZELLIJ_TOKEN")
    zellij_url: str | None = None
    zellij_session_name: str | None = "dimos-dashboard"
    https_key_path: str | None = os.environ.get("HTTPS_KEY_PATH")
    https_cert_path: str | None = os.environ.get("HTTPS_CERT_PATH")
    logger: logging.Logger | None = None
    rerun_id: str = os.environ.get("RERUN_ID", "dimos_main_rerun")
    rerun_grpc_port: int = os.environ.get("RERUN_GRPC_PORT", 9876)
    rerun_server_memory_limit: str = os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "25%")
    rrd_url: str | None = None

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__()
        # TODO: the following rerun aspects can only be set via ENV vars (for now)
        kwargs = {**kwargs, "rerun_id": self.rerun_id, "rerun_grpc_port": self.rerun_grpc_port}
        self.__dict__.update(kwargs)

    @rpc
    def start(self, **kwargs) -> None:
        # there's basically 3 parts to rerun
        # 1. some kind of python init that does local message aggregation
        # 2. the actual (separate process) grpc message aggregator
        # 3. the viewer/renderer
        # init starts part 1 (needed before rr.log or rr.send_blueprint)
        # we manually start the gprc here (part 2)
        # we serve our own viewer via a webserver (part 3) which is why spawn=False (we don't want it to spawn its own viewer, although we could)
        rr.init(self.rerun_id, spawn=False, strict=True, recording_id=self.rerun_id)
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
        self.rrd_url = self.rrd_url or rr.serve_grpc(
            grpc_port=self.rerun_grpc_port,
            default_blueprint=default_blueprint,
            server_memory_limit=self.rerun_server_memory_limit,
        )
        print(f"""[Dashboard] rrd_url = {self.rrd_url}""")
        thread = start_dashboard_server_thread(**self.__dict__)

        @self._disposables.add
        @Disposable
        def _cleanup_dashboard_thread():
            # Attempt to let the server thread shut down gracefully when the module stops.
            if thread.is_alive():
                thread.join(timeout=1.0)
