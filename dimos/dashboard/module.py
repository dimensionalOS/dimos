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
import multiprocessing as mp
import os
from pathlib import Path
import tempfile

from reactivex.disposable import Disposable
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

from dimos.core import Module, rpc
from dimos.dashboard.server import env_bool, start_dashboard_server_thread
from dimos.dashboard.support.utils import make_constants

config = make_constants(
    dict(
        default_rerun_grpc_port=9876,
        dashboard_started_lock=tempfile.NamedTemporaryFile(delete=False).name,
    )
)

try:
    os.unlink(config["dashboard_started_lock"])
except Exception:
    pass


@dataclasses.dataclass
class RerunInfo:
    logging_id: str = os.environ.get("RERUN_ID", "dimos_main_rerun")
    grpc_port: int = int(os.environ.get("RERUN_GRPC_PORT", config["default_rerun_grpc_port"]))
    server_memory_limit: str = os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "0%")
    url: str = os.environ.get(
        "RERUN_URL",
        f"rerun+http://127.0.0.1:{os.environ.get('RERUN_GRPC_PORT', config['default_rerun_grpc_port'])!s}/proxy",
    )


rerun_info = RerunInfo()


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

    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__()
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
        print("""[Dashboard] calling rr.init""")
        rr.init(rerun_info.logging_id, spawn=False, recording_id=rerun_info.logging_id)
        # send (basically) an empty blueprint to at least show the user that something is happening
        default_blueprint = self.__dict__.get(
            "rerun_default_blueprint",
            rrb.Blueprint(
                rrb.Tabs(
                    rrb.Horizontal(
                        rrb.Spatial3DView(
                            name="WorldView",
                            origin="/",
                            line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
                        ),
                        rrb.Spatial2DView(
                            name="ImageView1",
                            origin="/",
                        ),
                    ),
                )
            ),
        )
        print("[Dashboard] sending empty blueprint")
        rr.send_blueprint(default_blueprint)
        # get the rrd_url if it wasn't provided
        print("[Dashboard] starting rerun grpc if needed")
        if not os.environ.get("RERUN_URL", None):
            try:
                rr.serve_grpc(
                    grpc_port=rerun_info.grpc_port,
                    default_blueprint=default_blueprint,
                    server_memory_limit=rerun_info.server_memory_limit,
                )
            except Exception as error:
                self.logger.error(f"Failed to start Rerun GRPC server: {error}")

        thread = start_dashboard_server_thread(
            **self.__dict__, keep_alive=True, rrd_url=rerun_info.url
        )
        # set the lock
        with open(config["dashboard_started_lock"], "w+") as the_file:
            the_file.write("1")

        @self._disposables.add
        @Disposable
        def _cleanup_dashboard_thread():
            try:
                os.unlink(config["dashboard_started_lock"])
            except FileNotFoundError:
                pass
            # Attempt to let the server thread shut down gracefully when the module stops.
            if thread.is_alive():
                thread.join(timeout=1.0)


class RerunConnection:
    def __init__(self) -> None:
        self._init_id = None
        self.stream = None

    def log(self, msg: str, value, **kwargs) -> None:
        if not self.stream:
            if not Path(config["dashboard_started_lock"]).exists():
                return
            self.stream = rr.RecordingStream(
                rerun_info.logging_id, recording_id=rerun_info.logging_id
            )
            self.stream.connect_grpc(rerun_info.url)
            self._init_id = mp.current_process().pid

        if self._init_id != mp.current_process().pid:
            raise Exception(
                """Looks like you are somehow using RerunConnection to log data to rerun. However, the process/thread where you init RerunConnection is different from where you are logging. A RerunConnection object needs to be created once per process/thread."""
            )

        self.stream.log(msg, value, **kwargs)
