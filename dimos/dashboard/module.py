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
import tempfile

from reactivex.disposable import Disposable
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

from dimos.core import Module, rpc, In
from dimos.core.viz import VizMessageType, Viz
from dimos.dashboard.support.utils import (
    FileBasedBoolean,
    ensure_logger,
    make_constant_across_workers,
)

DASHBOARD_CONSTANTS = make_constant_across_workers(
    dict(
        default_rerun_grpc_port=9876,
        dashboard_started_signal=tempfile.NamedTemporaryFile(delete=False).name,
    )
)

FileBasedBoolean(DASHBOARD_CONSTANTS["dashboard_started_signal"]).set(False)


# these should be args for the dashboard constructor, but its a pain to share data between modules
# so right now they're just a function of ENV vars
@dataclasses.dataclass
class RerunInfo:
    logging_id: str = os.environ.get("RERUN_ID", "dimos_main_rerun")
    grpc_port: int = int(
        os.environ.get("RERUN_GRPC_PORT", DASHBOARD_CONSTANTS["default_rerun_grpc_port"])
    )
    server_memory_limit: str = os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "0%")
    url: str = os.environ.get(
        "RERUN_URL",
        f"rerun+http://127.0.0.1:{os.environ.get('RERUN_GRPC_PORT', DASHBOARD_CONSTANTS['default_rerun_grpc_port'])!s}/proxy",
    )


rerun_info = RerunInfo()

from dimos.msgs.sensor_msgs.Image import Image
Viz.viz_auto_log_types.add(Image) # add image as a viz type

# there can only be one dashboard at a time (e.g. global dashboard_config is alright)
class Dashboard(Module):
    viz_auto_log_types = tuple() # disable auto-viz for this module
    viz: In[VizMessageType]
    
    def __init__(
        self,
        *,
        logger: logging.Logger | None = None,
        open_rerun: bool = False,
    ) -> None:
        super().__init__()
        self.logger = ensure_logger(logger, "dashboard")
        self.open_rerun = open_rerun

    @rpc
    def start(self) -> None:
        dashboard_started = FileBasedBoolean(DASHBOARD_CONSTANTS["dashboard_started_signal"])
        self.logger.debug("[Dashboard] calling rr.init")
        rr.init(rerun_info.logging_id, spawn=self.open_rerun, recording_id=rerun_info.logging_id)
        # send (basically) an empty blueprint to at least show the user that something is happening
        default_blueprint = self.__dict__.get(
            "rerun_default_blueprint",
            rrb.Blueprint(
                rrb.Tabs(
                    rrb.Horizontal(
                        rrb.Spatial3DView(
                            name="WorldView1",
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
        self.logger.debug("[Dashboard] sending empty blueprint")
        rr.send_blueprint(default_blueprint)
        # get the rrd_url if it wasn't provided
        self.logger.debug("[Dashboard] starting rerun grpc if needed")
        if not os.environ.get("RERUN_URL", None):
            try:
                rr.serve_grpc(
                    grpc_port=rerun_info.grpc_port,
                    default_blueprint=default_blueprint,
                    server_memory_limit=rerun_info.server_memory_limit,
                )
            except Exception as error:
                self.logger.error(f"Failed to start Rerun GRPC server: {error}")
        
        def _on_viz(msg) -> None:
            try:
                rr.init(rerun_info.logging_id, spawn=False, recording_id=rerun_info.logging_id)
                value, address, metadata = msg
                print(f"[Dashboard] logging message: {value}")
                rr.log(address, value.to_rerun(**metadata.get("to_rerun", {})), metadata.get("rerun_log", False))
                print(f"[Dashboard] logged message: {value}")
            except Exception as error:
                self.logger.error(f"[Dashboard] Failed to receive viz message. Might be missing .to_rerun() method on data type: {error}")
        
        self._disposables.add(Disposable(self.viz.subscribe(_on_viz)))
        # set the lock
        dashboard_started.set(True)

        @self._disposables.add
        @Disposable
        def _cleanup_dashboard_thread():
            dashboard_started.clean()

    def stop(self) -> None:
        self.logger.info("Stopping dashboard server")


class RerunConnection:
    def __init__(self, logger: logging.Logger | None = None) -> None:
        self._logger = ensure_logger(logger, "RerunConnection")
        self._init_id = mp.current_process().pid
        self._dropped_logs = 0
        self.stream = None

    def __pickle__(self):
        raise Exception(
            f"""{self.__class__.__name__} is not picklable. Do not save it, and do not pass it between workers/processes. Create a fresh RerunConnection object within each worker/process/thread."""
        )

    def log(self, msg: str, value, **kwargs) -> None:
        # if not self.stream:
        #     if not FileBasedBoolean(DASHBOARD_CONSTANTS["dashboard_started_signal"]).get():
        #         if self._dropped_logs == 0:
        #             self._logger.info(
        #                 "[RerunConnection] rerun grpc not started yet, dropping rerun log. Will notify when this changes"
        #             )
        #         self._dropped_logs += 1
        #         return
        #     if self._dropped_logs > 1:
        #         self._dropped_logs = 0
        #         self._logger.info("[RerunConnection] rerun grpc connection found")

        #     self.stream = rr.RecordingStream(
        #         rerun_info.logging_id, recording_id=rerun_info.logging_id
        #     )
        #     self.stream.connect_grpc(rerun_info.url)

        # if self._init_id != mp.current_process().pid:
        #     raise Exception(
        #         """Looks like you are somehow using RerunConnection to log data to rerun. However, the process/thread where you init RerunConnection is different from where you are logging. A RerunConnection object needs to be created once per process/thread."""
        #     )

        # self.stream.log(msg, value, **kwargs)
        pass
