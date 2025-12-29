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
from random import random

from reactivex.disposable import Disposable
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

from dimos.core import Module, rpc, In
from dimos.core.viz import VizMessageType, Viz
from dimos.dashboard.support.utils import (
    ensure_logger,
)
from dimos.msgs.sensor_msgs.Image import Image

# the types that are auto-rendered
Viz.viz_auto_log_types.add(Image)

# these should be args for the dashboard constructor, but its a pain to share data between modules
# so right now they're just a function of ENV vars
@dataclasses.dataclass
class RerunConfig:
    open_rerun: bool = False
    open_browser: bool = False
    serve_web: bool = False
    web_port: int = 5555
    application_id: str = "dimos_main_rerun"
    recording_id: str = f"dimos_dashboard_{random()}"
    grpc_port: int = 9876
    server_memory_limit: str = "0%"


# there can only be one dashboard at a time (e.g. global dashboard_config is alright)
class Dashboard(Module):
    viz_auto_log_types = tuple() # disable auto-viz for this module
    viz: In[VizMessageType]
    
    def __init__(
        self,
        *,
        rerun_config: RerunConfig = RerunConfig(),
        logger: logging.Logger | None = None,
    ) -> None:
        super().__init__()
        self.rerun_config = rerun_config
        self.logger = ensure_logger(logger, "dashboard")

    @rpc
    def start(self) -> None:
        self.logger.debug("[Dashboard] calling rr.init")
        print(f'''self.rerun_config.application_id = {self.rerun_config.application_id}''')
        rr.init(self.rerun_config.application_id, spawn=self.rerun_config.open_rerun, recording_id=self.rerun_config.recording_id)
        # get the rrd_url if it wasn't provided
        self.logger.debug("[Dashboard] starting rerun grpc")
        server_uri = rr.serve_grpc(
            grpc_port=self.rerun_config.grpc_port,
            server_memory_limit=self.rerun_config.server_memory_limit,
        )
        if self.rerun_config.serve_web:
            rr.serve_web_viewer(
                connect_to=server_uri,
                open_browser=self.rerun_config.config.open_browser,
                web_port=self.rerun_config.config.web_port,
            )
            self.logger.info(f"Rerun web viewer serving on port {self.rerun_config.config.web_port}")
        
        def _on_viz(msg) -> None:
            try:
                value, address, metadata = msg
                kwargs_for_to_rerun = metadata.get("to_rerun", {})
                kwargs_for_rerun_log = metadata.get("rerun_log", {})
                rr_value = value.to_rerun(**kwargs_for_to_rerun)
                rr.log(address, rr_value, **kwargs_for_rerun_log)
            except Exception as error:
                self.logger.error(f"[Dashboard] Failed to receive viz message. Might be missing .to_rerun() method on data type: {error}")
        
        self._disposables.add(Disposable(self.viz.subscribe(_on_viz)))
        
    @rr.shutdown_at_exit
    def stop(self) -> None:
        self.logger.info("Stopping dashboard server")
