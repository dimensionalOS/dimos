#!/usr/bin/env python3
import os
import sys
from random import random, sample, choices

from typing import Optional
import logging
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
from reactivex.disposable import Disposable
from dimos.dashboard.server import start_dashboard_server_thread, env_bool
from dimos.core import Module, In, rpc

id_test = random()

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
    terminal_commands: Optional[dict[str, str]] = None
    https_enabled: bool = env_bool("HTTPS_ENABLED", False)
    zellij_host: str = os.environ.get("ZELLIJ_HOST", "127.0.0.1")
    zellij_port: int = int(os.environ.get("ZELLIJ_PORT", "8083"))
    zellij_token: Optional[str] = os.environ.get("ZELLIJ_TOKEN")
    zellij_url: Optional[str] = None
    zellij_session_name: Optional[str] = "dimos-dashboard"
    https_key_path: Optional[str] = os.environ.get("HTTPS_KEY_PATH")
    https_cert_path: Optional[str] = os.environ.get("HTTPS_CERT_PATH")
    logger: Optional[logging.Logger] = None
    rerun_grpc_port: int = os.environ.get("RERUN_GRPC_PORT", 9876)
    rerun_server_memory_limit: str = os.environ.get("RERUN_SERVER_MEMORY_LIMIT", "25%")
    rrd_url: Optional[str] = None
    
    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__()
        self.__dict__.update(kwargs)

    @rpc
    def start(self, **kwargs) -> None:
        file_path = './dashboard_run_once_hack.ignore.txt'
        try:
            # if file exits, then we've already run this once
            with open(file_path, 'r') as f:
                output = f.read()
            print(f'''output = {output}''')
            return
        except:
            print("#")
            print("# Starting Dashboard")
            print("#")
            with open(file_path, 'w') as the_file:
                the_file.write(str("exists"))
        
        # there's basically 3 parts to rerun
            # 1. some kind of python init that does local message aggregation
            # 2. the actual (separate process) grpc message aggregator
            # 3. the viewer/renderer
        # init starts part 1 (needed before rr.log or rr.send_blueprint)
        # we manually start the gprc here (part 2)
        # we serve our own viewer via a webserver (part 3) which is why spawn=False (we don't want it to spawn its own viewer, although we could)
        rr.init("rerun_main", spawn=False, strict=True)
        # send an empty blueprint to get the initial state
        default_blueprint = rrb.Blueprint(
            rrb.Tabs(
                rrb.Spatial3DView(
                    name="Spatial3D",
                    origin="/spatial3d",
                    line_grid=rrb.LineGrid3D(spacing=1.0, stroke_width=1.0),
                ),
                rrb.Spatial2DView(
                    name="Spatial2D",
                    origin="/color_image",
                ),
            )
        )
        rr.send_blueprint(default_blueprint)
        
        # get the rrd_url if it wasn't provided
        print("[Dashboard] serving grpc")
        self.rrd_url = self.rrd_url or rr.serve_grpc(
            grpc_port=self.rerun_grpc_port,
            default_blueprint=default_blueprint,
            server_memory_limit=self.rerun_server_memory_limit,
        )
        print(f'''[Dashboard] starting dashboard server with url = {self.rrd_url}''')
        thread = start_dashboard_server_thread(**self.__dict__)
        @self._disposables.add
        @Disposable
        def _cleanup_dashboard_thread():
            # Attempt to let the server thread shut down gracefully when the module stops.
            if thread.is_alive():
                thread.join(timeout=1.0)
        
