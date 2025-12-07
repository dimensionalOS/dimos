#!/usr/bin/env python3
import os
import sys
import threading

from typing import Optional
import logging
import rerun as rr  # pip install rerun-sdk
from reactivex.disposable import Disposable
from dimos.dashboard.server import start_dashboard_server_thread, env_bool
from dimos.dashboard.rerun.layouts_base import Layout 
from dimos.dashboard.rerun.layouts import LayoutAllTabs 
from dimos.core import Module, In, rpc
from dimos.dashboard.rerun.layouts import LayoutAllTabs 

# there can only be one dashboard at a time (e.g. global dashboard_config is alright)
dashboard_started = False
dashboard_started_lock = threading.Lock()
class Dashboard(Module):
    """
    Internals Note:
        The Dashboard handles rendering the terminals (Zellij) and the viewer (Rerun). 
        The Layout (elsewhere) handles the layout of rerun.
        The start_dashboard_server_thread mostly handles the logic for Zellij, with only an iframe for rerun.
    """
    
    layout: Layout = LayoutAllTabs(collapse_panels=False)
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
    rrd_url: Optional[str] = None
    
    def __init__(self, *args, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__()
        self.__dict__.update(kwargs)

    @rpc
    def start(self) -> None:
        global dashboard_started
        with dashboard_started_lock:
            if dashboard_started:
                print(f'''Dashboard already started, cannot start again.''', file=sys.stderr)
                return
            dashboard_started = True
        
        # start the rerun viewer
        rr.init("rerun_main", spawn=False)
        rr.send_blueprint(self.layout.rerun_blueprint)
        # get the rrd_url if it wasn't provided
        self.rrd_url = self.rrd_url or rr.serve_grpc()  # e.g. "rerun+http://127.0.0.1:9876/proxy"
        # TODO: add cleanup (disposal of thread)
        thread = start_dashboard_server_thread(**self.__dict__)
        
        @self._disposables.add
        @Disposable
        def _cleanup_dashboard_thread():
            # Attempt to let the server thread shut down gracefully when the module stops.
            global dashboard_started
            with dashboard_started_lock:
                dashboard_started = False
            if thread.is_alive():
                thread.join(timeout=1.0)
