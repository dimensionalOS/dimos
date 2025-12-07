#!/usr/bin/env python3
import os

from typing import Optional
import logging
import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
from reactivex.disposable import Disposable
from dimos.dashboard.server import start_dashboard_server_thread, env_bool
from dimos.dashboard.rerun.layouts_base import Layout 
from dimos.dashboard.rerun.layouts import LayoutAllTabs 
from dimos.core import Module, In, rpc

# there can only be one dashboard at a time (e.g. global dashboard_config is alright)
dashboard_started = False
dashboard_config = {}
rerun_config = {
    "layout": LayoutAllTabs(),
}
def Dashboard(
    *,
    layout: Layout,
    # the following just get passed directly to start_dashboard_server_thread
    auto_open: bool = False,
    port: int = int(os.environ.get("DASHBOARD_PORT", "4000")),
    dashboard_host: str = os.environ.get("DASHBOARD_HOST", "localhost"),
    terminal_commands: Optional[dict[str, str]] = None,
    https_enabled: bool = env_bool("HTTPS_ENABLED", False),
    zellij_host: str = os.environ.get("ZELLIJ_HOST", "127.0.0.1"),
    zellij_port: int = int(os.environ.get("ZELLIJ_PORT", "8083")),
    zellij_token: Optional[str] = os.environ.get("ZELLIJ_TOKEN"),
    zellij_url: Optional[str] = None,
    zellij_session_name: Optional[str] = "dimos-dashboard",
    https_key_path: Optional[str] = os.environ.get("HTTPS_KEY_PATH"),
    https_cert_path: Optional[str] = os.environ.get("HTTPS_CERT_PATH"),
    logger: Optional[logging.Logger] = None,
    rrd_url: Optional[str] = None,
) -> None:
    """
    Note: this exists as a wrapper to make the following possible:
        blueprint = (
            autoconnect(
                CameraModule.blueprint(),
                Dashboard(
                    port=9090,
                ).blueprint()
            )
            .global_config(n_dask_workers=1)
        )
    """
    rerun_config.update(dict(
        layout=layout,
    ))
    dashboard_config.update(dict(
        auto_open=auto_open,
        port=port,
        dashboard_host=dashboard_host,
        terminal_commands=terminal_commands,
        https_enabled=https_enabled,
        zellij_host=zellij_host,
        zellij_port=zellij_port,
        zellij_token=zellij_token,
        zellij_url=zellij_url,
        zellij_session_name=zellij_session_name,
        https_key_path=https_key_path,
        https_cert_path=https_cert_path,
        logger=logger,
        rrd_url=rrd_url,
    ))
    return DashboardModule

class DashboardModule(Module):
    """
    Internals Note:
        The Dashboard handles rendering the terminals (Zellij) and the viewer (Rerun). 
        The Layout (elsewhere) handles the layout of rerun.
        The dimos_dashboard_func mostly handles the logic for Zellij, with only an iframe for rerun.
    """

    @rpc
    def start(self) -> None:
        # FIXME: because this is an RPC, stuff like "global dashboard_started" and dashboard_config is not going to work as-intended in prod
        global dashboard_started
        if dashboard_started:
            raise Exception(f'''Dashboard already started, cannot start again.''')
        dashboard_started = True
        
        # start the rerun viewer
        rr.init("rerun_main", spawn=False)
        rr.send_blueprint(rerun_config["layout"].rerun_blueprint)
        # get the rrd_url if it wasn't provided
        dashboard_config["rrd_url"] = dashboard_config["rrd_url"] or rr.serve_grpc()  # e.g. "rerun+http://127.0.0.1:9876/proxy"
        thread = start_dashboard_server_thread(**dashboard_config)

        @self._disposables.add
        @Disposable
        def _cleanup_dashboard_thread():
            # Attempt to let the server thread shut down gracefully when the module stops.
            global dashboard_started
            dashboard_started = False
            if thread.is_alive():
                thread.join(timeout=1.0)