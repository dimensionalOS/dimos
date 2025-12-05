#!/usr/bin/env python3
from __future__ import annotations

import os


import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb
from dimos.wip_viz.rerun.types import BlueprintRecord
from dimos.wip_viz.dashboard.dimos_dashboard_func import dimos_dashboard_func

import secrets
import string

_dashboard_exists = False
class Dashboard(Module):
    blueprint_record : In[BlueprintRecord]  = None
    
    def __init__(
        self,
        *,
        port: int = int(os.environ.get("PROXY_PORT", "4000")),
        proxy_host: str = os.environ.get("PROXY_HOST", "localhost"),
        zellij_host: str = os.environ.get("ZELLIJ_HOST", "127.0.0.1"),
        zellij_port: int = int(os.environ.get("ZELLIJ_PORT", "8083")),
        backend_host: str = os.environ.get("BACKEND_HOST", "localhost"),
        backend_port: int = int(os.environ.get("BACKEND_PORT", "3001")),
        frontend_host: str = os.environ.get("FRONTEND_HOST", "localhost"),
        frontend_port: int = int(os.environ.get("FRONTEND_PORT", "5173")),
        frontend_base_path: str = normalize_path_prefix(os.environ.get("FRONTEND_BASE_PATH", "/zviewer")),
        api_base_path: str = normalize_path_prefix(os.environ.get("API_BASE_PATH", "/zviewer/api")),
        https_enabled: bool = env_bool("HTTPS_ENABLED", False),
        https_key_path: Optional[str] = os.environ.get("HTTPS_KEY_PATH"),
        https_cert_path: Optional[str] = os.environ.get("HTTPS_CERT_PATH"),
        zellij_target: Optional[str] = None,
        backend_target: Optional[str] = None,
        frontend_target: Optional[str] = None,
        terminal_commands: Optional[dict[str, str]] = None,
        zellij_token: Optional[str] = os.environ.get("ZELLIJ_TOKEN"),
        zellij_namespace: Optional[str] = "dimos",
        logger: Optional[logging.Logger] = None,
        rrd_url: Optional[str] = None,
        **kwargs,
    ) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        global _dashboard_exists
        if _dashboard_exists:
            raise Exception(f'''Dashboard already exists, currently only one Dashboard can be active at a time.''')
            _dashboard_exists = True
            
        self.kwargs_for_func = dict(
            port=port,
            proxy_host=proxy_host,
            zellij_host=zellij_host,
            zellij_port=zellij_port,
            backend_host=backend_host,
            backend_port=backend_port,
            frontend_host=frontend_host,
            frontend_port=frontend_port,
            frontend_base_path=frontend_base_path,
            api_base_path=api_base_path,
            https_enabled=https_enabled,
            https_key_path=https_key_path,
            https_cert_path=https_cert_path,
            zellij_target=zellij_target,
            backend_target=backend_target,
            frontend_target=frontend_target,
            terminal_commands=terminal_commands,
            zellij_token=zellij_token,
            zellij_namespace=zellij_namespace,
            logger=logger,
            rrd_url=rrd_url,
        )
        
        self.active_blueprint = None

    def start(self) -> None:
        @self.blueprint_record.subscribe
        def handle_blueprint(blueprint_record: BlueprintRecord):
            if self.active_blueprint == None:
                self.active_blueprint = blueprint_record.blueprint
                rr.send_blueprint(self.active_blueprint)
                # could let name be custom in the future
                rr.init("rerun_mega_blueprint", spawn=False)
                # FIXME: figure out how to get the rrd_url situation (what does init do? what address does rr.log() send to? how do I get the rdd_url out of rr.serve_grpc? is that even where the url is coming from?)
                rr.serve_grpc()
                # start the custom server, zellij tools, etc
                dimos_dashboard_func(**self.kwargs_for_func)
            else:
                raise Exception(f'''Dashboard already has a blueprint, cannot set new blueprint.''')