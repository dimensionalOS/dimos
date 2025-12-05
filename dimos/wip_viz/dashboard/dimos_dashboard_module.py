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
from typing import Optional

import rerun as rr  # pip install rerun-sdk
import rerun.blueprint as rrb

from dimos.core import In, Module, rpc
from dimos.wip_viz.dashboard.dimos_dashboard_func import (
    dimos_dashboard_func,
    env_bool,
    normalize_path_prefix,
)
from dimos.wip_viz.rerun.types import BlueprintRecord

_dashboard_exists = False


class Dashboard(Module):
    """
    Internals Note:
        The Dashboard handles rendering the terminals (Zellij) and the viewer (Rerun).
        The Layout (elsewhere) handles the layout of rerun.
        The dimos_dashboard_func mostly handles the logic for Zellij, with only an iframe for rerun.
    """

    blueprint_record: In[BlueprintRecord] = None

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
        frontend_base_path: str = normalize_path_prefix(
            os.environ.get("FRONTEND_BASE_PATH", "/zviewer")
        ),
        api_base_path: str = normalize_path_prefix(os.environ.get("API_BASE_PATH", "/zviewer/api")),
        https_enabled: bool = env_bool("HTTPS_ENABLED", False),
        https_key_path: str | None = os.environ.get("HTTPS_KEY_PATH"),
        https_cert_path: str | None = os.environ.get("HTTPS_CERT_PATH"),
        zellij_target: str | None = None,
        backend_target: str | None = None,
        frontend_target: str | None = None,
        terminal_commands: dict[str, str] | None = None,
        zellij_token: str | None = os.environ.get("ZELLIJ_TOKEN"),
        zellij_namespace: str | None = "dimos",
        logger: logging.Logger | None = None,
        rrd_url: str | None = None,
        **kwargs,
    ) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)
        global _dashboard_exists
        if _dashboard_exists:
            raise Exception(
                """Dashboard already exists, currently only one Dashboard can be active at a time."""
            )
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

    @rpc
    def start(self) -> None:
        print("STARTING DASHBOARD MODULE")

        @self.blueprint_record.subscribe
        def handle_blueprint(blueprint_record: BlueprintRecord):
            print("HANDLE BLUEPRINT")
            # print(f"[Dashboard] got blueprint! {blueprint_record}")
            print(f"""self.active_blueprint = {self.active_blueprint}""")
            if self.active_blueprint is None:
                self.active_blueprint = blueprint_record.blueprint
                print("[Dashboard] init-ing rerun")
                # could let name be custom in the future
                # init makes it so that rr.log() actually does something (buffers in memory until serve_grpc is called and available)
                rr.init("rerun_mega_blueprint", spawn=False)
                rr.send_blueprint(self.active_blueprint)  # needs to be after init
                print("[Dashboard] sent blueprint")
                # get the rrd_url if it wasn't provided
                print(kwargs_for_func)
                self.kwargs_for_func["rrd_url"] = (
                    self.kwargs_for_func["rrd_url"] or rr.serve_grpc()
                )  # e.g. "rerun+http://127.0.0.1:9876/proxy"
                # start the custom server, zellij tools, etc
                dimos_dashboard_func(**self.kwargs_for_func)
                print("[Dashboard] started dimos_dashboard_func")
            else:
                raise Exception("""Dashboard already has a blueprint, cannot set new blueprint.""")
