# Copyright 2026 Dimensional Inc.
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

"""Opt-in physical EEF policy; deployment bindings are operator-owned."""

import os
from pathlib import Path
import ssl
from types import SimpleNamespace

try:
    import tomllib
except ModuleNotFoundError:  # Python 3.10
    import tomli as tomllib


def _configured_blueprint():
    config_path = os.environ.get("OPENYAM_EEF_DIRECT_CONFIG")
    if not config_path:
        raise ValueError("OPENYAM_EEF_DIRECT_CONFIG must point to an operator-provided TOML file")
    path = Path(config_path).expanduser().resolve()
    settings = tomllib.loads(path.read_text(encoding="utf-8"))
    for key in ("can_port", "zenoh_domain", "zenoh_scout_addr"):
        if not isinstance(settings.get(key), str) or not settings[key].strip():
            raise ValueError(f"Explicit {key} required in OPENYAM_EEF_DIRECT_CONFIG")

    certs = None
    if "certs" in settings:
        certs = (path.parent / settings["certs"]).resolve()
        # An explicit directory must supply a valid pair; the default uses
        # the existing WebXR certificate generation path instead.
        ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER).load_cert_chain(
            certs / "cert.pem", certs / "key.pem", password=lambda: ""
        )

    from dimos.robot.manipulators.openyam.blueprints.eef_direct_real import real_eef_direct
    from dimos.robot.manipulators.openyam.eef_direct import (
        PLANT,
        PLANT_HASH,
        POLICY,
        POLICY_HASH,
        pinned_bytes,
    )

    pinned_bytes(POLICY, POLICY_HASH)
    pinned_bytes(PLANT, PLANT_HASH)
    return real_eef_direct(SimpleNamespace(
        can_port=settings["can_port"],
        quest_bind=settings.get("quest_bind", "127.0.0.1"),
        quest_port=settings.get("quest_port", 8443),
        viser_bind=settings.get("viser_bind", "127.0.0.1"),
        viser_port=settings.get("viser_port", 8080),
        zenoh_domain=settings["zenoh_domain"],
        zenoh_scout_addr=settings["zenoh_scout_addr"],
        certs=certs,
    ))


teleop_webxr_openyam_superreach_eef_direct_11p5k_real = _configured_blueprint().global_config(
    simulation="", zenoh_mode="peer", zenoh_interface="lo", zenoh_multicast=True,
    zenoh_connect="", robot_ip=None, robot_ips=None, local_relay=False, relay_url=None,
)
