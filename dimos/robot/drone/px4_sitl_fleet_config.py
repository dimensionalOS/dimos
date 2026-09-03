# Copyright 2025-2026 Dimensional Inc.
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

"""Fleet configuration for PX4 SITL multi-vehicle simulations.

The PX4 multi-vehicle convention (and the in-repo ``dimos/simulation/px4_hil`` simulator)
maps SITL instance N to:
    * MAVLink offboard endpoint: ``udp:127.0.0.1:1454N``
    * MAVLink GCS port:          14550 + N
    * MAV_SYS_ID:                N + 1
    * Gazebo model name:         ``x500_N``

This module turns the DimOS ``--robot-ips`` flag (or a default fleet size of 3,
matching the sim demo deliverable) into a list of ``Px4SitlDroneConfig``
records that the fleet module can consume.
"""

from __future__ import annotations

from dataclasses import dataclass

from dimos.core.global_config import GlobalConfig, global_config

DEFAULT_SITL_HOST = "127.0.0.1"
DEFAULT_OFFBOARD_PORT_BASE = 14540
DEFAULT_GCS_PORT_BASE = 14550
DEFAULT_FLEET_SIZE = 3  # matches the X500 sim demo deliverable


@dataclass(frozen=True)
class Px4SitlDroneConfig:
    """Per-drone identity for a PX4 SITL instance."""

    key: str
    instance: int
    host: str
    offboard_port: int
    gcs_port: int
    sys_id: int
    model_name: str

    @property
    def connection_string(self) -> str:
        return f"udp:{self.host}:{self.offboard_port}"


def _split_csv(raw: str | None) -> list[str]:
    if raw is None:
        return []
    return [item.strip() for item in raw.split(",") if item.strip()]


def get_px4_sitl_fleet_configs(
    cfg: GlobalConfig = global_config,
    fleet_size: int | None = None,
) -> list[Px4SitlDroneConfig]:
    """Return the configured PX4 SITL fleet.

    Resolution order:
      1. Explicit ``fleet_size`` argument (used by blueprints to pick a default).
      2. CSV count from ``--robot-ips`` (each entry overrides the host).
      3. ``DEFAULT_FLEET_SIZE``.
    """
    hosts = _split_csv(cfg.robot_ips)
    if fleet_size is None:
        fleet_size = len(hosts) if hosts else DEFAULT_FLEET_SIZE
    if fleet_size < 1:
        raise ValueError(f"fleet_size must be >= 1, got {fleet_size}")

    drones: list[Px4SitlDroneConfig] = []
    for index in range(fleet_size):
        host = hosts[index] if index < len(hosts) else (hosts[0] if hosts else DEFAULT_SITL_HOST)
        drones.append(
            Px4SitlDroneConfig(
                key=f"drone-{index + 1}",
                instance=index,
                host=host,
                offboard_port=DEFAULT_OFFBOARD_PORT_BASE + index,
                gcs_port=DEFAULT_GCS_PORT_BASE + index,
                sys_id=index + 1,
                model_name=f"x500_{index}",
            )
        )
    return drones


def format_px4_sitl_fleet_prompt_block(
    cfg: GlobalConfig = global_config,
    fleet_size: int | None = None,
) -> str:
    """Render a concise prompt section describing the configured fleet."""
    drones = get_px4_sitl_fleet_configs(cfg, fleet_size=fleet_size)
    return "\n".join(
        f"- {d.key}: instance={d.instance}, sys_id={d.sys_id}, "
        f"model={d.model_name}, offboard={d.connection_string}"
        for d in drones
    )


__all__ = [
    "DEFAULT_FLEET_SIZE",
    "DEFAULT_GCS_PORT_BASE",
    "DEFAULT_OFFBOARD_PORT_BASE",
    "DEFAULT_SITL_HOST",
    "Px4SitlDroneConfig",
    "format_px4_sitl_fleet_prompt_block",
    "get_px4_sitl_fleet_configs",
]
