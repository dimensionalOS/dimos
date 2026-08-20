#!/usr/bin/env python3
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

"""A configured robot IP becomes an explicit zenoh endpoint.

Multicast scouting dies on many APs, so a session opened while
``--robot-ip`` is set must dial the robot's bridge directly.
"""

from dimos.core.global_config import global_config
from dimos.protocol.service.zenohservice import ZenohConfig


def test_robot_ip_becomes_connect_endpoint(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    assert ZenohConfig().connect == ["tcp/192.0.2.10:7447"]


def test_no_robot_ip_keeps_scouting_only(zenoh_defaults):
    assert ZenohConfig().connect == []


def test_lcm_transport_derives_nothing(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "transport", "lcm")
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    assert ZenohConfig().connect == []


def test_explicit_port_is_kept(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10:9000")
    assert ZenohConfig().connect == ["tcp/192.0.2.10:9000"]


def test_robot_ips_list_dedupes_against_robot_ip(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    monkeypatch.setattr(global_config, "robot_ips", "192.0.2.10, 192.0.2.11")
    assert ZenohConfig().connect == [
        "tcp/192.0.2.10:7447",
        "tcp/192.0.2.11:7447",
    ]


def test_caller_override_wins(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    assert ZenohConfig(connect=["tcp/198.51.100.7:7447"]).connect == ["tcp/198.51.100.7:7447"]


def test_zenoh_connect_names_a_non_robot_endpoint(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "zenoh_connect", "tcp/127.0.0.1:17450")
    assert ZenohConfig().connect == ["tcp/127.0.0.1:17450"]


def test_zenoh_connect_appends_to_the_robot_endpoints(zenoh_defaults, monkeypatch):
    monkeypatch.setattr(global_config, "robot_ip", "192.0.2.10")
    monkeypatch.setattr(global_config, "zenoh_connect", "tcp/127.0.0.1:17450, tcp/127.0.0.1:17451")
    assert ZenohConfig().connect == [
        "tcp/192.0.2.10:7447",
        "tcp/127.0.0.1:17450",
        "tcp/127.0.0.1:17451",
    ]
