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

"""Sweep ROS_DOMAIN_ID 0-101 and report domains with foreign participants.

DDS participant discovery is multicast and fast on a wired LAN, so a short
dwell per domain is enough to spot a robot whose domain we don't know.
"""

import sys
import time

import rclpy
from rclpy.context import Context
from rclpy.node import Node


def scan_domain(domain_id: int, dwell_s: float = 0.7) -> list[str]:
    ctx = Context()
    rclpy.init(context=ctx, domain_id=domain_id)
    try:
        node = Node(f"domain_scan_{domain_id}", context=ctx)
        try:
            time.sleep(dwell_s)
            names = [
                f"{ns}/{name}" if ns != "/" else f"/{name}"
                for name, ns in node.get_node_names_and_namespaces()
            ]
            return [n for n in names if f"domain_scan_{domain_id}" not in n]
        finally:
            node.destroy_node()
    finally:
        rclpy.shutdown(context=ctx)


def main() -> None:
    hits = {}
    for d in range(0, 102):
        others = scan_domain(d)
        if others:
            hits[d] = others
            print(f"DOMAIN {d}: {len(others)} foreign node(s): {others[:8]}", flush=True)
        if d % 20 == 0:
            print(f"...scanned up to domain {d}", file=sys.stderr, flush=True)
    if not hits:
        print("No foreign ROS 2 participants found on any domain 0-101.")


if __name__ == "__main__":
    main()
