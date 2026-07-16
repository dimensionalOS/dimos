#!/usr/bin/env python3
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
