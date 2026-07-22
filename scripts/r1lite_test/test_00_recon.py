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

"""
Test 0: Recon, dump everything the robot's ROS 2 graph exposes.

Read-only. Run this FIRST (before any other test) and use its output to
correct r1lite_config.py. Prints:
  - all topics with types and publisher/subscriber counts
  - measured publish rate for each /hdas/* feedback topic
  - message field sizes (joint counts) for JointState feedback topics
  - node list

Run:
    export ROS_DOMAIN_ID=<from domain_scan.py>
    python3 scripts/r1lite_test/test_00_recon.py
"""

import time

import rclpy

DISCOVERY_WAIT = 10.0
RATE_SAMPLE_S = 3.0


def main() -> bool:
    node = rclpy.create_node("r1lite_recon")

    print(f"Waiting {DISCOVERY_WAIT}s for DDS discovery...")
    deadline = time.time() + DISCOVERY_WAIT
    while time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    topics = sorted(node.get_topic_names_and_types())
    print(f"\n=== {len(topics)} topics ===")
    for name, types in topics:
        pubs = node.count_publishers(name)
        subs = node.count_subscribers(name)
        print(f"  {name}  [{', '.join(types)}]  pubs={pubs} subs={subs}")

    nodes = sorted(node.get_node_names_and_namespaces())
    print(f"\n=== {len(nodes)} nodes ===")
    for n, ns in nodes:
        print(f"  {ns.rstrip('/')}/{n}")

    # Sample rates + joint counts on JointState feedback topics.
    from sensor_msgs.msg import JointState

    js_topics = [n for n, t in topics if "sensor_msgs/msg/JointState" in t]
    print(f"\n=== JointState topics: rate + joint count ({RATE_SAMPLE_S}s sample each) ===")
    for topic in js_topics:
        stats = {"count": 0, "dof": None}

        def cb(msg, s=stats):
            s["count"] += 1
            s["dof"] = len(msg.position)

        sub = node.create_subscription(JointState, topic, cb, 10)
        t0 = time.time()
        while time.time() - t0 < RATE_SAMPLE_S:
            rclpy.spin_once(node, timeout_sec=0.1)
        node.destroy_subscription(sub)
        rate = stats["count"] / RATE_SAMPLE_S
        print(f"  {topic}: {rate:.1f} Hz, {stats['dof']} joints")

    print("\nDone. Update scripts/r1lite_test/r1lite_config.py with the above.")
    node.destroy_node()
    return len(topics) > 2  # more than our own /rosout + /parameter_events


if __name__ == "__main__":
    rclpy.init()
    result = main()
    rclpy.shutdown()
    exit(0 if result else 1)
