#!/usr/bin/env python3
"""Test if drone system is healthy despite LCM errors."""

import time
from dimos.protocol.pubsub.lcmpubsub import LCM, Topic
from dimos.msgs.geometry_msgs import PoseStamped
from dimos.msgs.sensor_msgs import Image
from dimos_lcm.std_msgs import String

def check_topics():
    """Check if topics are being published."""
    lcm = LCM()
    
    topics_to_check = [
        ("/drone/odom", PoseStamped, "Odometry"),
        ("/drone/status", String, "Status"),
        ("/drone/video", Image, "Video"),
        ("/drone/color_image", Image, "Color Image"),
    ]
    
    results = {}
    
    for topic_name, msg_type, description in topics_to_check:
        topic = Topic(topic_name, msg_type)
        print(f"Checking {description} on {topic_name}...", end=" ")
        
        try:
            msg = lcm.wait_for_message(topic, timeout=2.0)
            if msg:
                print("✓ Received")
                results[topic_name] = True
            else:
                print("✗ No message")
                results[topic_name] = False
        except Exception as e:
            print(f"✗ Error: {e}")
            results[topic_name] = False
    
    return results

if __name__ == "__main__":
    print("Checking drone system health...")
    print("(System should be running with: python dimos/robot/drone/run_drone.py)\n")
    
    results = check_topics()
    
    print("\n=== Summary ===")
    working = sum(results.values())
    total = len(results)
    
    if working == total:
        print(f"✓ All {total} topics working!")
    else:
        print(f"⚠ {working}/{total} topics working")
        print("Not working:")
        for topic, status in results.items():
            if not status:
                print(f"  - {topic}")