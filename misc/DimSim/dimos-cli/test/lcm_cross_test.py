#!/usr/bin/env python3
"""Quick test: can Python receive LCM messages from Deno?"""
import sys, time, threading
sys.path.insert(0, "/Users/viswajitnair/Desktop/4Wall.nosync/Dimensional/dimos")
import lcm

received = {"count": 0}

def handler(channel, data):
    received["count"] += 1
    print(f"[py] Got message #{received['count']} on {channel} ({len(data)} bytes)")

lc = lcm.LCM("udpm://239.255.76.67:7667?ttl=0")
lc.subscribe(".*lcm_cross_test.*", handler)

print("[py] Listening for LCM messages on /lcm_cross_test...")
print("[py] Waiting 15s for Deno publisher...\n")

deadline = time.time() + 15
while time.time() < deadline:
    lc.handle_timeout(500)
    if received["count"] >= 3:
        print(f"\n[py] SUCCESS: received {received['count']} messages from Deno!")
        sys.exit(0)

print(f"\n[py] FAIL: only received {received['count']} messages")
sys.exit(1)
