#!/usr/bin/env python3
"""Debug telemetry to see what we're actually getting."""

import time
from pymavlink import mavutil

def debug_telemetry():
    print("Debugging telemetry...")
    
    # Connect
    master = mavutil.mavlink_connection('udp:0.0.0.0:14550')
    master.wait_heartbeat(timeout=30)
    print(f"Connected to system {master.target_system}")
    
    # Read messages for 3 seconds
    start_time = time.time()
    message_types = {}
    
    while time.time() - start_time < 3:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            if msg_type not in message_types:
                message_types[msg_type] = 0
            message_types[msg_type] += 1
            
            # Print specific messages
            if msg_type == 'ATTITUDE':
                print(f"ATTITUDE: roll={msg.roll:.3f}, pitch={msg.pitch:.3f}, yaw={msg.yaw:.3f}")
            elif msg_type == 'GLOBAL_POSITION_INT':
                print(f"GLOBAL_POSITION_INT: lat={msg.lat/1e7:.6f}, lon={msg.lon/1e7:.6f}, alt={msg.alt/1000:.1f}m")
            elif msg_type == 'HEARTBEAT':
                armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                print(f"HEARTBEAT: mode={msg.custom_mode}, armed={armed}")
            elif msg_type == 'VFR_HUD':
                print(f"VFR_HUD: alt={msg.alt:.1f}m, groundspeed={msg.groundspeed:.1f}m/s")
    
    print("\nMessage types received:")
    for msg_type, count in message_types.items():
        print(f"  {msg_type}: {count}")

if __name__ == "__main__":
    debug_telemetry()