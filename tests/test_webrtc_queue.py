#!/usr/bin/env python3

import time
from dimos.robot.unitree.unitree_go2 import UnitreeGo2, WebRTCConnectionMethod
import os
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl

def main():
    """Test WebRTC request queue with a sequence of 20 back-to-back commands"""
    
    print("Initializing UnitreeGo2...")
    
    # Get configuration from environment variables
    robot_ip = os.getenv("ROBOT_IP", "192.168.9.140")
    connection_method = getattr(WebRTCConnectionMethod, 
                               os.getenv("CONNECTION_METHOD", "LocalSTA"))
    
    # Initialize ROS control
    ros_control = UnitreeROSControl(
        node_name="unitree_go2_test",
        use_raw=True
    )
    
    # Initialize robot
    robot = UnitreeGo2(
        ip=robot_ip,
        connection_method=connection_method,
        ros_control=ros_control,
        use_ros=True,
        use_webrtc=False  # Using queue instead of direct WebRTC
    )
    
    # Wait for initialization
    print("Waiting for robot to initialize...")
    time.sleep(5)
    
    # First put the robot in a good starting state
    print("Running recovery stand...")
    robot.webrtc_req(api_id=1006)  # RecoveryStand
    
    # Queue 20 WebRTC requests back-to-back
    print("\n🤖 QUEUEING 20 COMMANDS BACK-TO-BACK 🤖\n")
    
    # Dance 1
    robot.webrtc_req(api_id=1022)  # Dance1
    print("Queued: Dance1 (1022)")
    
    # Wiggle Hips
    robot.webrtc_req(api_id=1033)  # WiggleHips
    print("Queued: WiggleHips (1033)")
    
    # Stretch
    robot.webrtc_req(api_id=1017)  # Stretch
    print("Queued: Stretch (1017)")
    
    # Hello
    robot.webrtc_req(api_id=1016)  # Hello
    print("Queued: Hello (1016)")
    
    # Dance 2
    robot.webrtc_req(api_id=1023)  # Dance2
    print("Queued: Dance2 (1023)")
    
    # Wallow
    robot.webrtc_req(api_id=1021)  # Wallow
    print("Queued: Wallow (1021)")
    
    # Scrape
    robot.webrtc_req(api_id=1029)  # Scrape
    print("Queued: Scrape (1029)")
    
    # Finger Heart
    robot.webrtc_req(api_id=1036)  # FingerHeart
    print("Queued: FingerHeart (1036)")
    
    # Recovery Stand (base position)
    robot.webrtc_req(api_id=1006)  # RecoveryStand
    print("Queued: RecoveryStand (1006)")
    
    # Hello again
    robot.webrtc_req(api_id=1016)  # Hello
    print("Queued: Hello (1016)")
    
    # Wiggle Hips again
    robot.webrtc_req(api_id=1033)  # WiggleHips
    print("Queued: WiggleHips (1033)")
    
    # Front Pounce
    robot.webrtc_req(api_id=1032)  # FrontPounce
    print("Queued: FrontPounce (1032)")
    
    # Dance 1 again
    robot.webrtc_req(api_id=1022)  # Dance1
    print("Queued: Dance1 (1022)")
    
    # Stretch again
    robot.webrtc_req(api_id=1017)  # Stretch
    print("Queued: Stretch (1017)")
    
    # Front Jump
    robot.webrtc_req(api_id=1031)  # FrontJump
    print("Queued: FrontJump (1031)")
    
    # Finger Heart again
    robot.webrtc_req(api_id=1036)  # FingerHeart
    print("Queued: FingerHeart (1036)")
    
    # Scrape again
    robot.webrtc_req(api_id=1029)  # Scrape
    print("Queued: Scrape (1029)")
    
    # Hello one more time
    robot.webrtc_req(api_id=1016)  # Hello
    print("Queued: Hello (1016)")
    
    # Dance 2 again
    robot.webrtc_req(api_id=1023)  # Dance2
    print("Queued: Dance2 (1023)")
    
    # Finish with recovery stand
    robot.webrtc_req(api_id=1006)  # RecoveryStand
    print("Queued: RecoveryStand (1006)")
    
    print("\nAll 20 commands queued successfully! Watch the robot perform them in sequence.")
    print("The WebRTC queue manager will process them one by one when the robot is ready.")
    print("Press Ctrl+C to stop the program when you've seen enough.\n")
    
    try:
        # Keep the program running so the queue can be processed
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\nStopping the test...")
    finally:
        # Cleanup
        print("Cleaning up resources...")
        robot.cleanup()
        print("Test completed.")

if __name__ == "__main__":
    main() 