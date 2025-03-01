#!/usr/bin/env python3

import cv2
from dimos.robot.unitree.unitree_go2 import UnitreeGo2, WebRTCConnectionMethod
import os
import time
from dimos.robot.unitree.unitree_ros_control import UnitreeROSControl
import math
def get_env_var(var_name, default=None, required=False):
    """Get environment variable with validation."""
    value = os.getenv(var_name, default)
    if required and not value:
        raise ValueError(f"{var_name} environment variable is required")
    return value

if __name__ == "__main__":
    # Get configuration from environment variables
    robot_ip = get_env_var("ROBOT_IP", "192.168.9.140")
    connection_method = get_env_var("CONNECTION_METHOD", "LocalSTA")
    serial_number = get_env_var("SERIAL_NUMBER", None)
    output_dir = get_env_var("ROS_OUTPUT_DIR", os.path.join(os.getcwd(), "assets/output/ros"))
    api_call_interval = int(get_env_var("API_CALL_INTERVAL", "5"))
    
    # Ensure output directory exists
    os.makedirs(output_dir, exist_ok=True)
    print(f"Ensuring output directory exists: {output_dir}")
    
    use_ros = True
    use_webrtc = False
    # Convert connection method string to enum
    connection_method = getattr(WebRTCConnectionMethod, connection_method)

    print("Initializing UnitreeGo2...")
    print(f"Configuration:")
    print(f"  IP: {robot_ip}")
    print(f"  Connection Method: {connection_method}")
    print(f"  Serial Number: {serial_number if serial_number else 'Not provided'}")
    print(f"  Output Directory: {output_dir}")
    print(f"  API Call Interval: {api_call_interval} seconds")

    if use_ros:
        ros_control = UnitreeROSControl(
            node_name="unitree_go2",
            use_raw=True
        )
    else:
        ros_control = None

    robot = UnitreeGo2(
        ip=robot_ip,
        connection_method=connection_method,
        serial_number=serial_number,
        output_dir=output_dir,
        api_call_interval=api_call_interval,
        ros_control=ros_control,
        use_ros=use_ros,
        use_webrtc=use_webrtc
    )
    time.sleep(5)
    try:

        # Start perception
        print("\nStarting perception system...")
        
        # Get the processed stream
        processed_stream = robot.get_ros_video_stream(fps=30)
        
        # Create frame counter for unique filenames
        frame_count = 0
        
        # Create a subscriber to handle the frames
        def handle_frame(frame):
            global frame_count
            frame_count += 1
            
            try:
                # TODO: add debug frame logging mode for reactiveX debugging
                # Save frame to output directory
                #frame_path = os.path.join(output_dir, f"frame_{frame_count:04d}.jpg")
                #success = cv2.imwrite(frame_path, frame)
                #print(f"Frame #{frame_count} {'saved successfully' if success else 'failed to save'} to {frame_path}")
                pass

            except Exception as e:
                print(f"Error in handle_frame: {e}")
                import traceback
                print(traceback.format_exc())
        
        def handle_error(error):
            print(f"Error in stream: {error}")
            
        def handle_completion():
            print("Stream completed")
            cv2.destroyAllWindows()
        
        # Subscribe to the stream
        print("Creating subscription...")
        try:
            subscription = processed_stream.subscribe(
                on_next=handle_frame,
                on_error=lambda e: print(f"Subscription error: {e}"),
                on_completed=lambda: print("Subscription completed")
            )
            print("Subscription created successfully")
        except Exception as e:
            print(f"Error creating subscription: {e}")
        
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
        


        # Keep the original movement sequence and timing
        # time.sleep(5)
        # print("\nExecuting movement sequence...")
        # print("Moving forward...")
        # robot.move(distance=1.0, speed=0.5)

        # time.sleep(3)
        # robot.reverse(distance=2.0, speed=0.5)

        # time.sleep(3)
        # robot.spin(degrees=-90.0, speed=45.0)

        # time.sleep(3)
        # robot.spin(degrees=90.0, speed=45.0)
        
    #     print("\nMonitoring agent outputs (Press Ctrl+C to stop)...")
        while True:
            time.sleep(0.1)
            # robot.read_agent_outputs()
            
    except KeyboardInterrupt:
        print("\nStopping perception...")
        if 'subscription' in locals():
            subscription.dispose()
        cv2.destroyAllWindows()
    except Exception as e:
        print(f"Error in main loop: {e}")
    finally:
        # Cleanup
        print("Cleaning up resources...")
        if 'subscription' in locals():
            subscription.dispose()
        cv2.destroyAllWindows()
        del robot
        print("Cleanup complete.") 