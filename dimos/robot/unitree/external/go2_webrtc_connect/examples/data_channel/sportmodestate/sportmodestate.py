import asyncio
import logging
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

def display_data(message):

    imu_state = message['imu_state']
    quaternion = imu_state['quaternion']
    gyroscope = imu_state['gyroscope']
    accelerometer = imu_state['accelerometer']
    rpy = imu_state['rpy']
    temperature = imu_state['temperature']

    mode = message['mode']
    progress = message['progress']
    gait_type = message['gait_type']
    foot_raise_height = message['foot_raise_height']
    position = message['position']
    body_height = message['body_height']
    velocity = message['velocity']
    yaw_speed = message['yaw_speed']
    range_obstacle = message['range_obstacle']
    foot_force = message['foot_force']
    foot_position_body = message['foot_position_body']
    foot_speed_body = message['foot_speed_body']

    # Clear the entire screen and reset cursor position to top
    sys.stdout.write("\033[H\033[J")

    # Print each piece of data on a separate line
    print("Go2 Robot Status")
    print("===================")
    print(f"Mode: {mode}")
    print(f"Progress: {progress}")
    print(f"Gait Type: {gait_type}")
    print(f"Foot Raise Height: {foot_raise_height} m")
    print(f"Position: {position}")
    print(f"Body Height: {body_height} m")
    print(f"Velocity: {velocity}")
    print(f"Yaw Speed: {yaw_speed}")
    print(f"Range Obstacle: {range_obstacle}")
    print(f"Foot Force: {foot_force}")
    print(f"Foot Position (Body): {foot_position_body}")
    print(f"Foot Speed (Body): {foot_speed_body}")
    print("-------------------")
    print(f"IMU - Quaternion: {quaternion}")
    print(f"IMU - Gyroscope: {gyroscope}")
    print(f"IMU - Accelerometer: {accelerometer}")
    print(f"IMU - RPY: {rpy}")
    print(f"IMU - Temperature: {temperature}°C")
    
    # Optionally, flush to ensure immediate output
    sys.stdout.flush()



async def main():
    try:
        # Choose a connection method (uncomment the correct one)
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

        # Connect to the WebRTC service.
        await conn.connect()


        # Define a callback function to handle sportmode status when received.
        def sportmodestatus_callback(message):
            current_message = message['data']
            
            display_data(current_message)


        # Subscribe to the sportmode status data and use the callback function to process incoming messages.
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC['LF_SPORT_MOD_STATE'], sportmodestatus_callback)


        # Keep the program running to allow event handling for 1 hour.
        await asyncio.sleep(3600)

    except ValueError as e:
        # Log any value errors that occur during the process.
        logging.error(f"An error occurred: {e}")


if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Handle Ctrl+C to exit gracefully.
        print("\nProgram interrupted by user")
        sys.exit(0)
