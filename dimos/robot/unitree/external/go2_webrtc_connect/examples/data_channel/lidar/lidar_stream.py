import asyncio
import logging
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)
    
async def main():
    try:
        # Choose a connection method (uncomment the correct one)
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

        # Connect to the WebRTC service.
        await conn.connect()

        # Disable traffic saving mode on the data channel.
        await conn.datachannel.disableTrafficSaving(True)

        # Publish a message to turn the LIDAR sensor on.
        conn.datachannel.pub_sub.publish_without_callback("rt/utlidar/switch", "on")

        # Define a callback function to handle LIDAR messages when received.
        def lidar_callback(message):
            # Print the data received from the LIDAR sensor.
            print(message["data"])

        # Subscribe to the LIDAR voxel map data and use the callback function to process incoming messages.
        conn.datachannel.pub_sub.subscribe("rt/utlidar/voxel_map_compressed", lidar_callback)

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
