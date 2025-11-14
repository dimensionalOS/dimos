import asyncio
import logging
import sys
import json
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

def display_data(message):

    message = json.loads(message)
    
    body_height = message['bodyHeight']
    brightness = message['brightness']
    foot_raise_height = message['footRaiseHeight']
    obstacles_avoid_switch = message['obstaclesAvoidSwitch']
    speed_level = message['speedLevel']
    uwb_switch = message['uwbSwitch']
    volume = message['volume']

    # Clear the entire screen and reset cursor position to top
    sys.stdout.write("\033[H\033[J")

    # Print each piece of data on a separate line
    print("Go2 Multiple Robot Status")
    print("===================")
    print(f"Body Height:           {body_height:.2f} meters")
    print(f"Brightness:            {brightness}")
    print(f"Foot Raise Height:     {foot_raise_height:.2f} meters")
    print(f"Obstacles Avoid Switch: {'Enabled' if obstacles_avoid_switch else 'Disabled'}")
    print(f"Speed Level:           {speed_level}")
    print(f"UWB Switch:            {'On' if uwb_switch else 'Off'}")
    print(f"Volume:                {volume}/10")
    print("===================")
    
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


        # Define a callback function to handle multiplestate when received.
        def multiplestate_callback(message):
            current_message = message['data']
            
            display_data(current_message)


        # Subscribe to the multiplestate data and use the callback function to process incoming messages.
        conn.datachannel.pub_sub.subscribe(RTC_TOPIC['MULTIPLE_STATE'], multiplestate_callback)


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
