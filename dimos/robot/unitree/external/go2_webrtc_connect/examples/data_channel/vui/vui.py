import asyncio
import logging
import json
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.constants import RTC_TOPIC, VUI_COLOR

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

        ############################
        ####### FLASH LIGHT ########
        ############################

        # Get the current brightness
        print("\nFetching the current brightness level...")
        response = await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"], 
            {"api_id": 1006}
        )

        if response['data']['header']['status']['code'] == 0:
            data = json.loads(response['data']['data'])
            current_brightness = data['brightness']
            print(f"Current brightness level: {current_brightness}\n")

        # Adjusting brightness level from 0 to 10
        print("Increasing brightness from 0 to 10...")
        for brightness_level in range(0, 11):
            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["VUI"], 
                {
                    "api_id": 1005,
                    "parameter": {"brightness": brightness_level}
                }
            )
            print(f"Brightness level: {brightness_level}/10")
            await asyncio.sleep(0.5)

        # Adjusting brightness level from 10 back to 0
        print("\nDecreasing brightness from 10 to 0...")
        for brightness_level in range(10, -1, -1):
            await conn.datachannel.pub_sub.publish_request_new(
                RTC_TOPIC["VUI"], 
                {
                    "api_id": 1005,
                    "parameter": {"brightness": brightness_level}
                }
            )
            print(f"Brightness level: {brightness_level}/10")
            await asyncio.sleep(0.5)

        # Change the LED color to purple
        print("\nChanging LED color to purple for 5 seconds...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"], 
            {
                "api_id": 1007,
                "parameter": 
                {
                    "color": VUI_COLOR.PURPLE,
                    "time": 5
                }
            }
        )
        await asyncio.sleep(6)

        # Change the LED color to cyan and flash
        # flash_cycle is between 499 and time*1000
        print("\nChanging LED color to cyan with flash (cycle: 1000ms)...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"], 
            {
                "api_id": 1007,
                "parameter": 
                {
                    "color": VUI_COLOR.CYAN,
                    "time": 5,
                    "flash_cycle": 1000  # Flash every second
                }
            }
        )
        await asyncio.sleep(5)

        ############################
        ####### VOLUME CONTROL ######
        ############################

        # Get the current volume
        print("\nFetching the current volume level...")
        response = await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"], 
            {"api_id": 1004}
        )

        if response['data']['header']['status']['code'] == 0:
            data = json.loads(response['data']['data'])
            current_volume = data['volume']
            print(f"Current volume level: {current_volume}/10\n")

        # Set Volume to 50%
        print("Setting volume to 50% (5/10)...")
        await conn.datachannel.pub_sub.publish_request_new(
            RTC_TOPIC["VUI"], 
            {
                "api_id": 1003,
                "parameter": {"volume": 5}
            }
        )

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
