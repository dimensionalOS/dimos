import logging
import asyncio
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from aiortc.contrib.media import MediaPlayer


# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

async def main():
    try:
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)
        
        await conn.connect()

        stream_url = "https://nashe1.hostingradio.ru:80/ultra-128.mp3" #Radio ultra

        logging.info(f"Playing internet radio: {stream_url}")
        player = MediaPlayer(stream_url)  # Use MediaPlayer with the URL
        audio_track = player.audio  # Get the audio track from the player
        conn.pc.addTrack(audio_track)  # Add the audio track to the WebRTC connection

        await asyncio.sleep(3600)  # Keep the program running to handle events

    except ValueError as e:
        logging.error(e)

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Handle Ctrl+C to exit gracefully.
        print("\nProgram interrupted by user")
        sys.exit(0)