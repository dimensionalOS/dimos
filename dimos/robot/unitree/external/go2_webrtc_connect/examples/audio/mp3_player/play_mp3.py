import logging
import asyncio
import os 
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from aiortc.contrib.media import MediaPlayer


# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

async def main():
    try:
        # Choose a connection method (uncomment the correct one)
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)
        
        await conn.connect()

        
        mp3_path = os.path.join(os.path.dirname(__file__), "dora-doradura-mp3.mp3")
        
        logging.info(f"Playing MP3: {mp3_path}")
        player = MediaPlayer(mp3_path)  # Use MediaPlayer for MP3
        audio_track = player.audio  # Get the audio track from the player
        conn.pc.addTrack(audio_track)  # Add the audio track to the WebRTC connection

        await asyncio.sleep(3600)  # Keep the program running to handle events

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

