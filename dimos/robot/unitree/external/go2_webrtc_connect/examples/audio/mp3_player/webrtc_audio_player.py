import logging
import asyncio
import os
import json
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.webrtc_audiohub import WebRTCAudioHub

# Enable logging for debugging
logging.basicConfig(level=logging.ERROR)
logger = logging.getLogger(__name__)

async def main():
    try:
        # Establish WebRTC connection
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.137.120")
        await conn.connect()
        logger.info("WebRTC connection established")

        # Create audio hub instance
        audio_hub = WebRTCAudioHub(conn, logger)
        logger.info("Audio hub initialized")

        # Define audio file to upload and play
        audio_file = "dog-barking.wav"
        audio_file_path = os.path.join(os.path.dirname(__file__), audio_file)
        logger.info(f"Using audio file: {audio_file_path}")

        # Get the list of available audio files
        response = await audio_hub.get_audio_list()
        if response and isinstance(response, dict):
            data_str = response.get('data', {}).get('data', '{}')
            audio_list = json.loads(data_str).get('audio_list', [])
            
            # Extract filename without extension
            filename = os.path.splitext(audio_file)[0]
            print(audio_list)
            # Check if file already exists by CUSTOM_NAME and store UUID
            existing_audio = next((audio for audio in audio_list if audio['CUSTOM_NAME'] == filename), None)
            if existing_audio:
                print(f"Audio file {filename} already exists, skipping upload")
                uuid = existing_audio['UNIQUE_ID']
            else:
                print(f"Audio file {filename} not found, proceeding with upload")
                uuid = None

                # Upload the audio file
                logger.info("Starting audio file upload...")
                await audio_hub.upload_audio_file(audio_file_path)
                logger.info("Audio file upload completed")
                response = await audio_hub.get_audio_list()
                existing_audio = next((audio for audio in audio_list if audio['CUSTOM_NAME'] == filename), None)
                uuid = existing_audio['UNIQUE_ID']

        # Play the uploaded audio file using its filename as UUID
        
        print(f"Starting audio playback of file: {uuid}")
        await audio_hub.play_by_uuid(uuid)
        logger.info("Audio playback completed")



    except Exception as e:
        logger.error(f"An error occurred: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        logger.info("Program terminated by user")
