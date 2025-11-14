import asyncio
import logging
import numpy as np
import pyaudio
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.webrtc_video import MediaHandler

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

# Define the audio properties
samplerate = 48000  # Sample rate for WebRTC audio
channels = 2  # Stereo audio
frames_per_buffer = 8192  # Number of frames per buffer for PyAudio

# Initialize PyAudio
p = pyaudio.PyAudio()

# Open a PyAudio stream to output audio
stream = p.open(format=pyaudio.paInt16,
                channels=channels,
                rate=samplerate,
                output=True,
                frames_per_buffer=frames_per_buffer)

# Function to handle receiving audio frames and play them through the speakers
async def recv_audio_stream(frame):
    # Convert the frame to audio data (assuming 16-bit PCM)
    audio_data = np.frombuffer(frame.to_ndarray(), dtype=np.int16)

    # Play the audio data by writing it to the PyAudio stream
    stream.write(audio_data.tobytes())

# Main function for setting up the WebRTC connection and handling streams
async def main():
    try:
        # Choose a connection method (uncomment the correct one)
        conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.8.181")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, serialNumber="B42D2000XXXXXXXX")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.Remote, serialNumber="B42D2000XXXXXXXX", username="email@gmail.com", password="pass")
        # conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalAP)

        # Connect to the device
        await conn.connect()

        # Switch audio channel on and start receiving audio frames
        conn.audio.switchAudioChannel(True)

        # Add callback to handle received audio frames
        conn.audio.add_track_callback(recv_audio_stream)

        # Keep the program running to handle events
        await asyncio.sleep(3600)  # Keep running for 1 hour or as needed

    except ValueError as e:
        logging.error(f"Error in WebRTC connection: {e}")

    finally:
        # Stop and close the PyAudio stream when done
        stream.stop_stream()
        stream.close()
        p.terminate()

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Handle Ctrl+C to exit gracefully.
        print("\nProgram interrupted by user")
        sys.exit(0)
