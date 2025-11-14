import asyncio
import logging
import wave
import numpy as np
import sys
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
from go2_webrtc_driver.webrtc_video import MediaHandler

# Enable logging for debugging
logging.basicConfig(level=logging.FATAL)

# Define the audio properties
samplerate = 48000  # Sample rate for WebRTC audio
channels = 2  # Stereo audio
filename = "output.wav"
record_duration = 5 # Record for 5 seconds
total_frames_to_record = record_duration * samplerate  # Total frames for the specified duration
frames_recorded = 0  # Counter for the number of frames recorded
done_writing_to_file = False  # Flag to indicate when writing is done

# Open the WAV file once for the entire duration
wf = wave.open(filename, 'wb')
wf.setnchannels(channels)
wf.setsampwidth(2)  # 2 bytes (16 bits) per sample
wf.setframerate(samplerate)

# Function to handle receiving audio frames and write them directly to file
async def recv_audio_stream(frame):
    global frames_recorded, done_writing_to_file

    if done_writing_to_file:
        return

    # Convert the frame to audio data (assuming 16-bit PCM)
    audio_data = np.frombuffer(frame.to_ndarray(), dtype=np.int16)

    # Write the audio data directly to the WAV file
    wf.writeframes(audio_data.tobytes())

    # Update the frame counter
    frames_recorded += len(audio_data) // channels

    # If we've recorded enough frames, stop further recording
    if frames_recorded >= total_frames_to_record:
        # Close the WAV file
        wf.close()
        print(f"Audio recording complete, saved to {filename}")
        done_writing_to_file = True

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
        await asyncio.sleep(record_duration + 1)  # Allow extra time to process the recording

    except ValueError as e:
        logging.error(f"Error in WebRTC connection: {e}")

if __name__ == "__main__":
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        # Handle Ctrl+C to exit gracefully.
        print("\nProgram interrupted by user")
        sys.exit(0)
