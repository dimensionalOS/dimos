import tests.test_header
import os

import logging
logging.basicConfig(level=logging.DEBUG)

from fastapi import FastAPI, Response
import cv2
import uvicorn
from starlette.responses import StreamingResponse

app = FastAPI()

# Note: Chrome does not allow for loading more than 6 simultaneous 
# video streams. Use Safari or another browser for utilizing 
# multiple simultaneous streams. Possibly build out functionality
# that will stop live streams.

@app.get("/")
async def root():
    pid = os.getpid()  # Get the current process ID
    return {"message": f"Video Streaming Server, PID: {pid}"}

def video_stream_generator():
    pid = os.getpid()
    print(f"Stream initiated by worker with PID: {pid}")  # Log the PID when the generator is called

    # Use the correct path for your video source
    cap = cv2.VideoCapture(f"{os.getcwd()}/assets/trimmed_video_480p.mov")  # Change 0 to a filepath for video files

    if not cap.isOpened():
        yield (b'--frame\r\nContent-Type: text/plain\r\n\r\n' + b'Could not open video source\r\n')
        return

    try:
        while True:
            ret, frame = cap.read()
            # If frame is read correctly ret is True
            if not ret:
                print(f"Reached the end of the video, restarting... PID: {pid}")
                cap.set(cv2.CAP_PROP_POS_FRAMES, 0)  # Set the position of the next video frame to 0 (the beginning)
                continue
            _, buffer = cv2.imencode('.jpg', frame)
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
    finally:
        cap.release()

@app.get("/video")
async def video_endpoint():
    logging.debug("Attempting to open video stream.")
    response = StreamingResponse(video_stream_generator(), media_type='multipart/x-mixed-replace; boundary=frame')
    logging.debug("Streaming response set up.")
    return response

if __name__ == "__main__":
    uvicorn.run("__main__:app", host="0.0.0.0", port=5555, workers=20)
