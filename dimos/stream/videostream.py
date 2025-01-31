import cv2

class VideoStream:
    def __init__(self, source=0):
        """
        Initialize the video stream from a camera source.
        
        Args:
            source (int or str): Camera index or video file path.
        """
        self.capture = cv2.VideoCapture(source)
        if not self.capture.isOpened():
            raise ValueError(f"Unable to open video source {source}")

    def __iter__(self):
        return self

    def __next__(self):
        ret, frame = self.capture.read()
        if not ret:
            self.capture.release()
            raise StopIteration
        return frame

    def release(self):
        self.capture.release()
