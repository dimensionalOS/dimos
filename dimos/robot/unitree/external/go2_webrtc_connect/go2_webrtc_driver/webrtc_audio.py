
from aiortc import AudioStreamTrack, RTCRtpSender
import logging
import sounddevice as sd
import numpy as np
import wave


class WebRTCAudioChannel:
    def __init__(self, pc, datachannel) -> None:
        self.pc = pc
        self.pc.addTransceiver("audio", direction="sendrecv")
        self.datachannel = datachannel

        # List to hold multiple callbacks
        self.track_callbacks = []
        
    async def frame_handler(self, frame):
        logging.info("Receiving audio frame")

        # Trigger all registered callbacks
        for callback in self.track_callbacks:
            try:
                # Call each callback function and pass the track
                await callback(frame)
            except Exception as e:
                logging.error(f"Error in callback {callback}: {e}")
    
    def add_track_callback(self, callback):
        """
        Adds a callback to be triggered when an audio track is received.
        """
        if callable(callback):
            self.track_callbacks.append(callback)
        else:
            logging.warning(f"Callback {callback} is not callable.")  

    def switchAudioChannel(self, switch: bool):
        self.datachannel.switchAudioChannel(switch)
    
        