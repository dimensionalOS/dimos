import logging
from .webrtc_datachannel import WebRTCDataChannel
from aiortc import RTCPeerConnection

class WebRTCVideoChannel:
    def __init__(self, pc:RTCPeerConnection, datachannel:WebRTCDataChannel) -> None:
        self.pc = pc
        self.pc.addTransceiver("video", direction="recvonly")
        self.datachannel = datachannel
        # List to hold multiple callbacks
        self.track_callbacks = []
    
    def switchVideoChannel(self, switch: bool):
        self.datachannel.switchVideoChannel(switch)
    
    def add_track_callback(self, callback):
        """
        Adds a callback to be triggered when an audio track is received.
        """
        if callable(callback):
            self.track_callbacks.append(callback)
        else:
            logging.warning(f"Callback {callback} is not callable.")  
    
    async def track_handler(self, track):
        logging.info("Receiving video frame")
        # Trigger all registered callbacks
        for callback in self.track_callbacks:
            try:
                # Call each callback function and pass the track
                await callback(track)
            except Exception as e:
                logging.error(f"Error in callback {callback}: {e}")
    