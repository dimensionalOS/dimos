import logging
import base64
from ..constants import DATA_CHANNEL_TYPE

class WebRTCDataChannelValidaton:
    def __init__(self, channel, pub_sub):
        self.channel = channel
        self.publish = pub_sub.publish
        self.on_validate_callbacks = []
        self.key = ""

    def set_on_validate_callback(self, callback):
        """Register a callback to be called upon validation."""
        if callback and callable(callback):
            self.on_validate_callbacks.append(callback)
    
    async def handle_response(self, message):
        if message.get("data") == "Validation Ok.":
            logging.info("Validation succeed")
            for callback in self.on_validate_callbacks:
                callback()
        else:
            self.channel._setReadyState("open")
            self.key = message.get("data")
            await self.publish(
                "",
                self.encrypt_key(self.key),
                DATA_CHANNEL_TYPE["VALIDATION"],
            )

    async def handle_err_response(self, message):
        if message.get("info") == "Validation Needed.":
            await self.publish(
                "",
                self.encrypt_key(self.key),
                DATA_CHANNEL_TYPE["VALIDATION"],
            )
        

    @staticmethod
    def hex_to_base64(hex_str):
        # Convert hex string to bytes
        bytes_array = bytes.fromhex(hex_str)
        # Encode the bytes to Base64 and return as a string
        return base64.b64encode(bytes_array).decode("utf-8")
    
    @staticmethod
    def encrypt_by_md5(input_str):
        import hashlib
        # Create an MD5 hash object
        hash_obj = hashlib.md5()
        # Update the hash object with the bytes of the input string
        hash_obj.update(input_str.encode("utf-8"))
        # Return the hex digest of the hash
        return hash_obj.hexdigest()

    @staticmethod
    def encrypt_key(key):
        # Append the prefix to the key
        prefixed_key = f"UnitreeGo2_{key}"
        # Encrypt the key using MD5 and convert to hex string
        encrypted = WebRTCDataChannelValidaton.encrypt_by_md5(prefixed_key)
        # Convert the hex string to Base64
        return WebRTCDataChannelValidaton.hex_to_base64(encrypted)
