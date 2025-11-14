# Copyright (c) 2024, RoboVerse community
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#    list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

# WEB_RTC_CON WAS ORIGINALY FORKED from https://github.com/tfoldi/go2-webrtc/tree/master
# and https://github.com/legion1581/go2_webrtc_connect
# Big thanks for your passion! @tfoldi (Földi Tamás) and @legion1581 (The RoboVerse Discord Group)


import binascii
import uuid
import base64
import hashlib
import json
import logging
import struct
from Crypto.PublicKey import RSA
from Crypto.Cipher import AES
from Crypto.Cipher import PKCS1_v1_5
import requests
from aiortc import RTCPeerConnection, RTCSessionDescription


from scripts.go2_lidar_decoder import LidarDecoder


logging.basicConfig(level=logging.WARN)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


decoder = LidarDecoder()


def calc_local_path_ending(data1):
    # Initialize an array of strings
    strArr = ["A", "B", "C", "D", "E", "F", "G", "H", "I", "J"]

    # Extract the last 10 characters of data1
    last_10_chars = data1[-10:]

    # Split the last 10 characters into chunks of size 2
    chunked = [last_10_chars[i:i + 2] for i in range(0, len(last_10_chars), 2)]

    # Initialize an empty list to store indices
    arrayList = []

    # Iterate over the chunks and find the index of the second character in strArr
    for chunk in chunked:
        if len(chunk) > 1:
            second_char = chunk[1]
            try:
                index = strArr.index(second_char)
                arrayList.append(index)
            except ValueError:
                # Handle case where the character is not found in strArr
                print(f"Character {second_char} not found in strArr.")

    # Convert arrayList to a string without separators
    joinToString = ''.join(map(str, arrayList))

    return joinToString


def generate_aes_key() -> str:
    uuid_32 = uuid.uuid4().bytes
    uuid_32_hex_string = binascii.hexlify(uuid_32).decode('utf-8')
    return uuid_32_hex_string


def rsa_load_public_key(pem_data: str) -> RSA.RsaKey:
    """Load an RSA public key from a PEM-formatted string."""
    key_bytes = base64.b64decode(pem_data)
    return RSA.import_key(key_bytes)


def pad(data: str) -> bytes:
    """Pad data to be a multiple of 16 bytes (AES block size)."""
    block_size = AES.block_size
    padding = block_size - len(data) % block_size
    padded_data = data + chr(padding) * padding
    return padded_data.encode('utf-8')


def aes_encrypt(data: str, key: str) -> str:
    """Encrypt the given data using AES (ECB mode with PKCS5 padding)."""
    # Ensure key is 32 bytes for AES-256
    key_bytes = key.encode('utf-8')

    # Pad the data to ensure it is a multiple of block size
    padded_data = pad(data)

    # Create AES cipher in ECB mode
    cipher = AES.new(key_bytes, AES.MODE_ECB)

    encrypted_data = cipher.encrypt(padded_data)
    encoded_encrypted_data = base64.b64encode(encrypted_data).decode('utf-8')

    return encoded_encrypted_data


def rsa_encrypt(data: str, public_key: RSA.RsaKey) -> str:
    """Encrypt data using RSA and a given public key."""
    cipher = PKCS1_v1_5.new(public_key)

    # Maximum chunk size for encryption with RSA/ECB/PKCS1Padding is key size - 11 bytes
    max_chunk_size = public_key.size_in_bytes() - 11
    data_bytes = data.encode('utf-8')

    encrypted_bytes = bytearray()
    for i in range(0, len(data_bytes), max_chunk_size):
        chunk = data_bytes[i:i + max_chunk_size]
        encrypted_chunk = cipher.encrypt(chunk)
        encrypted_bytes.extend(encrypted_chunk)

    # Base64 encode the final encrypted data
    encoded_encrypted_data = base64.b64encode(encrypted_bytes).decode('utf-8')
    return encoded_encrypted_data


def unpad(data: bytes) -> str:
    """Remove padding from data."""
    padding = data[-1]
    return data[:-padding].decode('utf-8')


def aes_decrypt(encrypted_data: str, key: str) -> str:
    """Decrypt the given data using AES (ECB mode with PKCS5 padding)."""
    # Ensure key is 32 bytes for AES-256
    key_bytes = key.encode('utf-8')

    # Decode Base64 encrypted data
    encrypted_data_bytes = base64.b64decode(encrypted_data)

    # Create AES cipher in ECB mode
    cipher = AES.new(key_bytes, AES.MODE_ECB)

    # Decrypt data
    decrypted_padded_data = cipher.decrypt(encrypted_data_bytes)

    # Unpad the decrypted data
    decrypted_data = unpad(decrypted_padded_data)

    return decrypted_data


def make_local_request(path, body=None, headers=None):
    try:
        # Send POST request with provided path, body, and headers
        response = requests.post(url=path, data=body, headers=headers)

        # Check if the request was successful (status code 200)
        response.raise_for_status()  # Raises an HTTPError for bad responses (4xx, 5xx)

        if response.status_code == 200:
            return response  # Returning the whole response object if needed
        else:
            # Handle non-200 responses
            return None

    except requests.exceptions.RequestException as e:
        # Handle any exception related to the request (e.g., connection errors, timeouts)
        logging.error(f"An error occurred: {e}")
        return None


class Go2Connection():
    def __init__(
            self,
            robot_ip=None,
            robot_num=None,
            token="",
            on_validated=None,
            on_message=None,
            on_open=None,
            on_video_frame=None,
            decode_lidar=True,
    ):

        self.pc = RTCPeerConnection()
        self.robot_ip = robot_ip
        self.robot_num = str(robot_num)
        self.token = token
        self.robot_validation = "PENDING"
        self.on_validated = on_validated
        self.on_message = on_message
        self.on_open = on_open
        self.on_video_frame = on_video_frame
        self.decode_lidar = decode_lidar

        self.data_channel = self.pc.createDataChannel("data", id=0)
        self.data_channel.on("open", self.on_data_channel_open)
        self.data_channel.on("message", self.on_data_channel_message)

        self.pc.on("track", self.on_track)
        self.pc.on("connectionstatechange", self.on_connection_state_change)

        if self.on_video_frame:
            self.pc.addTransceiver("video", direction="recvonly")

    def on_connection_state_change(self):
        logger.info(f"Connection state is {self.pc.connectionState}")

    async def on_track(self, track):
        logger.info(f"Receiving {track.kind}")
        if track.kind == "audio":
            pass
        elif track.kind == "video":
            if self.on_video_frame:
                frame = await track.recv()
                logger.info(f"Received frame {frame}")
                await self.on_video_frame(track, int(self.robot_num))
            else:
                pass

    async def generate_offer(self):
        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)
        return offer.sdp

    async def set_answer(self, sdp):
        answer = RTCSessionDescription(sdp, type="answer")
        await self.pc.setRemoteDescription(answer)

    def on_data_channel_open(self):
        logger.info("Data channel is open")
        if self.on_open:
            self.on_open()

    def on_data_channel_message(self, msg):

        logger.debug("Received message: %s", msg)

        if self.data_channel.readyState != "open":
            self.data_channel._setReadyState("open")

        try:
            if isinstance(msg, str):
                msgobj = json.loads(msg)
                if msgobj.get("type") == "validation":
                    self.validate_robot_conn(msgobj)
            elif isinstance(msg, bytes):
                msgobj = Go2Connection.deal_array_buffer(msg, perform_decode=self.decode_lidar)

            if self.on_message:
                self.on_message(msg, msgobj, self.robot_num)

        except json.JSONDecodeError:
            pass

    async def connect(self):

        logging.info("Trying to send SDP using a NEW method...")

        offer = await self.pc.createOffer()
        await self.pc.setLocalDescription(offer)

        sdp_offer = self.pc.localDescription

        sdp_offer_json = {
            "id": "STA_localNetwork",
            "sdp": sdp_offer.sdp,
            "type": sdp_offer.type,
            "token": self.token
        }

        new_sdp = json.dumps(sdp_offer_json)
        url = f"http://{self.robot_ip}:9991/con_notify"
        response = make_local_request(url, body=None, headers=None)

        if response:

            # Decode the response text from base64
            decoded_response = base64.b64decode(response.text).decode('utf-8')

            # Parse the decoded response as JSON
            decoded_json = json.loads(decoded_response)

            # Extract the 'data1' field from the JSON
            data1 = decoded_json.get('data1')

            # Extract the public key from 'data1'
            public_key_pem = data1[10:len(data1)-10]
            path_ending = calc_local_path_ending(data1)

            # Generate AES key
            aes_key = generate_aes_key()

            # Load Public Key
            public_key = rsa_load_public_key(public_key_pem)

            # Encrypt the SDP and AES key
            body = {
                "data1": aes_encrypt(new_sdp, aes_key),
                "data2": rsa_encrypt(aes_key, public_key),
            }

            # URL for the second request
            url = f"http://{self.robot_ip}:9991/con_ing_{path_ending}"

            # Set the appropriate headers for URL-encoded form data
            headers = {'Content-Type': 'application/x-www-form-urlencoded'}

            # Send the encrypted data via POST
            response = make_local_request(
                url, body=json.dumps(body), headers=headers)

            # If response is successful, decrypt it
            if response:
                decrypted_response = aes_decrypt(response.text, aes_key)
                peer_answer = json.loads(decrypted_response)
                answer = RTCSessionDescription(
                    sdp=peer_answer['sdp'], type=peer_answer['type'])
                await self.pc.setRemoteDescription(answer)
            else:
                logger.info(
                    f"Failed to get answer from server: Reason: {response}")

        else:
            raise ValueError(
                "Failed to receive initial public key response with new method.")

    def validate_robot_conn(self, message):
        if message.get("data") == "Validation Ok.":

            # turn on video
            self.publish(
                "",
                "on",
                "vid",
            )

            self.validation_result = "SUCCESS"
            if self.on_validated:
                self.on_validated(self.robot_num)
        else:
            self.publish(
                "",
                self.encrypt_key(message.get("data")),
                "validation",
            )

    def publish(self, topic, data, msg_type):
        if self.data_channel.readyState != "open":
            logger.info(
                f"Data channel is not open. State is {self.data_channel.readyState}",
            )
            return

        payload = {
            "type": msg_type,
            "topic": topic,
            "data": data,
        }

        payload_dumped = json.dumps(payload)
        logger.info(f"-> Sending message {payload_dumped}")
        self.data_channel.send(payload_dumped)

    @staticmethod
    def hex_to_base64(hex_str):
        bytes_array = bytes.fromhex(hex_str)
        return base64.b64encode(bytes_array).decode("utf-8")

    @staticmethod
    def encrypt_key(key):
        prefixed_key = f"UnitreeGo2_{key}"
        encrypted = Go2Connection.encrypt_by_md5(prefixed_key)
        return Go2Connection.hex_to_base64(encrypted)

    @staticmethod
    def encrypt_by_md5(input_str):
        hash_obj = hashlib.md5()
        hash_obj.update(input_str.encode("utf-8"))
        return hash_obj.hexdigest()

    @staticmethod
    def deal_array_buffer(buffer, perform_decode=True):

        if isinstance(buffer, bytes):
            length = struct.unpack("H", buffer[:2])[0]
            json_segment = buffer[4: 4 + length]
            compressed_data = buffer[4 + length:]
            json_str = json_segment.decode("utf-8")
            obj = json.loads(json_str)
            if perform_decode:
                decoded_data = decoder.decode(compressed_data, obj['data'])
                obj["decoded_data"] = decoded_data
            else:
                obj["compressed_data"] = compressed_data
            return obj
        return None
