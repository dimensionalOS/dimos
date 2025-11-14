import hashlib
import json
import logging
import random
import requests
import time
import sys
from Crypto.PublicKey import RSA
from .unitree_auth import make_remote_request
from .encryption import rsa_encrypt, rsa_load_public_key, aes_decrypt, generate_aes_key

# Function to generate MD5 hash of a string

def _generate_md5(string: str) -> str:
    md5_hash = hashlib.md5(string.encode())
    return md5_hash.hexdigest()

def generate_uuid():
    def replace_char(char):
        rand = random.randint(0, 15)
        if char == "x":
            return format(rand, 'x')
        elif char == "y":
            return format((rand & 0x3) | 0x8, 'x')

    uuid_template = "xxxxxxxx-xxxx-4xxx-yxxx-xxxxxxxxxxxx"
    return ''.join(replace_char(char) if char in 'xy' else char for char in uuid_template)


def get_nested_field(message, *fields):
    current_level = message
    for field in fields:
        if isinstance(current_level, dict) and field in current_level:
            current_level = current_level[field]
        else:
            return None
    return current_level

# Function to obtain a fresh token from the backend server
def fetch_token(email: str, password: str) -> str:
    logging.info("Obtaining TOKEN...")
    path = "login/email"
    body = {
        'email': email,
        'password': _generate_md5(password)
    }
    response = make_remote_request(path, body, token="", method="POST")
    if response.get("code") == 100:
        data = response.get("data")
        access_token = data.get("accessToken")
        return access_token
    else:
        logging.error("Failed to receive token")
        return None


# Function to obtain a public key
def fetch_public_key() -> RSA.RsaKey:
    logging.info("Obtaining a Public key...")
    path = "system/pubKey"
    
    try:
        # Attempt to make the request
        response = make_remote_request(path, {}, token="", method="GET")

        if response.get("code") == 100:
            public_key_pem = response.get("data")
            return rsa_load_public_key(public_key_pem)
        else:
            logging.error("Failed to receive public key")
            return None

    except requests.exceptions.ConnectionError as e:
        # Handle no internet connection or other connection errors
        logging.warning("No internet connection or failed to connect to the server. Unable to fetch public key.")
        logging.error(f"Connection error: {e}")
        return None
    except requests.exceptions.RequestException as e:
        # Handle other request exceptions
        logging.error(f"An error occurred while fetching the public key: {e}")
        return None


# Function to obtain TURN server info
def fetch_turn_server_info(serial: str, access_token: str, public_key: RSA.RsaKey) -> dict:
    logging.info("Obtaining TURN server info...")
    aes_key = generate_aes_key()
    path = "webrtc/account"
    body = {
        "sn": serial,
        "sk": rsa_encrypt(aes_key, public_key)
    }
    response = make_remote_request(path, body, token=access_token, method="POST")
    if response.get("code") == 100:
        return json.loads(aes_decrypt(response['data'], aes_key))
    else:
        logging.error("Failed to receive TURN server info")
        return None

def print_status(status_type, status_message):
    current_time = time.strftime("%H:%M:%S")
    print(f"🕒 {status_type:<25}: {status_message:<15} ({current_time})")


