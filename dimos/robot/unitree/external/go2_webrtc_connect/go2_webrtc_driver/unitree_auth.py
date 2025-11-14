import hashlib
import time
import requests
import urllib.parse
import base64
import logging
import json
import sys
from Crypto.PublicKey import RSA
from .encryption import aes_encrypt, generate_aes_key, rsa_encrypt, aes_decrypt, rsa_load_public_key

def _calc_local_path_ending(data1):
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

def make_remote_request(path, body, token, method="GET"):
    # Constants
    APP_SIGN_SECRET = "XyvkwK45hp5PHfA8"
    UM_CHANNEL_KEY = "UMENG_CHANNEL"
    BASE_URL = "https://global-robot-api.unitree.com/"
    
    # Current timestamp and nonce
    app_timestamp = str(int(round(time.time() * 1000)))
    app_nonce = hashlib.md5(app_timestamp.encode()).hexdigest()
    
    # Generating app sign
    sign_str = f"{APP_SIGN_SECRET}{app_timestamp}{app_nonce}"
    app_sign = hashlib.md5(sign_str.encode()).hexdigest()
    
    # Get system's timezone offset in seconds and convert it to hours and minutes
    timezone_offset = time.localtime().tm_gmtoff // 3600
    minutes_offset = abs(time.localtime().tm_gmtoff % 3600 // 60)
    sign = "+" if timezone_offset >= 0 else "-"
    app_timezone = f"GMT{sign}{abs(timezone_offset):02d}:{minutes_offset:02d}"
    
    # Headers
    headers = {
        "Content-Type": "application/x-www-form-urlencoded",
        "DeviceId": "Samsung/GalaxyS20/SM-G981B/s20/10/29",
        "AppTimezone": app_timezone,
        "DevicePlatform": "Android",
        "DeviceModel": "SM-G981B",
        "SystemVersion": "29",
        "AppVersion": "1.8.0",
        "AppLocale": "en_US",
        "AppTimestamp": app_timestamp,
        "AppNonce": app_nonce,
        "AppSign": app_sign,
        "Channel": UM_CHANNEL_KEY,
        "Token": token,
        "AppName": "Go2"
    }
    
    # Full URL
    url = BASE_URL + path
    
    if method.upper() == "GET":
        # Convert body dictionary to query parameters for GET request
        params = urllib.parse.urlencode(body)
        response = requests.get(url, params=params, headers=headers)
    else:
        # URL-encode the body for POST request
        encoded_body = urllib.parse.urlencode(body)
        response = requests.post(url, data=encoded_body, headers=headers)

    # Return the response as JSON
    return response.json()

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

# Function to send SDP to peer and receive the answer
def send_sdp_to_remote_peer(serial: str, sdp: str, access_token: str, public_key: RSA.RsaKey) -> str:
    logging.info("Sending SDP to Go2...")
    aes_key = generate_aes_key()
    path = "webrtc/connect"
    body = {
        "sn": serial,
        "sk": rsa_encrypt(aes_key, public_key),
        "data": aes_encrypt(sdp, aes_key),
        "timeout": 5
    }
    response = make_remote_request(path, body, token=access_token, method="POST")
    if response.get("code") == 100:
        logging.info("Received SDP Answer from Go2!")
        return aes_decrypt(response['data'], aes_key)
    elif response.get("code") == 1000:
        print("Device not online")
        sys.exit(1)
    else:
        raise ValueError(f"Failed to receive SDP Answer: {response}")
    


def send_sdp_to_local_peer(ip, sdp):
    try:
        # Try the old method first
        logging.info("Trying to send SDP using the old method...")
        response = send_sdp_to_local_peer_old_method(ip, sdp)
        if response:
            logging.info("SDP successfully sent using the old method.")
            return response
        else:
            logging.warning("Old method failed, trying the new method...")
    except Exception as e:
        logging.error(f"An error occurred with the old method: {e}")
        logging.info("Falling back to the new method...")

    # Now try the new method after the old method has failed
    try:
        response = send_sdp_to_local_peer_new_method(ip, sdp)  # Use the new method here
        if response:
            logging.info("SDP successfully sent using the new method.")
            return response
        else:
            logging.error("New method failed to send SDP.")
            return None
    except Exception as e:
        logging.error(f"An error occurred with the new method: {e}")
        return None


def send_sdp_to_local_peer_old_method(ip, sdp):
    """
    Sends an SDP message to a local peer using an HTTP POST request.
    
    Args:
        ip (str): The IP address of the local peer to send the SDP message.
        sdp (dict): The SDP message to be sent in the request body.
        
    Returns:
        response: The response from the local peer if the request is successful, otherwise None.
    """
    try:
        # Define the URL for the POST request
        url = f"http://{ip}:8081/offer"

        # Define headers for the POST request
        headers = {'Content-Type': 'application/json'}
        
        # Send the POST request with the SDP body (convert the dict to JSON)
        response = make_local_request(url, body=sdp, headers=headers)
        
        # Check if the response is valid
        if response and response.status_code == 200:
            logging.debug(f"Recieved SDP: {response.text}")
            return response.text
        else:
            raise ValueError(f"Failed to receive SDP Answer: {response.status_code if response else 'No response'}")

    except requests.exceptions.RequestException as e:
        # Handle any exceptions that occur during the request
        logging.error(f"An error occurred while sending the SDP: {e}")
        return None

def send_sdp_to_local_peer_new_method(ip, sdp):
    try:
        url = f"http://{ip}:9991/con_notify"

        # Initial request to get public key information
        response = make_local_request(url, body=None, headers=None)
        
        # Check if the response status code is 200 (OK)
        if response:
            # Decode the response text from base64
            decoded_response = base64.b64decode(response.text).decode('utf-8')
            logging.debug(f"Recieved con_notify response: {decoded_response}")

            # Parse the decoded response as JSON
            decoded_json = json.loads(decoded_response)
            
            # Extract the 'data1' field from the JSON
            data1 = decoded_json.get('data1')

            # Extract the public key from 'data1'
            public_key_pem = data1[10:len(data1)-10]
            path_ending = _calc_local_path_ending(data1)

            # Generate AES key
            aes_key = generate_aes_key()

            # Load Public Key
            public_key = rsa_load_public_key(public_key_pem)

            # Encrypt the SDP and AES key
            body = {
                "data1": aes_encrypt(sdp, aes_key),
                "data2": rsa_encrypt(aes_key, public_key),
            }

            # URL for the second request
            url = f"http://{ip}:9991/con_ing_{path_ending}"

            # Set the appropriate headers for URL-encoded form data
            headers = {'Content-Type': 'application/x-www-form-urlencoded'}

            # Send the encrypted data via POST
            response = make_local_request(url, body=json.dumps(body), headers=headers)

            # If response is successful, decrypt it
            if response:
                decrypted_response = aes_decrypt(response.text, aes_key)
                logging.debug(f"Recieved con_ing_{path_ending} response: {decrypted_response}")
                return decrypted_response
        else:
            raise ValueError("Failed to receive initial public key response.")

    except requests.exceptions.RequestException as e:
        # Handle any exceptions that occur during the request
        logging.error(f"An error occurred while sending the SDP: {e}")
        return None
    except json.JSONDecodeError as e:
        # Handle JSON decoding errors
        logging.error(f"An error occurred while decoding JSON: {e}")
        return None
    except base64.binascii.Error as e:
        # Handle base64 decoding errors
        logging.error(f"An error occurred while decoding base64: {e}")
        return None


