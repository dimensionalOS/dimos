from Crypto.Cipher import AES
from Crypto.PublicKey import RSA
from Crypto.Cipher import PKCS1_v1_5
import base64
import uuid
import binascii

###############
### AES handling
###############

# Function to generate a UUID and return it as a 32-character hexadecimal string
def _generate_uuid() -> str:
    uuid_32 = uuid.uuid4().bytes  
    uuid_32_hex_string = binascii.hexlify(uuid_32).decode('utf-8')
    return uuid_32_hex_string

def pad(data: str) -> bytes:
    """Pad data to be a multiple of 16 bytes (AES block size)."""
    block_size = AES.block_size
    padding = block_size - len(data) % block_size
    padded_data = data + chr(padding) * padding
    return padded_data.encode('utf-8')

def unpad(data: bytes) -> str:
    """Remove padding from data."""
    padding = data[-1]
    return data[:-padding].decode('utf-8')

def aes_encrypt(data: str, key: str) -> str:
    """Encrypt the given data using AES (ECB mode with PKCS5 padding)."""
    # Ensure key is 32 bytes for AES-256
    key_bytes = key.encode('utf-8')

    # Pad the data to ensure it is a multiple of block size
    padded_data = pad(data)

    # Create AES cipher in ECB mode
    cipher = AES.new(key_bytes, AES.MODE_ECB)

    # Encrypt data
    encrypted_data = cipher.encrypt(padded_data)

    # Encode encrypted data to Base64
    encoded_encrypted_data = base64.b64encode(encrypted_data).decode('utf-8')

    return encoded_encrypted_data

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

# Function to generate an AES key
def generate_aes_key() -> str:
    return _generate_uuid()

###############
### RSA handling
###############

def rsa_load_public_key(pem_data: str) -> RSA.RsaKey:
    """Load an RSA public key from a PEM-formatted string."""
    key_bytes = base64.b64decode(pem_data)
    return RSA.import_key(key_bytes)

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

# Example usage
if __name__ == "__main__":
    # Public key
    public_key_pem = """
    MIIBIjANBgkqhkiG9w0BAQEFAAOCAQ8AMIIBCgKCAQEAnOc1sgpzL4GTVp9/oQ0H
    D7eeAO2GJUABfjX3TitgXiXN1Ktn2WLsLrtAiIuj3OrrRogx8fCT16oxnXx/Xrap
    BRHD/ufHZ08A2IRVw6U6vKDv8TpQH22sAEtUji4/P2AaZmeOxFsYW5FshQr37KBG
    +cBb7rJWLWEJpIXmCpnt37GGCtsACqRegkl7qQ8Q0OiJmtrYLPi00xSstZb+Wv1v
    8B0eTY3POAUXjgl357L5dc6vS99rYFkYeUCTWHaH4d51Z/KgCRYUadboDc2cgNg/
    z2dbO9S3HADegbIsN3fTbjDCruKfvc5ejxlFZ0Xbu6SScQbmkP8t3TPvy/DXGJAh
    NwIDAQAB
    """

    # Example value of UUID or data you wish to encrypt
    value_of = "26a663562a6f4dfbbbbf2b50c1a278cb"

    # Load public key
    public_key = rsa_load_public_key(public_key_pem)

    # Encrypt the message
    encrypted_value = rsa_encrypt(value_of, public_key)
    print(f"Encrypted Value: {encrypted_value}")

    # AES testing
    aes_key = "26a663562a6f4dfbbbbf2b50c1a278cb"  # Example 32-character UUID

    # Encrypt a message with AES
    encrypted_message = aes_encrypt("Hello, world!", aes_key)
    print(f"Encrypted AES Message: {encrypted_message}")

    # Decrypt the AES message
    decrypted_message = aes_decrypt(encrypted_message, aes_key)
    print(f"Decrypted AES Message: {decrypted_message}")
