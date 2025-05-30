from ..constants import app_error_messages
import time

def integer_to_hex_string(error_code):
    """
    Converts an integer error code to a hexadecimal string.
    
    Args:
        error_code (int): The error code as an integer.
        
    Returns:
        str: The error code as a hexadecimal string, without the '0x' prefix, in uppercase.
    """
    if not isinstance(error_code, int):
        raise ValueError("Input must be an integer.")

    # Convert the integer to a hex string and remove the '0x' prefix
    hex_string = hex(error_code)[2:].upper()

    return hex_string

def get_error_code_text(error_source, error_code):
    """
    Retrieve the error message based on the error source and error code.

    Args:
        error_code_dict (dict): Dictionary mapping error codes to messages.
        error_source (int): The error source code (e.g., 100, 200, etc.).
        error_code (str): The specific error code in string form (e.g., "01", "10").

    Returns:
        str: The corresponding error message, or the fallback format.
    """
    # Generate the key for looking up the error message
    key = f"app_error_code_{error_source}_{error_code}"
    
    # Check if the key exists in the error_code_dict
    if key in app_error_messages:
        return app_error_messages[key]
    else:
        # Fallback: return the combination of error_source and error_code
        return f"{error_source}-{error_code}"

def get_error_source_text(error_source):
    """
    Retrieve the error message based on the error source and error code.

    Args:
        error_code_dict (dict): Dictionary mapping error codes to messages.
        error_source (int): The error source code (e.g., 100, 200, etc.).
        error_code (str): The specific error code in string form (e.g., "01", "10").

    Returns:
        str: The corresponding error message, or the fallback format.
    """
    # Generate the key for looking up the error message
    key = f"app_error_source_{error_source}"
    
    # Check if the key exists in the error_code_dict
    if key in app_error_messages:
        return app_error_messages[key]
    else:
        # Fallback: return the combination of error_source and error_code
        return f"{error_source}"

def handle_error(message):
    """
    Handle the error message, print the time, error source, and error message.

    Args:
        message (dict): The error message containing the data field.
    """
    data = message["data"]

    for error in data:
        timestamp, error_source, error_code_int = error
        
        # Convert the timestamp to human-readable format
        readable_time = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(timestamp))

        error_source_text = get_error_source_text(error_source)
        
        # Convert the error code to a hexadecimal string
        error_code_hex = integer_to_hex_string(error_code_int)
        
        # Get the error message
        error_code_text = get_error_code_text(error_source, error_code_hex)

        print(f"\n🚨 Error Received from Go2:\n"
            f"🕒 Time:          {readable_time}\n"
            f"🔢 Error Source:  {error_source_text}\n"
            f"❗ Error Code:    {error_code_text}\n")
