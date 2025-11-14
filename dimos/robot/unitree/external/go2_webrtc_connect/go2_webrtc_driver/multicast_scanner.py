import socket
import struct
import json
import logging

RECV_PORT = 10134  # Port where the devices will send the multicast responses
MULTICAST_GROUP = '231.1.1.1'  # Multicast group IP address
MULTICAST_PORT = 10131  # Port to send multicast query to devices

def discover_ip_sn(timeout=2):
    print("Discovering devices on the network...")

    # Use a dictionary to store the serial number to IP mapping
    serial_to_ip = {}

    # Create a UDP socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

    # Bind the socket to the port
    sock.bind(('', RECV_PORT))

    # Tell the operating system to add the socket to the multicast group
    # on all interfaces.
    mreq = struct.pack("4sl", socket.inet_aton(MULTICAST_GROUP), socket.INADDR_ANY)
    sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

    # Send a multicast query to discover devices
    query_message = json.dumps({"name": "unitree_dapengche"})
    
    try:
        sock.sendto(query_message.encode('utf-8'), (MULTICAST_GROUP, MULTICAST_PORT))
    except Exception as e:
        logging.error(f"Error sending multicast query: {e}")
        sock.close()
        return serial_to_ip

    # Set a timeout for receiving responses
    sock.settimeout(timeout)

    try:
        while True:
            # Receive the response from the device
            data, addr = sock.recvfrom(1024)
            message = data.decode('utf-8')
            # Convert the JSON message to a dictionary
            message_dict = json.loads(message)
            if "sn" in message_dict:
                serial_number = message_dict["sn"]
                ip_address = message_dict.get("ip", addr[0])
                serial_to_ip[serial_number] = ip_address
                print(f"Discovered device: {serial_number} at {ip_address}")
    except socket.timeout:
        logging.info("Timeout reached, stopping listening.")
    except json.JSONDecodeError as e:
        logging.error(f"Error decoding JSON message: {e}")
    except Exception as e:
        logging.error(f"An error occurred: {e}")
    finally:
        # Close the socket
        sock.close()

    return serial_to_ip

if __name__ == '__main__':
    print("Discovering devices on the network...")
    serial_to_ip = discover_ip_sn(timeout=3)
    print("\nDiscovered devices:")
    for serial_number, ip_address in serial_to_ip.items():
        print(f"Serial Number: {serial_number}, IP Address: {ip_address}")
