import asyncio
import logging
import base64
from ..constants import DATA_CHANNEL_TYPE, WebRTCConnectionMethod
from ..util import generate_uuid

class WebRTCChannelProbeResponse:
    def __init__(self, channel, pub_sub):
        self.channel = channel
        self.publish = pub_sub.publish_without_callback
        
    def handle_response(self, info):
        self.publish(
            "",
            info,
            DATA_CHANNEL_TYPE["RTC_INNER_REQ"],
        )

class WebRTCDataChannelNetworkStatus:
    def __init__(self, conn, channel, pub_sub):
        self.conn = conn
        self.channel = channel
        self.publish = pub_sub.publish
        self.network_timer = None
        self.network_status = ""
        self.on_network_status_callbacks = []
    
    def set_on_network_status_callback(self, callback):
        """Register a callback to be called upon validation."""
        if callback and callable(callback):
            self.on_network_status_callbacks.append(callback)

    def start_network_status_fetch(self):
        """Start sending network status requests every 1 second."""
        self.network_timer = asyncio.get_event_loop().call_later(1, self.schedule_network_status_request)
    
    def stop_network_status_fetch(self):
        """Stop the network status fetch."""
        if self.network_timer:
            self.network_timer.cancel()
            self.network_timer = None
    
    def schedule_network_status_request(self):
        """Schedule the next network status request."""
        asyncio.create_task(self.send_network_status_request())

    async def send_network_status_request(self):
        """Send a network status request."""
        data = {
            "req_type": "public_network_status",
            "uuid": generate_uuid()
        }
        try:
            response = await self.publish(
                "",
                data,
                DATA_CHANNEL_TYPE["RTC_INNER_REQ"],
            )
            self.handle_response(response.get("info"))
        except Exception as e:
            logging.error("Failed to publish:", e)
        
    def handle_response(self, info):
        """Handle a received network status message."""
        logging.info("Network status message received.")
        status = info.get("status")
        if status == "Undefined" or status == "NetworkStatus.DISCONNECTED":
            # Schedule the next network status request in 0.5s
            self.network_timer = asyncio.get_event_loop().call_later(0.5, self.schedule_network_status_request)

        elif status == "NetworkStatus.ON_4G_CONNECTED":
            self.network_status = "4G"
            self.stop_network_status_fetch()
        elif status == "NetworkStatus.ON_WIFI_CONNECTED":
            if self.conn.connectionMethod == WebRTCConnectionMethod.Remote:
                self.network_status = "STA-T"
            else:
                self.network_status = "STA-L"
        
        if status == "NetworkStatus.ON_4G_CONNECTED" or status == "NetworkStatus.ON_WIFI_CONNECTED":
            for callback in self.on_network_status_callbacks:
                callback(self.network_status)
            self.stop_network_status_fetch()


class WebRTCDataChannelFileUploader:
    
    def __init__(self, channel, pub_sub):
        self.channel = channel
        self.publish = pub_sub.publish
        self.cancel_upload = False
    
    def slice_base64_into_chunks(self, data, chunk_size):
        """Slices the base64 data into chunks of the given size."""
        return [data[i:i + chunk_size] for i in range(0, len(data), chunk_size)]
    
    
    async def upload_file(self, data, file_path, chunk_size=60*1024, progress_callback=None):
        """Uploads a file in chunks with the possibility to cancel the upload."""
        
        # Encode the data to Base64
        encoded_data = base64.b64encode(data).decode('utf-8')
        
        print("Total size after Base64 encoding:", len(encoded_data))
        chunks = self.slice_base64_into_chunks(encoded_data, chunk_size)
        total_chunks = len(chunks)
        
        self.cancel_upload = False
        
        for i, chunk in enumerate(chunks):
            if self.cancel_upload:
                print("Upload canceled.")
                return "cancel"
            
            if i % 5 == 0:
                await asyncio.sleep(0.5)
            
            uuid = generate_uuid()
            req_uuid = f"upload_req_{uuid}"
            
            message = {
                "req_type": "push_static_file",
                "req_uuid": req_uuid,
                "related_bussiness": "uslam_final_pcd",
                "file_md5": "null",
                "file_path": file_path,
                "file_size_after_b64": len(encoded_data),
                "file": {
                    "chunk_index": i + 1,
                    "total_chunk_num": total_chunks,
                    "chunk_data": chunk,
                    "chunk_data_size": len(chunk)
                }
            }
            
            self.publish("", message, DATA_CHANNEL_TYPE["RTC_INNER_REQ"])
            
            if progress_callback:
                progress_callback(int(((i + 1) / total_chunks) * 100))
        
        return "ok"
    
    def cancel(self):
        """Cancel the ongoing upload."""
        self.cancel_upload = True


class WebRTCDataChannelFileDownloader:
    
    def __init__(self, channel, pub_sub):
        self.channel = channel
        self.publish = pub_sub.publish
        self.cancel_download = False
        self.chunk_data_storage = {}

    async def download_file(self, file_path, chunk_size=60*1024, progress_callback=None):
        """Downloads a file in chunks with the possibility to cancel the download."""
        self.cancel_download = False

        try:
            uuid = generate_uuid()

            # Send the request to download the file
            request_message = {
                "req_type": "request_static_file",
                "req_uuid": f"req_{uuid}",
                "related_bussiness": "uslam_final_pcd",
                "file_md5": "null",
                "file_path": file_path
            }
            response = await self.publish("", request_message, DATA_CHANNEL_TYPE["RTC_INNER_REQ"])

            # Check if the download was canceled
            if self.cancel_download:
                logging.info("Download canceled.")
                return "cancel"
            
            # Extract the complete data after all chunks have been combined in the resolver
            complete_data = response.get("info", {}).get("file", {}).get("data")

            if not complete_data:
                logging.error("Failed to get the file data.")
                return "error"
            
            # Decode the Base64-encoded data
            decoded_data = base64.b64decode(complete_data)

            # Call progress_callback with 100% progress since the download is complete
            if progress_callback:
                progress_callback(100)

            return decoded_data

        except Exception as e:
            logging.error("Failed to download file:", e)
            return "error"

def cancel(self):
    """Cancel the ongoing download."""
    self.cancel_download = True

class WebRTCDataChannelRTCInnerReq:
    def __init__(self, conn, channel, pub_sub):
        self.conn = conn
        self.channel = channel

        self.network_status = WebRTCDataChannelNetworkStatus(self.conn, self.channel, pub_sub)
        self.probe_res = WebRTCChannelProbeResponse(self.channel, pub_sub)
    
    def handle_response(self, msg):
        """Handle a received network status message."""
        info = msg.get("info")
        req_type = info.get("req_type")
        if req_type == 'rtt_probe_send_from_mechine':
            self.probe_res.handle_response(info)

    
