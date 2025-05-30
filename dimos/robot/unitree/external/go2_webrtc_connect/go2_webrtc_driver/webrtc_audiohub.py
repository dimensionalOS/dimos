import logging
import json
import base64
import time
import uuid
import os
import hashlib
from pydub import AudioSegment
from go2_webrtc_driver.constants import AUDIO_API
from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection
import asyncio

CHUNK_SIZE = 61440

class WebRTCAudioHub:
    def __init__(self, connection: Go2WebRTCConnection, logger: logging.Logger = None):
        self.logger = logger.getChild(self.__class__.__name__) if logger else logging.getLogger(self.__class__.__name__)
        self.conn = connection
        self.data_channel = None
        self._setup_data_channel()

    def _setup_data_channel(self):
        """Setup the WebRTC data channel for audio control"""
        if not self.conn.datachannel:
            self.logger.error("WebRTC connection not established")
            raise RuntimeError("WebRTC connection not established")
        self.data_channel = self.conn.datachannel

    async def get_audio_list(self):
        """Get list of available audio files"""
        response = await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['GET_AUDIO_LIST'],
                "parameter": json.dumps({})
            }
        )
        return response

    async def play_by_uuid(self, uuid):
        """Play audio by UUID"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['SELECT_START_PLAY'],
                "parameter": json.dumps({
                    'unique_id': uuid
                })
            }
        )

    async def pause(self):
        """Pause current audio playback"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['PAUSE'],
                "parameter": json.dumps({})
            }
        )

    async def resume(self):
        """Resume paused audio playback"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['UNSUSPEND'],
                "parameter": json.dumps({})
            }
        )

    async def set_play_mode(self, play_mode):
        """Set audio play mode (single_cycle, no_cycle, list_loop)"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['SET_PLAY_MODE'],
                "parameter": json.dumps({
                    'play_mode': play_mode
                })
            }
        )

    async def rename_record(self, uuid, new_name):
        """Rename an audio record"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['SELECT_RENAME'],
                "parameter": json.dumps({
                    'unique_id': uuid,
                    'new_name': new_name
                })
            }
        )

    async def delete_record(self, uuid):
        """Delete an audio record"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['SELECT_DELETE'],
                "parameter": json.dumps({
                    'unique_id': uuid
                })
            }
        )

    async def get_play_mode(self):
        """Get current play mode"""
        response = await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['GET_PLAY_MODE'],
                "parameter": json.dumps({})
            }
        )
        return response

    async def upload_audio_file(self, audiofile_path):
        """Upload audio file (MP3 or WAV)"""
        # Convert MP3 to WAV if necessary
        if audiofile_path.endswith(".mp3"):
            self.logger.info("Converting MP3 to WAV")
            audio = AudioSegment.from_mp3(audiofile_path)
            # Set specific audio parameters for compatibility
            audio = audio.set_frame_rate(44100)  # Standard sample rate
            wav_file_path = audiofile_path.replace('.mp3', '.wav')
            audio.export(wav_file_path, format='wav', parameters=["-ar", "44100"])
        else:
            wav_file_path = audiofile_path
        
        # Read the WAV file
        with open(wav_file_path, 'rb') as f:
            audio_data = f.read()

        # Generate a unique ID for the audio file
        unique_id = str(uuid.uuid4())
        
        try:
            # Calculate MD5 of the file
            file_md5 = hashlib.md5(audio_data).hexdigest()
            
            # Convert to base64
            b64_data = base64.b64encode(audio_data).decode('utf-8')
            
            # Split into smaller chunks (4KB each)
            chunk_size = 4096
            chunks = [b64_data[i:i + chunk_size] for i in range(0, len(b64_data), chunk_size)]
            total_chunks = len(chunks)
            
            self.logger.info(f"Splitting file into {total_chunks} chunks")

            # Send each chunk
            for i, chunk in enumerate(chunks, 1):
                parameter = {
                    'file_name': os.path.splitext(os.path.basename(audiofile_path))[0],
                    'file_type': 'wav',
                    'file_size': len(audio_data),
                    'current_block_index': i,
                    'total_block_number': total_chunks,
                    'block_content': chunk,
                    'current_block_size': len(chunk),
                    'file_md5': file_md5,
                    'create_time': int(time.time() * 1000)
                }
                print(json.dumps(parameter, ensure_ascii=True))
                # Send the chunk
                self.logger.info(f"Sending chunk {i}/{total_chunks}")
                
                response = await self.data_channel.pub_sub.publish_request_new(
                    "rt/api/audiohub/request",
                    {
                        "api_id": AUDIO_API['UPLOAD_AUDIO_FILE'],
                        "parameter": json.dumps(parameter, ensure_ascii=True)
                    }
                )
                
                # Wait a small amount between chunks
                await asyncio.sleep(0.1)
                
            self.logger.info("All chunks sent")
            return response
            
        except Exception as e:
            self.logger.error(f"Error uploading audio file: {e}")
            raise

    async def enter_megaphone(self):
        """Enter megaphone mode"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['ENTER_MEGAPHONE'],
                "parameter": json.dumps({})
            }
        )

    async def exit_megaphone(self):
        """Exit megaphone mode"""
        await self.data_channel.pub_sub.publish_request_new(
            "rt/api/audiohub/request",
            {
                "api_id": AUDIO_API['EXIT_MEGAPHONE'],
                "parameter": json.dumps({})
            }
        )

    async def upload_megaphone(self, audiofile_path):
        """Upload audio file (MP3 or WAV)"""
        # Convert MP3 to WAV if necessary
        if audiofile_path.endswith(".mp3"):
            self.logger.info("Converting MP3 to WAV")
            audio = AudioSegment.from_mp3(audiofile_path)
            # Set specific audio parameters for compatibility
            audio = audio.set_frame_rate(44100)  # Standard sample rate
            wav_file_path = audiofile_path.replace('.mp3', '.wav')
            audio.export(wav_file_path, format='wav', parameters=["-ar", "44100"])
        else:
            wav_file_path = audiofile_path

        # Read and chunk the WAV file
        with open(wav_file_path, 'rb') as f:
            audio_data = f.read()

        try:
            # Calculate MD5 of the file
            file_md5 = hashlib.md5(audio_data).hexdigest()
            
            # Convert to base64
            b64_data = base64.b64encode(audio_data).decode('utf-8')
            
            # Split into smaller chunks (4KB each)
            chunk_size = 4096
            chunks = [b64_data[i:i + chunk_size] for i in range(0, len(b64_data), chunk_size)]
            total_chunks = len(chunks)
            
            self.logger.info(f"Splitting file into {total_chunks} chunks")

            # Send each chunk
            for i, chunk in enumerate(chunks, 1):
                parameter = {
                    'current_block_size': len(chunk),
                    'block_content': chunk,
                    'current_block_index': i,
                    'total_block_number': total_chunks
                }
                print(json.dumps(parameter, ensure_ascii=True))
                # Send the chunk
                self.logger.info(f"Sending chunk {i}/{total_chunks}")
                
                response = await self.data_channel.pub_sub.publish_request_new(
                    "rt/api/audiohub/request",
                    {
                        "api_id": AUDIO_API['UPLOAD_MEGAPHONE'],
                        "parameter": json.dumps(parameter, ensure_ascii=True)
                    }
                )
                
                # Wait a small amount between chunks
                await asyncio.sleep(0.1)
                
            self.logger.info("All chunks sent")
            return response
        except Exception as e:
            self.logger.error(f"Error uploading audio file: {e}")
            raise