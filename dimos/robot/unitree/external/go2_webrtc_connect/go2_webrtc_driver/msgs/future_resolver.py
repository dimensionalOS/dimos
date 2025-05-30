import logging
from ..constants import DATA_CHANNEL_TYPE
from ..util import get_nested_field

class FutureResolver:
    def __init__(self):
        self.pending_responses = {}
        self.pending_callbacks = {}
        self.chunk_data_storage = {}

    def save_resolve(self, message_type, topic, future, identifier):
        key = self.generate_message_key(message_type,topic,identifier)
        if key in self.pending_callbacks:
            self.pending_callbacks[key].append(future)
        else:
            self.pending_callbacks[key] = [future]

    def run_resolve_for_topic(self, message):
        if not message.get("type"):
            return

        if message["type"] == DATA_CHANNEL_TYPE["RTC_INNER_REQ"] and get_nested_field(message, "info", "req_type") == "request_static_file":
            self.run_resolve_for_topic_for_file(message)
            return

        key = self.generate_message_key(
            message["type"],
            message.get("topic", ""),
            get_nested_field(message, "data", "uuid") or
            get_nested_field(message, "data", "header", "identity", "id") or
            get_nested_field(message, "info", "uuid") or
            get_nested_field(message, "info", "req_uuid")
        )

        content_info = get_nested_field(message, "data", "content_info")
        if content_info and content_info.get("enable_chunking"):
            chunk_index = content_info.get("chunk_index")
            total_chunks = content_info.get("total_chunk_num")

            if total_chunks is None or total_chunks == 0:
                raise ValueError("Total number of chunks cannot be zero")
            if chunk_index is None:
                raise ValueError("Chunk index is missing")

            data_chunk = message["data"].get("data")
            if chunk_index < total_chunks:
                if key in self.chunk_data_storage:
                    self.chunk_data_storage[key].append(data_chunk)
                else:
                    self.chunk_data_storage[key] = [data_chunk]
                return
            else:
                self.chunk_data_storage[key].append(data_chunk)
                message["data"]["data"] = self.merge_array_buffers(self.chunk_data_storage[key])
                del self.chunk_data_storage[key]

        # Resolve the pending future with the final message
        if key in self.pending_callbacks:
            for future in self.pending_callbacks[key]:
                if future:
                    future.set_result(message)  # Resolve the future with the message
            del self.pending_callbacks[key]

    def merge_array_buffers(self, buffers):
        total_length = sum(len(buf) for buf in buffers)
        merged_buffer = bytearray(total_length)

        current_position = 0
        for buffer in buffers:
            merged_buffer[current_position:current_position + len(buffer)] = buffer
            current_position += len(buffer)

        return bytes(merged_buffer)

    def run_resolve_for_topic_for_file(self, message):
        key = self.generate_message_key(
            message["type"], 
            message.get("topic", ""), 
            get_nested_field(message, "data", "uuid") or
            get_nested_field(message, "data", "header", "identity", "id") or
            get_nested_field(message, "info", "uuid") or
            get_nested_field(message, "info", "req_uuid")
        )

        file_info = get_nested_field(message, "info", "file")
        if file_info and file_info.get("enable_chunking"):
            chunk_index = file_info.get("chunk_index")
            total_chunks = file_info.get("total_chunk_num")

            if total_chunks is None or total_chunks == 0:
                raise ValueError("Total number of chunks cannot be zero")
            if chunk_index is None:
                raise ValueError("Chunk index is missing")

            # Extract the chunk data
            data_chunk = file_info.get("data")

            # Initialize the key in chunk_data_storage if it doesn't exist
            if key not in self.chunk_data_storage:
                self.chunk_data_storage[key] = []

            # Append the chunk to the storage, ensuring it's in bytes
            self.chunk_data_storage[key].append(data_chunk.encode('utf-8') if isinstance(data_chunk, str) else data_chunk)

            # If this is the last chunk, combine all chunks and store the complete data
            if chunk_index == total_chunks:
                message["info"]["file"]["data"] = b''.join(self.chunk_data_storage[key])
                del self.chunk_data_storage[key]  # Clean up the storage

        # Resolve the pending future with the final message
        if key in self.pending_callbacks:
            for future in self.pending_callbacks[key]:
                if future:
                    future.set_result(message)  # Resolve the future with the message
            del self.pending_callbacks[key]

    def generate_message_key(self, message_type, topic, identifier):
        return identifier or f"{message_type} $ {topic}"


