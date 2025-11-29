# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Shared memory registry for tracking actor deployments across processes."""

import json
from multiprocessing import shared_memory


class ActorRegistry:
    """Shared memory registry of actor deployments."""

    SHM_NAME = "dimos_actor_registry"
    SHM_SIZE = 65536  # 64KB should be enough for most deployments

    @staticmethod
    def update(actor_name: str, worker_id: str) -> None:
        """Update registry with new actor deployment."""
        try:
            shm = shared_memory.SharedMemory(name=ActorRegistry.SHM_NAME)
        except FileNotFoundError:
            shm = shared_memory.SharedMemory(
                name=ActorRegistry.SHM_NAME, create=True, size=ActorRegistry.SHM_SIZE
            )

        # Read existing data
        data = ActorRegistry._read_from_shm(shm)

        # Update with new actor
        data[actor_name] = worker_id

        # Write back
        ActorRegistry._write_to_shm(shm, data)
        shm.close()

    @staticmethod
    def get_all() -> dict[str, str]:
        """Get all actor->worker mappings."""
        try:
            shm = shared_memory.SharedMemory(name=ActorRegistry.SHM_NAME)
            data = ActorRegistry._read_from_shm(shm)
            shm.close()
            return data
        except FileNotFoundError:
            return {}

    @staticmethod
    def clear() -> None:
        """Clear the registry and free shared memory."""
        try:
            shm = shared_memory.SharedMemory(name=ActorRegistry.SHM_NAME)
            ActorRegistry._write_to_shm(shm, {})
            shm.close()
            shm.unlink()
        except FileNotFoundError:
            pass

    @staticmethod
    def _read_from_shm(shm) -> dict[str, str]:  # type: ignore[no-untyped-def]
        """Read JSON data from shared memory."""
        raw = bytes(shm.buf[:]).rstrip(b"\x00")
        if not raw:
            return {}
        return json.loads(raw.decode("utf-8"))  # type: ignore[no-any-return]

    @staticmethod
    def _write_to_shm(shm, data: dict[str, str]):  # type: ignore[no-untyped-def]
        """Write JSON data to shared memory."""
        json_bytes = json.dumps(data).encode("utf-8")
        if len(json_bytes) > ActorRegistry.SHM_SIZE:
            raise ValueError("Registry data too large for shared memory")
        shm.buf[: len(json_bytes)] = json_bytes
        shm.buf[len(json_bytes) :] = b"\x00" * (ActorRegistry.SHM_SIZE - len(json_bytes))
