# Copyright 2026 Dimensional Inc.
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


"""MCP Bridge - Connects stdio (Claude Code) to TCP (DimOS Agent)."""

import asyncio
import os
import sys

DEFAULT_PORT = 9990


async def main() -> None:
    port = int(os.environ.get("MCP_PORT", DEFAULT_PORT))
    host = os.environ.get("MCP_HOST", "localhost")

    reader, writer = await asyncio.open_connection(host, port)
    sys.stderr.write(f"MCP Bridge connected to {host}:{port}\n")

    async def stdin_to_tcp() -> None:
        loop = asyncio.get_event_loop()
        while True:
            line = await loop.run_in_executor(None, sys.stdin.readline)
            if not line:
                break
            writer.write(line.encode())
            await writer.drain()

    async def tcp_to_stdout() -> None:
        while True:
            data = await reader.readline()
            if not data:
                break
            sys.stdout.write(data.decode())
            sys.stdout.flush()

    await asyncio.gather(stdin_to_tcp(), tcp_to_stdout())


if __name__ == "__main__":
    asyncio.run(main())
