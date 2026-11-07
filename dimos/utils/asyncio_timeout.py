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

"""`asyncio.timeout` on every supported Python; the standard one needs 3.11."""

import asyncio
from collections.abc import AsyncIterator
import contextlib
import sys

if sys.version_info >= (3, 11):
    timeout = asyncio.timeout
else:

    @contextlib.asynccontextmanager
    async def timeout(delay: float) -> AsyncIterator[None]:
        """Cancel the enclosing task after `delay` seconds and raise TimeoutError."""
        task = asyncio.current_task()
        assert task is not None
        expired = False

        def expire() -> None:
            nonlocal expired
            expired = True
            task.cancel()

        handle = asyncio.get_running_loop().call_later(delay, expire)
        try:
            yield
        except asyncio.CancelledError:
            if expired:
                raise TimeoutError from None
            raise
        finally:
            handle.cancel()
