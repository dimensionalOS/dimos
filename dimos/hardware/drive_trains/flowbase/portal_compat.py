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

"""Stops Portal's client socket thread spinning after a request races its wake-up.

``portal.ClientSocket._loop`` skips its ``poll()`` while ``writing`` is set and clears
``writing`` only inside ``if self.sendq:``. ``send()`` appends to the queue and then writes
a wake byte to a pipe. When the loop thread is between its ``poll()`` and its send block,
handling the previous reply, it sends the new buffer straight away and reads the wake byte
on the next pass with nothing left to send: ``writing`` stays True, ``poll()`` is skipped
from then on, and the thread spins on ``recv()`` raising ``BlockingIOError``, a whole core,
until the next request empties the queue again. A client polling a robot at 50 Hz is in
that state most of the time and stays in it when the calls stop: measured against the
FlowBase, 99.9% CPU while idle after one burst. Portal 3.7.4 and 3.8.1 share the loop.

``install()`` replaces the method with the same loop plus the one-line fix, on the Portal
versions whose loop this is a copy of. On any other version it leaves Portal alone and
warns, so an upstream fix is not papered over.
"""

from __future__ import annotations

from importlib import metadata
import os
import select
from typing import Any

from dimos.utils.logging_config import setup_logger

logger = setup_logger()

# The minor versions whose ClientSocket._loop the copy below is taken from.
PATCHED_PORTAL_MINORS = ("3.7", "3.8")

_installed = False


def portal_minor() -> str:
    version = metadata.version("portal")
    return ".".join(version.split(".")[:2])


def install() -> bool:
    """Patch the running Portal if its version is known. Returns whether it is patched."""
    global _installed
    if _installed:
        return True
    from portal import buffers, client_socket

    minor = portal_minor()
    if minor not in PATCHED_PORTAL_MINORS:
        logger.warning(
            "portal_compat knows no client loop for this Portal; its socket thread may spin",
            portal=metadata.version("portal"),
            patched=PATCHED_PORTAL_MINORS,
        )
        return False

    def _loop(self: Any) -> None:
        recvbuf = buffers.RecvBuffer(maxsize=self.options.max_msg_size)
        sock: Any = None
        poll = select.poll()
        poll.register(self.get_signal, select.POLLIN)
        isconn = False  # Local mirror of self.isconn without the lock.
        writing = False

        while self.running or (self.sendq and isconn):
            if not isconn:
                if not self.options.autoconn and not self.wantconn.wait(timeout=0.2):
                    continue
                sock = self._connect()
                if not sock:
                    break
                poll.register(sock, select.POLLIN)
                self.isconn.set()
                isconn = True
                if not self.options.autoconn:
                    self.wantconn.clear()
                for on_connect in self.callbacks_conn:
                    on_connect()

            try:
                if not writing:
                    fds = [fd for fd, _ in poll.poll(0.2)]
                    if self.get_signal in fds:
                        writing = True
                        os.read(self.get_signal, 1)

                try:
                    recvbuf.recv(sock)
                    if recvbuf.done():
                        if self.recvq.qsize() > self.options.max_recv_queue:
                            raise RuntimeError("Too many incoming messages enqueued")
                        msg = recvbuf.result()
                        self.recvq.put(msg)
                        for on_recv in self.callbacks_recv:
                            on_recv(msg)
                        recvbuf = buffers.RecvBuffer(maxsize=self.options.max_msg_size)
                except BlockingIOError:
                    pass

                if self.sendq:
                    try:
                        self.sendq[0].send(sock)
                        if self.sendq[0].done():
                            self.sendq.popleft()
                    except BlockingIOError:
                        pass
                    except ConnectionResetError:
                        # The server is gone but there may be replies left to read, so
                        # the socket stays open until recv() fails.
                        pass
                # The fix. An empty queue means there is nothing to write, whatever a
                # stale wake byte said; Portal only cleared this inside the branch above.
                writing = bool(self.sendq)

            except OSError as e:
                detail = f"{type(e).__name__}"
                detail = f"{detail}: {e}" if str(e) else detail
                self._log(f"Connection to server lost ({detail})")
                self.isconn.clear()
                isconn = False
                poll.unregister(sock)
                sock.close()
                # Portal drops the queue on disconnect: delivery is not tracked at this
                # level, so resending is the caller's business.
                self.sendq.clear()
                recvbuf = buffers.RecvBuffer(maxsize=self.options.max_msg_size)
                for on_disconnect in self.callbacks_disc:
                    on_disconnect()
                continue

        if sock:
            sock.close()

    client_socket.ClientSocket._loop = _loop
    _installed = True
    logger.info("Patched Portal's client socket loop", portal=metadata.version("portal"))
    return True
