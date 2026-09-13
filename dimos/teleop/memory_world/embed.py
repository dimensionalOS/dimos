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

"""Adding SigLIP embeddings to a recording that has none, by running siglipify.

siglipify (``github:jeff-hykin/siglipify``) embeds a recording's image stream
and writes the vectors back into it: a mem2 ``.db`` gains a stream, an
``.mcap`` is rewritten with a new topic. It names that stream after the image
stream and the model, which is exactly the stream the visual index looks for,
so nothing else has to agree on a name. The tool is fetched and run with
``nix run``; the first run also builds it, which shows up as nix output.
"""

from __future__ import annotations

from collections.abc import Callable
import contextlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import threading
import time

from dimos.utils.logging_config import setup_logger

logger = setup_logger()


def siglipify_config(model_name: str, image_stream_name: str, stride: int) -> str:
    """The TOML siglipify reads: per-patch vectors of one image stream, every *stride*-th frame."""
    return (
        f"model = {json.dumps(model_name)}\n"
        'embedding = "patches"\n'
        "batch = 8\n"
        f"stride = {int(stride)}\n"
        f"streams = [{json.dumps(image_stream_name)}]\n"
    )


def siglipify_command(flake: str, store_path: str) -> list[str]:
    """The job appends the config file's path (see EmbeddingJob.start)."""
    return ["nix", "run", flake, "--", "run", store_path, "--config"]


# How long the group is given to go on its own before it is killed outright.
TERMINATE_GRACE_S = 2.0


def _signal_group(pgid: int, sig: int) -> None:
    """Signal a whole process group by its id.

    By ID, not by `Popen`. The child is spawned with `start_new_session=True`, so its pid
    IS its group's, and the group outlives it: a launcher that forks rather than execs --
    `nix run` does -- can exit 0 while the work it started runs on. Asking `os.getpgid` on
    a reaped launcher raises, and guarding that with `process.poll() is not None: return`
    meant the surviving worker was never signalled at all. The id is taken once, at spawn,
    and stays valid for as long as any member of the group is alive.
    """
    with contextlib.suppress(OSError, ProcessLookupError, PermissionError):
        os.killpg(pgid, sig)


def _group_alive(pgid: int) -> bool:
    """Whether any member of the group is still there."""
    try:
        os.killpg(pgid, 0)
    except ProcessLookupError:
        return False
    except (OSError, PermissionError):
        return True  # it exists; we merely may not signal it
    return True


class EmbeddingJob:
    """One background run of siglipify, with its state kept for the viewer.

    ``state`` is idle, running, done or failed; ``progress`` is the tool's
    latest output line. *on_finished* runs after every attempt, success or
    not, on the job's thread.
    """

    def __init__(
        self,
        on_finished: Callable[[EmbeddingJob], None] | None = None,
        name: str = "siglipify",
        done: str = "embeddings added",
    ) -> None:
        self.name = name
        self.done_message = done
        self._lock = threading.Lock()
        self.state = "idle"
        # Which run owns the shared state. See the finally block in `_run`.
        self._run_id = 0
        self.progress = ""
        self._process: subprocess.Popen[str] | None = None
        self._pgid: int | None = None
        self._terminated = False
        self._on_finished = on_finished

    def status(self) -> dict[str, str]:
        with self._lock:
            return {"embedding": self.state, "progress": self.progress}

    def _set(self, state: str, progress: str) -> None:
        with self._lock:
            self.state = state
            self.progress = progress

    def start(self, command: list[str], config_text: str | None, adopt: Callable[[], None]) -> bool:
        """Run *command* unless already running. With *config_text*, it is written
        to a temp file whose path becomes the command's last argument.

        *adopt* runs after a successful exit, before the job is marked done;
        it is where the caller picks up what the tool wrote.
        """
        with self._lock:
            if self.state == "running":
                return False
            self._terminated = False
            self._run_id += 1
            run_id = self._run_id
            self.state, self.progress = "running", f"starting {self.name}"
        threading.Thread(
            target=self._run,
            args=(command, config_text, adopt, run_id),
            daemon=True,
            name="MemoryWorldEmbed",
        ).start()
        return True

    def terminate(self) -> None:
        """Stop the job, and everything it started. Returns once it is actually gone."""
        with self._lock:
            self._terminated = True
            pgid = self._pgid
        if pgid is not None:
            self._stop_group(pgid)

    @staticmethod
    def _stop_group(pgid: int) -> None:
        """TERM the group, give it a moment, then KILL what is left.

        The escalation belongs here, not only in `_run`'s finally. The read loop blocks on
        a pipe every member of the group holds open, so a process that ignores the TERM
        never lets that thread reach its finally at all: measured, a child with SIGTERM
        set to SIG_IGN was still alive and the job still reporting "running" twelve
        seconds after a terminate() that was supposed to end it.
        """
        _signal_group(pgid, signal.SIGTERM)
        deadline = time.monotonic() + TERMINATE_GRACE_S
        while time.monotonic() < deadline:
            if not _group_alive(pgid):
                return
            time.sleep(0.05)
        _signal_group(pgid, signal.SIGKILL)

    def _run(
        self, command: list[str], config_text: str | None, adopt: Callable[[], None], run_id: int
    ) -> None:
        last = ""
        config_path: str | None = None
        process: subprocess.Popen[bytes] | None = None
        pgid: int | None = None
        try:
            if config_text is not None:
                with tempfile.NamedTemporaryFile("w", suffix=".toml", delete=False) as handle:
                    handle.write(config_text)
                    config_path = handle.name
                command = [*command, config_path]
            logger.info("%s: %s", self.name, " ".join(command))
            # Its own process group, so `terminate()` can stop the whole tree. The real
            # command is `nix run <flake> -- run <recording> ...`, and `nix run` commonly
            # forks into the built program rather than exec'ing it -- so signalling the
            # immediate child alone left siglipify running, still WRITING INTO THE
            # RECORDING, after the module reported itself stopped. The read loop below
            # blocks on the pipe, which the grandchild still holds open, so the job also
            # went on reporting "running" for as long as the work it no longer controlled
            # took. Measured: a stop at 0.5 s had no effect at all until the grandchild
            # finished on its own six seconds later.
            process = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
            # This run's own group id, held as a LOCAL as well as published. `process` is
            # already kept locally for exactly this reason: the instance field belongs to
            # whichever run is CURRENT, and by the time the cleanup below runs that can be
            # a later one. Reading it there killed the successor -- B started in the window
            # after A published "done", and A's finally then signalled B's group: B died
            # with -15 and never ran the adoption its embeddings needed. The same ownership
            # mistake the `_run_id` guard was added for, one field along.
            pgid = process.pid
            with self._lock:
                self._process = process
                self._pgid = pgid
                if self._terminated:  # stop() came while the process was starting
                    # Group-wide and escalating, like `terminate()`. A stop landing in
                    # this window used to send a bare TERM to the immediate child and
                    # nothing else, so one that ignores it stayed alive and the job went
                    # on reporting "running".
                    self._stop_group(pgid)
            assert process.stdout is not None
            # Progress bars redraw with a carriage return, so split on both. os.read
            # returns what is there instead of waiting to fill a buffer.
            pending = ""
            fd = process.stdout.fileno()
            for chunk in iter(lambda: os.read(fd, 4096).decode("utf-8", "replace"), ""):
                pending += chunk
                *lines, pending = pending.replace("\r", "\n").split("\n")
                for line in lines:
                    line = line.strip()
                    if line:
                        last = line[-160:]
                        self._set("running", last)
                        logger.info("%s: %s", self.name, line)
            # Whatever is left when the pipe closes: a process that dies mid-line ends
            # without a newline, and that unterminated last line is the one saying why.
            tail = pending.strip()
            if tail:
                last = tail[-160:]  # `last` is what the RuntimeError below reports
                self._set("running", last)
                logger.info("%s: %s", self.name, tail)
            code = process.wait()
            if code != 0:
                raise RuntimeError(f"{self.name} exited with {code}: {last}")
            adopt()
            self._set("done", self.done_message)
        except Exception as error:
            logger.exception("%s failed", self.name)
            self._set("failed", str(error)[-200:])
        finally:
            if process is not None:
                # The read loop above can leave this child alive and unreaped: anything it
                # raises skips the process.wait() on the success path, and then nothing
                # else waits on it. Closing the pipe first makes the child's next write
                # fail, so one that ignores the terminate still finishes on its own.
                if process.stdout is not None:
                    with contextlib.suppress(OSError):
                        process.stdout.close()
                # `pgid`, the local: THIS run's group, never whichever run is current.
                if pgid is not None:
                    _signal_group(pgid, signal.SIGTERM)
                with contextlib.suppress(subprocess.TimeoutExpired, OSError):
                    process.wait(timeout=10)
                # Anything that ignored the TERM. Without this the group outlives the
                # module, and what it outlives the module doing is writing to the
                # recording.
                if pgid is not None and _group_alive(pgid):
                    _signal_group(pgid, signal.SIGKILL)
                    with contextlib.suppress(subprocess.TimeoutExpired, OSError):
                        process.wait(timeout=5)
            with self._lock:
                if self._process is process:  # a job started after "done" owns the handle now
                    self._process = None
                    self._pgid = None
                # A thread that leaves any other way leaves the state at "running" -- and
                # `start()` refuses to run anything while it reads that, so the job would
                # be dead, the viewer would wait for it forever, and no later attempt
                # could replace it. `except Exception` does not cover SystemExit or
                # KeyboardInterrupt, and the handler can raise on its own besides.
                #
                # `run_id`, not the state alone and not `_process`: this method publishes
                # "done" BEFORE it reaches here, so a second job can start in that window
                # -- and it sets `_process` only once its own subprocess has spawned, so
                # for a moment the handle above still says this run owns it while the new
                # one is already "running". Flipping the state then reports the LIVE job
                # as failed and lets a third start beside it. The counter is taken under
                # the same lock `start()` bumps it under, so it cannot be stale.
                if self._run_id == run_id and self.state == "running":
                    self.state = "failed"
                    self.progress = f"{self.name} stopped without saying why"
            if config_path is not None:
                Path(config_path).unlink(missing_ok=True)
            if self._on_finished is not None:
                self._on_finished(self)
