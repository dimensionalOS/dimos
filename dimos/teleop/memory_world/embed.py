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


def _signal_group(process: subprocess.Popen[bytes], sig: int) -> None:
    """Signal the whole process group, falling back to the one child.

    The child is spawned with `start_new_session=True`, so its pid IS its group's. A
    launcher that forks rather than execs -- `nix run` does -- puts the real work in a
    grandchild, which a signal to the child alone never reaches.
    """
    if process.poll() is not None:
        return
    with contextlib.suppress(OSError, ProcessLookupError, PermissionError):
        os.killpg(os.getpgid(process.pid), sig)
        return
    with contextlib.suppress(OSError):
        process.send_signal(sig)


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
        """Stop the job, and everything it started."""
        with self._lock:
            self._terminated = True
            process = self._process
        if process is None:
            return
        _signal_group(process, signal.SIGTERM)

    def _run(
        self, command: list[str], config_text: str | None, adopt: Callable[[], None], run_id: int
    ) -> None:
        last = ""
        config_path: str | None = None
        process: subprocess.Popen[bytes] | None = None
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
            with self._lock:
                self._process = process
                if self._terminated:  # stop() came while the process was starting
                    process.terminate()
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
                if process.poll() is None:
                    _signal_group(process, signal.SIGTERM)
                with contextlib.suppress(subprocess.TimeoutExpired, OSError):
                    process.wait(timeout=10)
                # Anything that ignored the TERM. Without this the group outlives the
                # module, and what it outlives the module doing is writing to the
                # recording.
                if process.poll() is None:
                    _signal_group(process, signal.SIGKILL)
                    with contextlib.suppress(subprocess.TimeoutExpired, OSError):
                        process.wait(timeout=5)
            with self._lock:
                if self._process is process:  # a job started after "done" owns the handle now
                    self._process = None
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
