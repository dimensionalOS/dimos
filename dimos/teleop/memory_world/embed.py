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
import json
import os
from pathlib import Path
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
            self.state, self.progress = "running", f"starting {self.name}"
        threading.Thread(
            target=self._run,
            args=(command, config_text, adopt),
            daemon=True,
            name="MemoryWorldEmbed",
        ).start()
        return True

    def terminate(self) -> None:
        with self._lock:
            self._terminated = True
            process = self._process
        if process is not None:
            process.terminate()

    def _run(self, command: list[str], config_text: str | None, adopt: Callable[[], None]) -> None:
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
            process = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
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
            with self._lock:
                if self._process is process:  # a job started after "done" owns the handle now
                    self._process = None
            if config_path is not None:
                Path(config_path).unlink(missing_ok=True)
            if self._on_finished is not None:
                self._on_finished(self)
