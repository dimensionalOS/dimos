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

"""Python execution for agents working with a live robot memory recording."""

from __future__ import annotations

import subprocess
import sys
from typing import Annotated, ClassVar

from pydantic import Field

from dimos.agents.annotation import skill
from dimos.core.module import Module, ModuleConfig
from dimos.memory.module import Recorder

DEFAULT_MAX_OUTPUT_CHARS = 12_000
MAX_TIMEOUT_SECONDS = 100.0

_PYTHON_BOOTSTRAP = """
import sys

import numpy as np

from dimos.memory.store.sqlite import SqliteStore

store = SqliteStore(path=sys.argv[1], must_exist=True)
store.start()
try:
    namespace = {"__name__": "__main__", "store": store, "np": np}
    exec(compile(sys.stdin.read(), "<run_python>", "exec"), namespace)
finally:
    store.stop()
"""


class RunPythonSkillConfig(ModuleConfig):
    max_output_chars: int = Field(default=DEFAULT_MAX_OUTPUT_CHARS, gt=0)


class RunPythonSkill(Module):
    """Give an agent a fresh Python process for each tool call."""

    dedicated_worker: ClassVar[bool] = True

    config: RunPythonSkillConfig
    _recorder: Recorder

    @skill
    def run_python(
        self,
        code: str,
        timeout: Annotated[float, Field(gt=0.0, le=MAX_TIMEOUT_SECONDS)] = MAX_TIMEOUT_SECONDS,
    ) -> str:
        """Run Python for calculations and analysis, with live robot memory available.

        Use this for computation or to inspect any data recorded by the robot,
        including images, poses, transforms, point clouds, and other streams. The
        namespace starts with ``store`` (the live memory store) and ``np`` (NumPy),
        and normal Python imports are available.

        Each call runs in a new Python process. Variables and imports do not carry
        over, so include the complete operation in every call. Print the result you
        want returned. To discover available data, start with::

            print(store.list_streams())
            print(store.summary())

        Access a stream with ``store.streams.<name>``. Stream queries are lazy until
        a terminal operation such as ``first()``, ``last()``, ``count()``, or
        ``to_list()``. Observations expose ``data``, ``ts``, ``pose``, and ``tags``.
        Import any additional message types or transforms needed for the task.

        Output is truncated, so print summaries rather than large arrays. Tracebacks
        are returned with any output that preceded the error.

        Args:
            code: Complete Python source for this call. Print what you want returned.
            timeout: Maximum execution time in seconds, up to 100 seconds.
        """
        if not 0.0 < timeout <= MAX_TIMEOUT_SECONDS:
            raise ValueError(f"timeout must be greater than 0 and at most {MAX_TIMEOUT_SECONDS:g}")

        try:
            result = subprocess.run(
                [sys.executable, "-c", _PYTHON_BOOTSTRAP, self._recorder.recording_path()],
                input=code,
                capture_output=True,
                text=True,
                check=False,
                timeout=timeout,
            )
        except subprocess.TimeoutExpired:
            return f"(execution timed out after {timeout:g} seconds)"
        output = result.stdout
        if result.stderr:
            if output and not output.endswith("\n"):
                output += "\n"
            output += result.stderr

        if not output.strip():
            return "(no output - did you forget to print()?)"
        return self._cap(output)

    def _cap(self, text: str) -> str:
        limit = self.config.max_output_chars
        if len(text) <= limit:
            return text
        return text[:limit] + f"\n... [truncated, {len(text)} chars total]"
