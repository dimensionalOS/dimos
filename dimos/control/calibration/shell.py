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

"""``cal`` in ``dimos shell``: a ``CalibrationModule``'s RPCs, with ``run`` walking the run."""

from __future__ import annotations

import time
from typing import TYPE_CHECKING, Any

if TYPE_CHECKING:
    from dimos.control.calibration.module import CalibrationStatus

POLL_S = 0.3
ENDED = ("idle", "done", "failed", "aborted")
LEFT = "\nLeft the run, which goes on: cal.run() follows it again, cal.abort() stops it."


class CalibrationShell:
    """A ``CalibrationModule``'s RPCs for the terminal; ``run`` follows the run until it ends.

    ``run`` prints each new message and asks each question at a ``> `` prompt, Enter giving its
    first answer. Ctrl-C leaves it and the run goes on; ``status``, ``answer``, ``abort`` and
    ``bake`` are the RPCs.

    Args:
        proxy: The deployed module, as ``app.CalibrationModule``.
    """

    def __init__(self, proxy: Any) -> None:
        self._proxy = proxy

    @property
    def arm(self) -> str:
        return self.status().arm

    def run(self, steps: list[str] | None = None) -> None:
        """Calibrate: check, then ``steps`` (default: the ``steps`` setting), answering each
        question here until the run ends; follows a run already going."""
        seen = self.status().logged
        reply = self._proxy.run(steps)
        print(reply)
        if reply == "already running":
            status = self.status()
            seen = status.logged
            if status.question is None:
                print(str(status).splitlines()[0])
        elif reply != "started":
            return
        try:
            self._follow(seen)
        except (KeyboardInterrupt, EOFError):
            print(LEFT)

    def _follow(self, seen: int) -> None:
        while True:
            status = self.status()
            new = min(status.logged - seen, len(status.log))
            for line in status.log[len(status.log) - new :]:
                if not line.startswith(("? ", "> ")):
                    print(line)
            seen = status.logged
            if status.question is not None:
                self._ask(status)
            elif status.state in ENDED:
                return
            else:
                time.sleep(POLL_S)

    def _ask(self, status: CalibrationStatus) -> None:
        print(str(status).splitlines()[0])
        print(f"? {status.question}")
        print("  answer: " + " | ".join(status.answers))
        while True:
            word = input("> ").strip() or status.answers[0]
            now = self.status()
            if (now.question, now.logged) != (status.question, status.logged):
                print("The question changed before this answer; not sent.")
                return
            reply = self._proxy.answer(word)
            if reply == "ok":
                return
            print(reply)

    def status(self) -> CalibrationStatus:
        """What the run does, has found and asks; ``print`` it."""
        status: CalibrationStatus = self._proxy.status()
        return status

    def answer(self, text: str) -> str:
        """Answer the pending question with one of ``status().answers``: "ok", or why not."""
        reply: str = self._proxy.answer(text)
        return reply

    def abort(self) -> str:
        """Stop streaming now (the connection holds the arm), then ask whether to go home."""
        reply: str = self._proxy.abort()
        return reply

    def bake(self, urdf_path: str, out_path: str | None = None) -> str:
        """Write the URDF at ``urdf_path`` with the fitted links of every calibrated arm."""
        reply: str = self._proxy.bake(urdf_path, out_path)
        return reply
