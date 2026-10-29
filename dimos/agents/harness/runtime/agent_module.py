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

"""The base class of agent modules: modules an agent writes and deploys while DimOS runs."""

from __future__ import annotations

from collections.abc import Callable
import json
import threading
import time
from typing import Any, ClassVar, TypeVar

from reactivex.disposable import Disposable

from dimos.agents.harness.runtime.constants import AGENT_MODULE_PATH, METACLASS_NAMES
from dimos.agents.harness.runtime.lib.exports import replace_exports
from dimos.core.module import Module
from dimos.core.stream import In, Out
from dimos.msgs.agent_msgs.AgentNotice import AgentNotice
from dimos.utils.logging_config import setup_logger

T = TypeVar("T")


def _harness_name(cls: type) -> str:
    match = AGENT_MODULE_PATH.match(cls.__module__)
    return match["name"] if match else cls.__name__


class AgentModule(Module):
    """A module an agent saved with the harness. It runs on its own worker.

    Subscriptions made with subscribe(), handle_* handlers and anything passed to
    register_disposable() are disposed when the module stops; work it starts in other
    modules is set up before the yield of main() and torn down after it.
    """

    dedicated_worker: ClassVar[bool] = True
    notify_interval_s: ClassVar[float] = 10.0
    """notify() sends at most one notice per interval and returns False for the rest."""
    agent_notice: Out[AgentNotice]

    _notify_lock: threading.Lock
    _last_notify: float
    _suppressed: int
    _logger: Any

    def __init_subclass__(cls, **kwargs: Any) -> None:
        reserved = {
            name
            for base in AgentModule.__mro__
            for name in (*vars(base), *vars(base).get("__annotations__", {}))
        }
        taken = sorted(
            name
            for name in (*vars(cls), *vars(cls).get("__annotations__", {}))
            if name.startswith("_")
            and not name.startswith("__")
            and name in reserved
            and name not in METACLASS_NAMES
        )
        if taken:
            raise TypeError(
                f"{cls.__name__} defines {', '.join(taken)}, which AgentModule or Module "
                "uses; choose other names"
            )
        super().__init_subclass__(**kwargs)

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._logger = setup_logger()
        self._notify_lock = threading.Lock()
        self._last_notify = float("-inf")
        self._suppressed = 0

    def subscribe(self, stream: In[T], callback: Callable[[T], Any]) -> None:
        """Call callback with each message of stream until the module stops.

        An exception in callback goes to the standard DimOS log with its traceback.
        """

        def guarded(message: T) -> None:
            try:
                callback(message)
            except Exception:
                self._logger.exception(
                    "Agent module callback failed",
                    module=_harness_name(type(self)),
                    stream=stream.name,
                )

        self.register_disposable(Disposable(stream.subscribe(guarded)))

    def notify(self, text: str, **data: object) -> bool:
        """Wake the conversation that created this module. False when rate-limited.

        Every Image and PointCloud2 in data, at any depth, is saved to a file and
        replaced by its path. Everything else must be JSON with finite numbers; other data
        raises TypeError and sends nothing.
        """
        with self._notify_lock:
            now = time.monotonic()
            if now - self._last_notify < self.notify_interval_s:
                self._suppressed += 1
                return False
            name = _harness_name(type(self))
            at = time.time()
            replaced, exported = replace_exports(data, f"{name}.{int(at * 1000)}")
            notice = AgentNotice(
                name,
                str(text),
                json.dumps(replaced, allow_nan=False),
                json.dumps([item.to_json() for item in exported]),
                self._suppressed,
                at,
            )
            self._last_notify = now
            self._suppressed = 0
        self.agent_notice.publish(notice)
        return True
