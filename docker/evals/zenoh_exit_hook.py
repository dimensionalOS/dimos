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
"""Close zenoh sessions a script forgot to close, before Python joins its threads at exit.
eclipse-zenoh 1.10 otherwise keeps the interpreter alive; a no-dimOS eval agent's probe script
then runs until the tool-call cap. Loaded through zenoh_exit_hook.pth."""

import threading

try:
    import zenoh
except ImportError:
    zenoh = None

if zenoh is not None:
    _open = zenoh.open
    _sessions: list[object] = []

    def _tracked_open(*args: object, **kwargs: object) -> object:
        session = _open(*args, **kwargs)
        _sessions.append(session)
        return session

    def _close_all() -> None:
        for session in _sessions:
            try:
                session.close()
            except Exception:
                pass

    zenoh.open = _tracked_open
    threading._register_atexit(_close_all)
