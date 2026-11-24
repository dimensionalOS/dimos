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

import fcntl
from pathlib import Path
import threading

from dimos.cli.commands.agent import _build


def test_build_waits_out_another_build_before_using_the_tree(tmp_path: Path, mocker) -> None:
    package = tmp_path
    (package / "dist").mkdir()
    lock_path = package / "dist" / ".build.lock"
    lock_path.write_text("")
    mocker.patch("dimos.cli.commands.agent._installed", return_value=True)
    mocker.patch("dimos.cli.commands.agent._built", return_value=True)

    holding = threading.Event()
    release = threading.Event()

    def hold_build_lock() -> None:
        with lock_path.open("w") as lock:
            fcntl.flock(lock, fcntl.LOCK_EX)
            holding.set()
            assert release.wait(timeout=5)

    holder = threading.Thread(target=hold_build_lock)
    holder.start()
    assert holding.wait(timeout=5)

    waiter = threading.Thread(target=_build, args=(package, "node"))
    waiter.start()
    try:
        waiter.join(timeout=0.3)
        assert waiter.is_alive()
    finally:
        release.set()
        waiter.join(timeout=5)
        holder.join(timeout=5)
    assert not waiter.is_alive()
    assert not holder.is_alive()
