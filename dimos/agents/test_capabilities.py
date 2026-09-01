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


import threading
import time

from dimos.agents.capabilities import CapabilityRegistry


def test_acquire_then_release():
    reg = CapabilityRegistry()
    assert reg.acquire(["movement"], tool_name="start_patrol", token="t1") is None
    assert reg.snapshot() == {"movement": "start_patrol"}
    released = reg.release_by_token("t1")
    assert released == ["movement"]
    assert reg.snapshot() == {}


def test_acquire_conflict_reports_existing_tool():
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="start_patrol", token="t1")
    conflict = reg.acquire(["movement"], tool_name="follow_person", token="t2")
    assert conflict == ("movement", "start_patrol")
    # State is unchanged after a refused acquire.
    assert reg.snapshot() == {"movement": "start_patrol"}


def test_acquire_multi_cap_is_atomic_on_conflict():
    """If any cap conflicts, no caps are acquired (all-or-nothing)."""
    reg = CapabilityRegistry()
    reg.acquire(["audio"], tool_name="speak", token="t1")
    conflict = reg.acquire(["movement", "audio"], tool_name="multi", token="t2")
    assert conflict == ("audio", "speak")
    # `movement` must NOT have leaked in.
    assert "movement" not in reg.snapshot()


def test_same_tool_reacquire_takes_over():
    """Re-acquiring for the same tool succeeds and takes over the hold.

    The McpServer doesn't know whether a `tools/call` is a fresh request or the
    LLM re-issuing the same call while a previous background invocation is still
    active. The new invocation's token replaces the old one's, so the old
    invocation's later release can't free the live hold.
    """
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="start_patrol", token="t1")
    assert reg.acquire(["movement"], tool_name="start_patrol", token="t2") is None
    assert reg.snapshot() == {"movement": "start_patrol"}
    # The superseded token no longer owns anything.
    assert reg.release_by_token("t1") == []
    assert reg.snapshot() == {"movement": "start_patrol"}
    assert reg.release_by_token("t2") == ["movement"]


def test_takeover_release_does_not_free_new_holder():
    """Regression: a re-entrant same-tool invocation's stale stop must not
    release the capability held by the invocation that took over.

    Mirrors the re-entrant `follow_person` case: follow #1 (token T1) is active,
    follow #2 (token T2) takes over, then follow #1's loop tears down and emits
    its stop frame (T1). That release must be a no-op so `movement` stays held.
    """
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="follow_person", token="T1")
    reg.acquire(["movement"], tool_name="follow_person", token="T2")
    # Old loop's stop frame arrives carrying the superseded token T1.
    assert reg.release_by_token("T1") == []
    assert reg.snapshot() == {"movement": "follow_person"}
    # New loop's own stop frees it.
    assert reg.release_by_token("T2") == ["movement"]
    assert reg.snapshot() == {}


def test_release_by_token_only_releases_matching():
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="patrol", token="ta")
    reg.acquire(["audio"], tool_name="speak", token="tb")
    released = reg.release_by_token("ta")
    assert released == ["movement"]
    assert reg.snapshot() == {"audio": "speak"}


def test_release_by_unknown_token_is_noop():
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="patrol", token="ta")
    assert reg.release_by_token("nobody") == []
    assert reg.snapshot() == {"movement": "patrol"}


def test_acquire_default_is_nonblocking():
    """Without a timeout, acquire is the original fail-fast try-lock."""
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="patrol", token="t1")
    start = time.monotonic()
    conflict = reg.acquire(["movement"], tool_name="follow", token="t2")
    assert conflict == ("movement", "patrol")
    assert time.monotonic() - start < 0.5  # returned immediately, did not wait


def test_acquire_blocks_until_holder_releases():
    """A blocking acquire waits for the holder to release, then succeeds."""
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="weigh", token="t1")

    def _release_soon() -> None:
        time.sleep(0.1)
        reg.release_by_token("t1")

    start = time.monotonic()
    releaser = threading.Thread(target=_release_soon)
    releaser.start()
    conflict = reg.acquire(["movement"], tool_name="secure", token="t2", timeout=2.0)
    elapsed = time.monotonic() - start
    releaser.join()

    assert conflict is None  # acquired once the holder released
    assert elapsed >= 0.05  # actually waited rather than failing fast
    assert reg.snapshot() == {"movement": "secure"}


def test_acquire_times_out_returns_conflict():
    """If the holder never releases, a blocking acquire times out and reports it."""
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="weigh", token="t1")
    start = time.monotonic()
    conflict = reg.acquire(["movement"], tool_name="secure", token="t2", timeout=0.1)
    elapsed = time.monotonic() - start
    assert conflict == ("movement", "weigh")
    assert elapsed >= 0.05  # waited out the timeout
    # The holder still holds it; nothing leaked.
    assert reg.snapshot() == {"movement": "weigh"}


def test_acquire_can_wait_false_returns_immediately():
    """`can_wait` returning False refuses without waiting out the timeout."""
    reg = CapabilityRegistry()
    reg.acquire(["movement"], tool_name="patrol", token="t1")
    start = time.monotonic()
    conflict = reg.acquire(
        ["movement"],
        tool_name="follow",
        token="t2",
        timeout=5.0,
        can_wait=lambda _holder: False,
    )
    assert conflict == ("movement", "patrol")
    assert time.monotonic() - start < 0.5  # did not block for the 5s timeout
