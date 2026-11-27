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

from concurrent.futures import ThreadPoolExecutor
import threading

import pytest
from reactivex import Subject, empty

from dimos.memory.replay_progress import ReplayProgress
from dimos.memory.store.memory import MemoryStore
from dimos.memory.testing import PLAYBACK_TIMEOUT


@pytest.mark.parametrize("seek,duration,expected", [(None, None, 20.0), (5.0, 5.0, 0.0)])
def test_start_logs_selected_window_and_playback_settings(mocker, seek, duration, expected):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    with MemoryStore() as store:
        for ts in (10.0, 20.0, 30.0):
            store.stream("odom", int).append(1, ts=ts)
        store.stream("unused", int).append(2, ts=50.0)
        replay = store.replay(speed=2.0, loop=True, seek=seek, duration=duration)
        progress = ReplayProgress("recording")
        progress.start(replay, [replay.stream("odom")], path="/data/recording.db")
    log.info.assert_called_once_with(
        "Replay started",
        source="recording",
        path="/data/recording.db",
        duration_s=expected,
        speed=2.0,
        loop=True,
    )


def test_completion_waits_for_all_subscriptions_and_seal(mocker):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    finished = mocker.Mock()
    progress = ReplayProgress("recording", on_complete=finished)
    with progress.track(empty()).subscribe():
        log.info.assert_not_called()
        with Subject() as source, progress.track(source).subscribe():
            progress.seal()
            log.info.assert_not_called()
            source.on_completed()
            progress.seal()
    log.info.assert_called_once_with("Replay finished", source="recording")
    finished.assert_called_once_with()


def test_concurrent_stream_completion_reports_once(mocker):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    progress = ReplayProgress("recording")
    with Subject() as first, Subject() as second:
        with progress.track(first).subscribe(), progress.track(second).subscribe():
            progress.seal()
            with ThreadPoolExecutor(max_workers=2) as workers:
                first_done = workers.submit(first.on_completed)
                second_done = workers.submit(second.on_completed)
                first_done.result(timeout=5)
                second_done.result(timeout=5)
    log.info.assert_called_once_with("Replay finished", source="recording")


def test_cancelled_stream_does_not_report_success(mocker):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    progress = ReplayProgress("recording")
    with Subject() as source:
        with progress.track(source).subscribe():
            progress.seal()
        source.on_completed()
    log.info.assert_not_called()


def test_failed_stream_propagates_error_without_reporting_success(mocker):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    progress = ReplayProgress("recording")
    error = ValueError("bad frame")
    errors = []
    with Subject() as source, progress.track(source).subscribe(on_error=errors.append):
        progress.seal()
        source.on_error(error)
    assert errors == [error]
    log.info.assert_not_called()


def test_empty_recording_logs_zero_duration_and_completion(mocker):
    log = mocker.patch("dimos.memory.replay_progress.logger")
    progress = ReplayProgress("empty.db")
    with MemoryStore() as store:
        store.stream("odom", int)
        replay = store.replay()
        stream = replay.stream("odom")
        progress.start(replay, [stream])
        playback = replay.playback()
        finished = threading.Event()
        with progress.track(playback.add(stream)).subscribe(on_completed=finished.set):
            progress.seal()
            playback.play()
            assert finished.wait(PLAYBACK_TIMEOUT)
        playback.stop()
    assert log.info.call_args_list == [
        mocker.call(
            "Replay started",
            source="empty.db",
            path="empty.db",
            duration_s=0.0,
            speed=1.0,
            loop=False,
        ),
        mocker.call("Replay finished", source="empty.db"),
    ]
