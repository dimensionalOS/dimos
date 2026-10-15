# Copyright 2025-2026 Dimensional Inc.
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
from contextlib import contextmanager
import errno
import os
import pickle
import threading
import time
from unittest.mock import MagicMock, create_autospec, patch

from lcm import LCM
import pytest

from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.service.lcmservice import (
    _DEFAULT_LCM_URL,
    LCMConfig,
    LCMService,
    autoconf,
)
from dimos.protocol.service.system_configurator.lcm import (
    BufferConfiguratorLinux,
    BufferConfiguratorMacOS,
    MaxFileConfiguratorMacOS,
    MulticastConfiguratorLinux,
    MulticastConfiguratorMacOS,
)
from dimos.protocol.service.system_configurator.libpython import LibPythonConfiguratorMacOS


@contextmanager
def _memory_lcm_service(borrowed):
    external_lcm = LCM("memq://") if borrowed else None
    service = LCMService(url="memq://", lcm=external_lcm)
    try:
        yield service, external_lcm
    finally:
        service.stop()


@pytest.fixture(params=[False, True], ids=["owned", "borrowed"])
def memory_lcm_service(request):
    with _memory_lcm_service(request.param) as service:
        yield service


@pytest.mark.parametrize("borrowed", [False, True], ids=["owned", "borrowed"])
@pytest.mark.parametrize("publish_before_start", [False, True])
def test_stop_from_callback_releases_after_dispatch_and_allows_restart(
    run_in_subprocess, borrowed, publish_before_start
):
    run_in_subprocess(_check_callback_restart, borrowed, publish_before_start)


def _check_callback_restart(borrowed, publish_before_start):
    with _memory_lcm_service(borrowed) as (service, external_lcm):
        _stop_from_callback_and_restart(service, external_lcm, publish_before_start)


def _stop_from_callback_and_restart(service, external_lcm, publish_before_start):
    native_id = id(service.l)
    received = []
    preserved_during_callback = []
    stopped = threading.Event()

    def stop_from_callback(channel, message):
        service.stop()
        received.append(message)
        preserved_during_callback.append(id(service.l) == native_id)
        service._defer_unsubscribe(subscription)
        stopped.set()

    subscription = service.l.subscribe("shutdown", stop_from_callback)
    if publish_before_start:
        service.l.publish("shutdown", b"first")
    service.start()
    thread = service._thread
    if not publish_before_start:
        service.l.publish("shutdown", b"first")

    assert stopped.wait(timeout=2.0)
    thread.join(timeout=2.0)
    assert not thread.is_alive()
    assert received == [b"first"]
    assert preserved_during_callback == [True]
    assert service.l is external_lcm

    service.start()
    delivered = threading.Event()

    def receive_after_restart(channel, message):
        received.append(message)
        delivered.set()

    service.l.subscribe("shutdown", receive_after_restart)
    service.l.publish("shutdown", b"second")
    assert delivered.wait(timeout=2.0)
    assert received == [b"first", b"second"]


@pytest.mark.parametrize("borrowed", [False, True], ids=["owned", "borrowed"])
@pytest.mark.parametrize("use_service_pool", [False, True])
def test_callback_can_stop_while_another_thread_is_joining(
    run_in_subprocess, borrowed, use_service_pool
):
    run_in_subprocess(_check_concurrent_stop, borrowed, use_service_pool)


def _check_concurrent_stop(borrowed, use_service_pool):
    with _memory_lcm_service(borrowed) as (service, external_lcm):
        _stop_concurrently(service, external_lcm, use_service_pool)


def _stop_concurrently(service, external_lcm, use_service_pool):
    entered = threading.Event()
    callback_stopped = threading.Event()

    def stop_from_callback(channel, message):
        entered.set()
        assert service._stop_event.wait(timeout=2.0)
        service.stop()
        callback_stopped.set()

    service.l.subscribe("shutdown", stop_from_callback)
    service.start()
    service.l.publish("shutdown", b"stop")
    assert entered.wait(timeout=2.0)

    with (
        service._get_call_thread_pool() if use_service_pool else ThreadPoolExecutor(max_workers=1)
    ) as executor:
        stopped = executor.submit(service.stop)
        assert callback_stopped.wait(timeout=2.0)
        stopped.result(timeout=2.0)

    assert service._thread is None
    assert service.l is external_lcm


def test_callback_stop_closes_owned_native_descriptor(run_in_subprocess):
    run_in_subprocess(_check_native_destruction)


def _check_native_destruction():
    with _memory_lcm_service(False) as (service, _):
        descriptor = service.l.fileno()
        preserved = []

        def stop_callback(channel, message):
            service.stop()
            preserved.append(os.fstat(descriptor))

        service.l.subscribe("shutdown", stop_callback)
        service.l.publish("shutdown", b"stop")
        service.start()
        service._thread.join(timeout=2.0)

        assert len(preserved) == 1
        with pytest.raises(OSError) as error:
            os.fstat(descriptor)
        assert error.value.errno == errno.EBADF


def test_stop_timeout_keeps_native_instance_until_callback_returns(memory_lcm_service, monkeypatch):
    service, external_lcm = memory_lcm_service
    monkeypatch.setattr("dimos.protocol.service.lcmservice.DEFAULT_THREAD_JOIN_TIMEOUT", 0.0)
    native_lcm = service.l
    entered = threading.Event()
    release_callback = threading.Event()

    def block_callback(channel, message):
        entered.set()
        release_callback.wait(timeout=5.0)

    native_lcm.subscribe("shutdown", block_callback)
    service.start()
    thread = service._thread
    native_lcm.publish("shutdown", b"stop")
    try:
        assert entered.wait(timeout=2.0)
        service.stop()
        assert thread.is_alive()
        assert service.l is native_lcm
        with pytest.raises(RuntimeError, match="still stopping"):
            service.start()
        with pytest.raises(RuntimeError, match="cannot accept calls"):
            service._get_call_thread_pool()
    finally:
        release_callback.set()
        thread.join(timeout=2.0)

    assert not thread.is_alive()
    assert service.l is external_lcm


def test_restart_is_rejected_until_stop_finishes(memory_lcm_service, mocker):
    service, _ = memory_lcm_service
    service.start()
    old_thread = service._thread
    joined = threading.Event()
    finish_stop = threading.Event()
    real_join = old_thread.join

    def pause_after_join(timeout):
        real_join(timeout)
        joined.set()
        assert finish_stop.wait(timeout=2.0)

    mocker.patch.object(old_thread, "join", side_effect=pause_after_join)
    with ThreadPoolExecutor(max_workers=1) as executor:
        stopped = executor.submit(service.stop)
        try:
            assert joined.wait(timeout=2.0)
            with pytest.raises(RuntimeError, match="still stopping"):
                service.start()
            with pytest.raises(RuntimeError, match="cannot accept calls"):
                service._get_call_thread_pool()
        finally:
            finish_stop.set()
            stopped.result(timeout=2.0)

    service.start()
    pool = service._get_call_thread_pool()
    assert pool.submit(lambda: 42).result(timeout=2.0) == 42


def test_restart_from_stopping_callback_is_rejected(run_in_subprocess):
    run_in_subprocess(_check_callback_restart_rejected)


def _check_callback_restart_rejected():
    with _memory_lcm_service(False) as (service, _):
        rejected = threading.Event()

        def stop_and_restart(channel, message):
            service.stop()
            with pytest.raises(RuntimeError, match="still stopping"):
                service.start()
            rejected.set()

        service.l.subscribe("shutdown", stop_and_restart)
        service.l.publish("shutdown", b"stop")
        service.start()
        assert rejected.wait(timeout=2.0)


# autoconf tests


class TestConfigureSystemForLcm:
    def test_creates_linux_checks_on_linux(self) -> None:
        with patch(
            "dimos.protocol.service.system_configurator.lcm_config.platform.system",
            return_value="Linux",
        ):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                autoconf()
                mock_configure.assert_called_once()
                checks = mock_configure.call_args[0][0]
                assert len(checks) == 2
                assert isinstance(checks[0], MulticastConfiguratorLinux)
                assert isinstance(checks[1], BufferConfiguratorLinux)
                assert checks[0].loopback_interface == "lo"

    def test_creates_macos_checks_on_darwin(self) -> None:
        with patch(
            "dimos.protocol.service.system_configurator.lcm_config.platform.system",
            return_value="Darwin",
        ):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                autoconf()
                mock_configure.assert_called_once()
                checks = mock_configure.call_args[0][0]
                assert len(checks) == 4
                assert isinstance(checks[0], MulticastConfiguratorMacOS)
                assert isinstance(checks[1], BufferConfiguratorMacOS)
                assert isinstance(checks[2], MaxFileConfiguratorMacOS)
                assert isinstance(checks[3], LibPythonConfiguratorMacOS)
                assert checks[0].loopback_interface == "lo0"

    def test_passes_check_only_flag(self) -> None:
        with patch(
            "dimos.protocol.service.system_configurator.lcm_config.platform.system",
            return_value="Linux",
        ):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                autoconf(check_only=True)
                mock_configure.assert_called_once()
                assert mock_configure.call_args[1]["check_only"] is True

    def test_logs_error_on_unsupported_system(self) -> None:
        with patch(
            "dimos.protocol.service.system_configurator.lcm_config.platform.system",
            return_value="Windows",
        ):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                with patch("dimos.protocol.service.lcmservice.logger") as mock_logger:
                    autoconf()
                    mock_configure.assert_not_called()
                    mock_logger.error.assert_called_once()
                    assert "Windows" in mock_logger.error.call_args[0][0]


# LCMConfig tests


class TestLCMConfig:
    def test_default_values(self) -> None:
        config = LCMConfig()
        assert config.ttl == 0
        assert config.url == _DEFAULT_LCM_URL
        assert config.lcm is None

    def test_custom_url(self) -> None:
        custom_url = "udpm://192.168.1.1:7777?ttl=1"
        config = LCMConfig(url=custom_url)
        assert config.url == custom_url


# Topic tests


class TestTopic:
    def test_str_without_lcm_type(self) -> None:
        topic = Topic(topic="my_topic")
        assert str(topic) == "my_topic"

    def test_str_with_lcm_type(self) -> None:
        mock_type = MagicMock()
        mock_type.__msgtype__ = "TestMessage"
        topic = Topic(topic="my_topic", msg_type=mock_type)
        assert str(topic) == "my_topic#TestMessage"


# LCMService tests


class TestLCMService:
    def test_init_with_defaults(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            assert service.config.url == _DEFAULT_LCM_URL
            assert service.l == mock_lcm_instance
            mock_lcm_class.assert_called_once_with(_DEFAULT_LCM_URL)

    def test_init_with_custom_url(self) -> None:
        custom_url = "udpm://192.168.1.1:7777?ttl=1"
        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)
            mock_lcm_class.return_value = mock_lcm_instance

            # Pass url as kwarg, not config=
            LCMService(url=custom_url)
            mock_lcm_class.assert_called_once_with(custom_url)

    def test_init_with_existing_lcm_instance(self) -> None:
        mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)

        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            # Pass lcm as kwarg
            service = LCMService(lcm=mock_lcm_instance)
            mock_lcm_class.assert_not_called()
            assert service.l == mock_lcm_instance

    def test_start_and_stop(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            service.start()

            # Verify thread is running
            assert service._thread is not None
            assert service._thread.is_alive()

            service.stop()

            # Give the thread a moment to stop
            time.sleep(0.1)
            assert service._thread is None

    def test_getstate_excludes_unpicklable_attrs(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            state = service.__getstate__()

            assert "l" not in state
            assert "_stop_event" not in state
            assert "_thread" not in state
            assert "_l_lock" not in state

    def test_unpickling_reinitializes_runtime_attrs(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            new_service = pickle.loads(pickle.dumps(service))

            assert new_service.l is None
            assert isinstance(new_service._stop_event, threading.Event)
            assert new_service._thread is None
            # threading.Lock is a factory function, not a type
            # Just check that the lock exists and has acquire/release methods
            assert hasattr(new_service._l_lock, "acquire")
            assert hasattr(new_service._l_lock, "release")

    def test_start_reinitializes_lcm_after_unpickling(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            new_service = pickle.loads(pickle.dumps(service))

            # Start should reinitialize LCM
            new_service.start()

            # LCM should be created again
            assert mock_lcm_class.call_count == 2

            new_service.stop()

    def test_stop_cleans_up_lcm_instance(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm_mod.LCM") as mock_lcm_class:
            mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            service.start()
            service.stop()

            # LCM instance should be cleaned up when we created it
            assert service.l is None

    def test_stop_preserves_external_lcm_instance(self) -> None:
        mock_lcm_instance = create_autospec(LCM, spec_set=True, instance=True)

        # Pass lcm as kwarg
        service = LCMService(lcm=mock_lcm_instance)
        service.start()
        service.stop()

        # External LCM instance should not be cleaned up
        assert service.l == mock_lcm_instance

    def test_get_call_thread_pool_reuses_executor(self, memory_lcm_service):
        service, _ = memory_lcm_service
        pool = service._get_call_thread_pool()
        assert pool.submit(lambda: 42).result(timeout=2.0) == 42
        assert service._get_call_thread_pool() is pool

    def test_stop_shuts_down_thread_pool(self, memory_lcm_service):
        service, _ = memory_lcm_service
        service.start()
        pool = service._get_call_thread_pool()

        service.stop()

        with pytest.raises(RuntimeError, match="cannot schedule new futures"):
            pool.submit(lambda: 42)
        with pytest.raises(RuntimeError, match="cannot accept calls"):
            service._get_call_thread_pool()
