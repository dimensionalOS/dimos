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

import threading
import time
from unittest.mock import MagicMock, patch

from dimos.protocol.pubsub.impl.lcmpubsub import Topic
from dimos.protocol.service.lcmservice import (
    _DEFAULT_LCM_URL,
    LCMConfig,
    LCMService,
    autoconf,
)
from dimos.protocol.service.system_configurator import (
    BufferConfiguratorLinux,
    BufferConfiguratorMacOS,
    MaxFileConfiguratorMacOS,
    MulticastConfiguratorLinux,
    MulticastConfiguratorMacOS,
)

# ----------------------------- autoconf tests -----------------------------


class TestConfigureSystemForLcm:
    def test_creates_linux_checks_on_linux(self) -> None:
        with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                autoconf()
                mock_configure.assert_called_once()
                checks = mock_configure.call_args[0][0]
                assert len(checks) == 2
                assert isinstance(checks[0], MulticastConfiguratorLinux)
                assert isinstance(checks[1], BufferConfiguratorLinux)
                assert checks[0].loopback_interface == "lo"

    def test_creates_macos_checks_on_darwin(self) -> None:
        with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Darwin"):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                autoconf()
                mock_configure.assert_called_once()
                checks = mock_configure.call_args[0][0]
                assert len(checks) == 3
                assert isinstance(checks[0], MulticastConfiguratorMacOS)
                assert isinstance(checks[1], BufferConfiguratorMacOS)
                assert isinstance(checks[2], MaxFileConfiguratorMacOS)
                assert checks[0].loopback_interface == "lo0"

    def test_passes_check_only_flag(self) -> None:
        with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Linux"):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                autoconf(check_only=True)
                mock_configure.assert_called_once()
                assert mock_configure.call_args[1]["check_only"] is True

    def test_logs_error_on_unsupported_system(self) -> None:
        with patch("dimos.protocol.service.lcmservice.platform.system", return_value="Windows"):
            with patch("dimos.protocol.service.lcmservice.configure_system") as mock_configure:
                with patch("dimos.protocol.service.lcmservice.logger") as mock_logger:
                    autoconf()
                    mock_configure.assert_not_called()
                    mock_logger.error.assert_called_once()
                    assert "Windows" in mock_logger.error.call_args[0][0]


# ----------------------------- LCMConfig tests -----------------------------


class TestLCMConfig:
    def test_default_values(self) -> None:
        config = LCMConfig()
        assert config.ttl == 0
        assert config.url == _DEFAULT_LCM_URL
        assert config.autoconf is True
        assert config.lcm is None

    def test_custom_url(self) -> None:
        custom_url = "udpm://192.168.1.1:7777?ttl=1"
        config = LCMConfig(url=custom_url)
        assert config.url == custom_url

    def test_post_init_sets_default_url_when_none(self) -> None:
        config = LCMConfig(url=None)
        assert config.url == _DEFAULT_LCM_URL

    def test_autoconf_can_be_disabled(self) -> None:
        config = LCMConfig(autoconf=False)
        assert config.autoconf is False


# ----------------------------- Topic tests -----------------------------


class TestTopic:
    def test_str_without_lcm_type(self) -> None:
        topic = Topic(topic="my_topic")
        assert str(topic) == "my_topic"

    def test_str_with_lcm_type(self) -> None:
        mock_type = MagicMock()
        mock_type.msg_name = "TestMessage"
        topic = Topic(topic="my_topic", lcm_type=mock_type)
        assert str(topic) == "my_topic#TestMessage"


# ----------------------------- LCMService tests -----------------------------


class TestLCMService:
    def test_init_with_default_config(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            assert service.config.url == _DEFAULT_LCM_URL
            assert service.l == mock_lcm_instance
            mock_lcm_class.assert_called_once_with(_DEFAULT_LCM_URL)

    def test_init_with_custom_url(self) -> None:
        custom_url = "udpm://192.168.1.1:7777?ttl=1"
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            # Pass url as kwarg, not config=
            LCMService(url=custom_url)
            mock_lcm_class.assert_called_once_with(custom_url)

    def test_init_with_existing_lcm_instance(self) -> None:
        mock_lcm_instance = MagicMock()

        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            # Pass lcm as kwarg
            service = LCMService(lcm=mock_lcm_instance)
            mock_lcm_class.assert_not_called()
            assert service.l == mock_lcm_instance

    def test_start_and_stop(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            with patch("dimos.protocol.service.lcmservice.autoconf"):
                service = LCMService(autoconf=False)
                service.start()

                # Verify thread is running
                assert service._thread is not None
                assert service._thread.is_alive()

                service.stop()

                # Give the thread a moment to stop
                time.sleep(0.1)
                assert not service._thread.is_alive()

    def test_start_calls_configure_system(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            with patch("dimos.protocol.service.lcmservice.autoconf") as mock_configure:
                service = LCMService(autoconf=True)
                service.start()

                # With autoconf=True, check_only should be False
                mock_configure.assert_called_once_with(check_only=False)

                service.stop()

    def test_start_with_autoconf_disabled(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            with patch("dimos.protocol.service.lcmservice.autoconf") as mock_configure:
                service = LCMService(autoconf=False)
                service.start()

                # With autoconf=False, check_only should be True
                mock_configure.assert_called_once_with(check_only=True)

                service.stop()

    def test_getstate_excludes_unpicklable_attrs(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            state = service.__getstate__()

            assert "l" not in state
            assert "_stop_event" not in state
            assert "_thread" not in state
            assert "_l_lock" not in state
            assert "_call_thread_pool" not in state
            assert "_call_thread_pool_lock" not in state

    def test_setstate_reinitializes_runtime_attrs(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            state = service.__getstate__()

            # Simulate unpickling
            new_service = object.__new__(LCMService)
            new_service.__setstate__(state)

            assert new_service.l is None
            assert isinstance(new_service._stop_event, threading.Event)
            assert new_service._thread is None
            # threading.Lock is a factory function, not a type
            # Just check that the lock exists and has acquire/release methods
            assert hasattr(new_service._l_lock, "acquire")
            assert hasattr(new_service._l_lock, "release")

    def test_start_reinitializes_lcm_after_unpickling(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            with patch("dimos.protocol.service.lcmservice.autoconf"):
                service = LCMService()
                state = service.__getstate__()

                # Simulate unpickling
                new_service = object.__new__(LCMService)
                new_service.__setstate__(state)

                # Start should reinitialize LCM
                new_service.start()

                # LCM should be created again
                assert mock_lcm_class.call_count == 2

                new_service.stop()

    def test_stop_cleans_up_lcm_instance(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            with patch("dimos.protocol.service.lcmservice.autoconf"):
                service = LCMService()
                service.start()
                service.stop()

                # LCM instance should be cleaned up when we created it
                assert service.l is None

    def test_stop_preserves_external_lcm_instance(self) -> None:
        mock_lcm_instance = MagicMock()

        with patch("dimos.protocol.service.lcmservice.autoconf"):
            # Pass lcm as kwarg
            service = LCMService(lcm=mock_lcm_instance)
            service.start()
            service.stop()

            # External LCM instance should not be cleaned up
            assert service.l == mock_lcm_instance

    def test_get_call_thread_pool_creates_pool(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            service = LCMService()
            assert service._call_thread_pool is None

            pool = service._get_call_thread_pool()
            assert pool is not None
            assert service._call_thread_pool == pool

            # Should return same pool on subsequent calls
            pool2 = service._get_call_thread_pool()
            assert pool2 is pool

            # Clean up
            pool.shutdown(wait=False)

    def test_stop_shuts_down_thread_pool(self) -> None:
        with patch("dimos.protocol.service.lcmservice.lcm.LCM") as mock_lcm_class:
            mock_lcm_instance = MagicMock()
            mock_lcm_class.return_value = mock_lcm_instance

            with patch("dimos.protocol.service.lcmservice.autoconf"):
                service = LCMService()
                service.start()

                # Create thread pool
                pool = service._get_call_thread_pool()
                assert pool is not None

                service.stop()

                # Pool should be cleaned up
                assert service._call_thread_pool is None
