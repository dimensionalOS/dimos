#!/usr/bin/env python3
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

"""The memlock configurator only fires when the limit is actually too low."""

import platform
import resource

from dimos.protocol.service.system_configurator.zenoh import MemlockConfiguratorLinux
from dimos.protocol.service.system_configurator.zenoh_config import zenoh_configurators

REQUIRED = 64 * 1024 * 1024


def _configurator(monkeypatch, soft: int, hard: int | None = None) -> MemlockConfiguratorLinux:
    """A configurator over a fake rlimit pair."""
    state = {"limit": (soft, soft if hard is None else hard)}
    monkeypatch.setattr(resource, "getrlimit", lambda _: state["limit"])
    monkeypatch.setattr(resource, "setrlimit", lambda _, value: state.__setitem__("limit", value))
    return MemlockConfiguratorLinux(required_bytes=REQUIRED)


def test_check_passes_when_limit_is_sufficient(monkeypatch):
    assert _configurator(monkeypatch, 64 * 1024 * 1024).check() is True


def test_check_fails_when_limit_is_too_low(monkeypatch):
    """systemd's 8MB default can't hold zenoh's 16MB pool."""
    assert _configurator(monkeypatch, 8 * 1024 * 1024).check() is False


def test_unlimited_passes(monkeypatch):
    assert _configurator(monkeypatch, resource.RLIM_INFINITY).check() is True


def test_explanation_is_silent_when_configured(monkeypatch):
    assert _configurator(monkeypatch, 64 * 1024 * 1024).explanation() is None


def test_explanation_names_the_command_and_sizes(monkeypatch):
    text = _configurator(monkeypatch, 8 * 1024 * 1024).explanation()
    assert text is not None
    assert "prlimit" in text
    assert "8MB" in text and "64MB" in text


def test_fix_is_not_critical(monkeypatch):
    """Zenoh degrades to non-SHM, so a declined fix must not abort startup."""
    assert _configurator(monkeypatch, 8 * 1024 * 1024).critical is False


def test_configurators_are_linux_only():
    expected = 1 if platform.system() == "Linux" else 0
    assert len(zenoh_configurators()) == expected


def test_soft_limit_is_raised_when_hard_limit_allows(monkeypatch):
    """Raising soft up to hard needs no privileges, so it must not prompt."""
    configurator = _configurator(monkeypatch, soft=8 * 1024 * 1024, hard=REQUIRED)
    assert configurator.check() is True
    assert resource.getrlimit(resource.RLIMIT_MEMLOCK)[0] >= REQUIRED
