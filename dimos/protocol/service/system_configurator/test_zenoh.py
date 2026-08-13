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

from dimos.protocol.service.system_configurator import zenoh as zenoh_mod
from dimos.protocol.service.system_configurator.zenoh import MemlockConfiguratorLinux
from dimos.protocol.service.system_configurator.zenoh_config import zenoh_configurators

REQUIRED = 64 * 1024 * 1024
USER = "testuser"


def _configurator(
    monkeypatch, soft: int, hard: int | None = None, persisted: str | None = None
) -> MemlockConfiguratorLinux:
    """A configurator over a fake rlimit pair and an optional pam_limits file."""
    state = {"limit": (soft, soft if hard is None else hard)}
    monkeypatch.setattr(resource, "getrlimit", lambda _: state["limit"])
    monkeypatch.setattr(resource, "setrlimit", lambda _, value: state.__setitem__("limit", value))
    monkeypatch.setattr(zenoh_mod, "LIMITS_FILE", _StubPath(persisted))
    monkeypatch.setattr(MemlockConfiguratorLinux, "_user", property(lambda self: USER))
    return MemlockConfiguratorLinux(required_bytes=REQUIRED)


class _StubPath:
    """Stands in for LIMITS_FILE without touching /etc."""

    def __init__(self, text: str | None) -> None:
        self._text = text

    def read_text(self) -> str:
        if self._text is None:
            raise OSError("no such file")
        return self._text

    def __str__(self) -> str:
        return "/etc/security/limits.d/99-dimos-memlock.conf"


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


def test_user_scoped_drop_in_stops_the_prompt(monkeypatch):
    """pam_limits only applies at login; re-asking every run would never help."""
    configurator = _configurator(
        monkeypatch,
        soft=8 * 1024 * 1024,
        persisted=f"{USER}\t-\tmemlock\t65536\n",
    )
    assert configurator.check() is True
    assert configurator.explanation() is None


def test_wildcard_drop_in_still_counts(monkeypatch):
    """A legacy '*' line applies to this user too, so it must not re-prompt."""
    configurator = _configurator(
        monkeypatch,
        soft=8 * 1024 * 1024,
        persisted="*\t-\tmemlock\t65536\n",
    )
    assert configurator.check() is True


def test_drop_in_for_another_user_is_ignored(monkeypatch):
    """Someone else's line grants us nothing, so the prompt must still fire."""
    configurator = _configurator(
        monkeypatch,
        soft=8 * 1024 * 1024,
        persisted="somebodyelse\t-\tmemlock\t65536\n",
    )
    assert configurator.check() is False


def test_drop_in_below_requirement_still_prompts(monkeypatch):
    configurator = _configurator(
        monkeypatch,
        soft=8 * 1024 * 1024,
        persisted=f"{USER}\t-\tmemlock\t16384\n",
    )
    assert configurator.check() is False


def test_unparseable_drop_in_still_prompts(monkeypatch):
    configurator = _configurator(monkeypatch, soft=8 * 1024 * 1024, persisted="garbage\n")
    assert configurator.check() is False
