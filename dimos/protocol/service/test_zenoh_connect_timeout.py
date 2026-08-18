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

"""The connect timeout bounds our own wait and zenoh's dial retries."""

from pydantic import ValidationError
import pytest
import zenoh

from dimos.core.global_config import GlobalConfig
from dimos.protocol.service import zenohservice
from dimos.protocol.service.zenohservice import ZenohConfig, ZenohSessionPool


def _opened_with(monkeypatch, config: ZenohConfig) -> zenoh.Config:
    """The zenoh config the pool would open this session with."""
    captured = {}

    def fake_open(zconfig):
        captured["config"] = zconfig
        return object()

    monkeypatch.setattr(zenohservice.zenoh, "open", fake_open)
    ZenohSessionPool().acquire(config)
    return captured["config"]


def test_a_timeout_is_passed_in_milliseconds(zenoh_defaults, monkeypatch):
    opened = _opened_with(monkeypatch, ZenohConfig(connect_timeout=2.5))
    assert opened.get_json("connect/timeout_ms") == "2500"


def test_zero_leaves_zenohs_own_retry_policy(zenoh_defaults, monkeypatch):
    """Zenoh reads a zero timeout as dial once and never retry."""
    opened = _opened_with(monkeypatch, ZenohConfig(connect_timeout=0.0))
    assert opened.get_json("connect/timeout_ms") == zenoh.Config().get_json("connect/timeout_ms")


def test_a_negative_timeout_is_rejected(zenoh_defaults):
    with pytest.raises(ValidationError, match="greater than or equal to 0"):
        ZenohConfig(connect_timeout=-1.0)


def test_an_infinite_timeout_is_rejected():
    with pytest.raises(ValidationError, match="less than or equal to 86400"):
        GlobalConfig(zenoh_connect_timeout=float("inf"))


def test_the_timeout_is_part_of_the_session_identity(zenoh_defaults):
    """It reaches zenoh, so two services that differ in it need two sessions."""
    quick = ZenohConfig(connect=["tcp/192.0.2.10:7447"], connect_timeout=1.0)
    patient = ZenohConfig(connect=["tcp/192.0.2.10:7447"], connect_timeout=30.0)
    assert quick.session_key != patient.session_key


def test_a_pooled_session_is_still_shared_by_matching_configs(zenoh_defaults, monkeypatch):
    opens = []
    monkeypatch.setattr(
        zenohservice.zenoh, "open", lambda zconfig: opens.append(zconfig) or object()
    )
    pool = ZenohSessionPool()
    assert pool.acquire(ZenohConfig(connect_timeout=1.0)) is pool.acquire(
        ZenohConfig(connect_timeout=1.0)
    )
    assert len(opens) == 1
