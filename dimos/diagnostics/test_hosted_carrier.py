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

"""The dedicated hosted type must leave legacy and other-link messages alone."""

import pytest

from dimos.diagnostics.hosted_carrier import HostedTraceCarrier
from dimos.msgs.std_msgs.String import String
from dimos.protocol.pubsub.impl.webrtc.providers.spec import Provider, ProviderConfig


@pytest.fixture
def provider(mocker):
    provider = mocker.Mock(spec=Provider)
    mocker.patch.object(ProviderConfig, "provider", return_value=provider)
    return provider


def test_directional_carrier_preserves_text_and_ignores_other_message_types(provider):
    carrier = HostedTraceCarrier("trace-link", ProviderConfig(), "outbound", "inbound")
    received = []
    carrier.subscribe(received.append)
    carrier.publish(String(data="hello π"))
    receive = provider.subscribe.call_args.args[1]
    assert provider.subscribe.call_args.args[0] == "inbound"
    topic, wire = provider.publish.call_args.args
    assert topic == "outbound"
    receive(b"legacy robot payload", "inbound")
    other = HostedTraceCarrier("another-link", ProviderConfig(), "outbound", "inbound")
    other.publish(String(data="other"))
    receive(provider.publish.call_args.args[1], "inbound")
    receive(wire, "inbound")
    assert [value.data for value in received] == ["hello π"]


def test_failed_provider_start_can_be_retried_without_a_half_started_carrier(provider):
    provider.start.side_effect = [RuntimeError("connect failed"), None]
    carrier = HostedTraceCarrier("trace", ProviderConfig(), "out", "in")
    with pytest.raises(RuntimeError, match="connect failed"):
        carrier.start()
    carrier.publish(String(data="after retry"))
    assert provider.start.call_count == 2
    assert provider.publish.call_count == 1


def test_oversized_hosted_message_is_rejected_before_provider_send(provider):
    carrier = HostedTraceCarrier("trace", ProviderConfig(), "out", "in")
    with pytest.raises(ValueError, match="32 KiB"):
        carrier.publish(String(data="x" * 32768))
    provider.publish.assert_not_called()
