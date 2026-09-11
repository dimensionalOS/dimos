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

from unittest.mock import Mock

from dimos.agents.annotation import skill
from dimos.agents.mcp import tool_stream
from dimos.core.global_config import global_config


@skill
def open_stream() -> tool_stream.ToolStream:
    """Start a test skill notification stream."""
    return tool_stream.ToolStream("explore")


def test_background_messages_and_stop_events_stay_on_the_own_runtime_topic(monkeypatch):
    transport = Mock()
    factory = Mock(return_value=transport)
    monkeypatch.setattr(tool_stream, "make_transport", factory)
    monkeypatch.setattr(global_config, "tool_stream_topic", "/duck2/session-a/tool_streams")
    stream = open_stream()
    try:
        monkeypatch.setattr(global_config, "tool_stream_topic", "/duck3/session-b/tool_streams")
        stream.send("Found a frontier")
        factory.assert_called_once_with("/duck2/session-a/tool_streams")
    finally:
        stream.stop()
    assert transport.publish.call_args.args[0]["method"] == tool_stream.TOOL_STREAM_STOPPED_METHOD
    transport.stop.assert_called_once()


def test_agent_subscribes_only_to_its_configured_tool_updates(monkeypatch):
    transport = Mock()
    factory = Mock(return_value=transport)
    monkeypatch.setattr(tool_stream, "make_transport", factory)
    monkeypatch.setattr(global_config, "tool_stream_topic", "/duck3/session-b/tool_streams")
    close = tool_stream.subscribe(Mock())
    try:
        factory.assert_called_once_with("/duck3/session-b/tool_streams")
    finally:
        close()
    transport.stop.assert_called_once()
