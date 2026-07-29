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

from langchain_core.messages import AIMessage, HumanMessage, ToolMessage
from langchain_openai import ChatOpenAI
from langgraph.graph import END, START, MessagesState, StateGraph
from langgraph.prebuilt import ToolNode
import pytest

from dimos.agents.testing.mock_model import MockModel
from dimos.simulation.dimsim.agent_turn_control import CANCEL_ACTIVE_TURN
from dimos.simulation.dimsim.mcp_client import DimSimMcpClient


def _mcp_tool(name: str) -> dict:
    return {
        "name": name,
        "description": f"{name} description",
        "inputSchema": {"type": "object", "properties": {}},
    }


@pytest.fixture
def client() -> DimSimMcpClient:
    mcp_client = DimSimMcpClient()
    try:
        yield mcp_client
    finally:
        mcp_client.stop()


def test_observe_delivers_image_in_same_graph_turn(mocker, client) -> None:
    image = {
        "type": "image_url",
        "image_url": {"url": "data:image/jpeg;base64,AA=="},
    }
    mocker.patch.object(
        client,
        "_mcp_tool_call",
        return_value={"content": [image]},
    )
    node = ToolNode([client._mcp_tool_to_langchain(_mcp_tool("observe"))])
    graph = StateGraph(MessagesState)
    graph.add_node("tools", node)
    graph.add_edge(START, "tools")
    graph.add_edge("tools", END)
    request = AIMessage(
        content="",
        tool_calls=[
            {
                "name": "observe",
                "args": {},
                "id": "observe-call",
                "type": "tool_call",
            }
        ],
    )

    result = graph.compile().invoke({"messages": [request]})

    tool_result, image_result = result["messages"][-2:]
    assert isinstance(tool_result, ToolMessage)
    assert tool_result.content == "Current camera frame attached."
    assert tool_result.name == "observe"
    assert tool_result.tool_call_id == "observe-call"
    assert isinstance(image_result, HumanMessage)
    assert image_result.content == [
        {
            "type": "text",
            "text": "This is the current camera frame returned by observe.",
        },
        image,
    ]
    assert client._message_queue.empty()


def test_non_image_tools_keep_upstream_text_result(mocker, client) -> None:
    mocker.patch.object(
        client,
        "_mcp_tool_call",
        return_value={"content": [{"type": "text", "text": "done"}]},
    )
    tool = client._mcp_tool_to_langchain(_mcp_tool("wait"))

    result = tool.invoke({})

    assert result == "done"


def test_background_update_is_injected_at_active_tool_boundary(mocker, client) -> None:
    def call_tool(_name, _args):
        client._on_tool_stream_message(
            {
                "method": "notifications/progress",
                "params": {
                    "message": 'Found a match for ["bathtub"].',
                    "_meta": {"tool_name": "look_out_for"},
                },
            },
        )
        return {"content": [{"type": "text", "text": "wait complete"}]}

    mocker.patch.object(client, "_mcp_tool_call", side_effect=call_tool)
    with client._live_update_lock:
        client._processing_turn = True

    result = client._mcp_tool_to_langchain(_mcp_tool("wait")).invoke({})

    assert result == ('wait complete\n\n[tool:look_out_for] Found a match for ["bathtub"].')
    assert client._message_queue.empty()
    assert client._take_live_update_text() == ""


def test_lookout_continuation_result_is_injected_into_active_turn(mocker, client) -> None:
    client._tool_registry = {"stop_navigation": _mcp_tool("stop_navigation")}
    mocker.patch.object(
        client,
        "_mcp_tool_call",
        side_effect=[
            {"content": [{"type": "text", "text": "Stopped"}]},
            {"content": [{"type": "text", "text": "wait complete"}]},
        ],
    )
    with client._live_update_lock:
        client._processing_turn = True

    client.dispatch_continuation(
        {"tool": "stop_navigation", "args": {}},
        {"label": "bathtub"},
    )
    result = client._mcp_tool_to_langchain(_mcp_tool("wait")).invoke({})

    assert result == (
        "wait complete\n\n"
        "Automatically executed 'stop_navigation' as a continuation of lookout "
        "detection (detected: bathtub). Result: Stopped"
    )
    assert client._message_queue.empty()


def test_background_update_queues_normally_while_agent_is_idle(client) -> None:
    client._on_tool_stream_message(
        {
            "method": "notifications/message",
            "params": {
                "data": "exploration finished",
                "logger": "begin_exploration",
            },
        },
    )

    message = client._message_queue.get_nowait()
    assert message == HumanMessage(
        content="[tool:begin_exploration] exploration finished",
    )


def test_turn_cancellation_skips_the_next_tool_and_exits_create_agent(mocker, client) -> None:
    call_tool = mocker.patch.object(client, "_mcp_tool_call")
    tool = client._mcp_tool_to_langchain(_mcp_tool("relative_move"))
    model = MockModel(
        responses=[
            AIMessage(
                content="",
                tool_calls=[
                    {
                        "name": "relative_move",
                        "args": {},
                        "id": "move-call",
                        "type": "tool_call",
                    }
                ],
            ),
            AIMessage(content="model should not run again"),
        ]
    )
    graph = client._create_eval_agent(model, [tool])
    with client._live_update_lock:
        client._processing_turn = True
    client._on_eval_turn_control(
        {
            "type": CANCEL_ACTIVE_TURN,
            "runId": "run-current",
        }
    )

    updates = list(graph.stream({"messages": []}, stream_mode="updates"))

    assert model.i == 1
    assert call_tool.call_count == 0
    assert tool.return_direct is True
    tool_messages = [
        message
        for update in updates
        for node in update.values()
        for message in node.get("messages", [])
        if isinstance(message, ToolMessage)
    ]
    assert [message.content for message in tool_messages] == [
        "The DimSim evaluation has ended. End this agent turn now."
    ]


def test_turn_cancellation_after_active_tool_ends_at_that_boundary(mocker, client) -> None:
    def call_tool(_name, _args):
        client._on_eval_turn_control(
            {
                "type": CANCEL_ACTIVE_TURN,
                "runId": "run-current",
            }
        )
        return {"content": [{"type": "text", "text": "movement stopped"}]}

    call = mocker.patch.object(client, "_mcp_tool_call", side_effect=call_tool)
    tool = client._mcp_tool_to_langchain(_mcp_tool("relative_move"))
    with client._live_update_lock:
        client._processing_turn = True

    result = tool.invoke({})

    call.assert_called_once_with("relative_move", {})
    assert result == (
        "movement stopped\n\nThe DimSim evaluation has ended. End this agent turn now."
    )
    assert tool.return_direct is True


def test_turn_cancellation_is_ignored_while_idle(client) -> None:
    client._on_eval_turn_control(
        {
            "type": CANCEL_ACTIVE_TURN,
            "runId": "stale-run",
        }
    )

    assert not client._eval_turn_cancel.is_set()


def test_agent_enables_streaming_for_responses_model(mocker, client) -> None:
    model = ChatOpenAI(
        api_key="test-key",
        model="gpt-5.6-luna",
        use_responses_api=True,
    )
    graph = mocker.Mock()
    mocker.patch.object(client, "_fetch_tools", return_value=[])
    init_model = mocker.patch(
        "dimos.simulation.dimsim.mcp_client._init_model",
        return_value=model,
    )
    create_agent = mocker.patch(
        "dimos.simulation.dimsim.mcp_client.create_agent",
        return_value=graph,
    )
    mocker.patch.object(client._thread, "start")

    client.on_system_modules([])

    init_model.assert_called_once_with(client.config.model)
    configured_model = create_agent.call_args.kwargs["model"]
    assert isinstance(configured_model, ChatOpenAI)
    assert configured_model is not model
    assert configured_model.use_responses_api is True
    assert configured_model.streaming is True
    create_agent.assert_called_once_with(
        model=configured_model,
        tools=[],
        system_prompt=client.config.system_prompt,
    )
    assert client._state_graph is graph


def test_human_task_clears_spatial_memory_before_queueing(mocker, client) -> None:
    spatial_memory = mocker.patch.object(
        client,
        "_spatial_memory",
        create=True,
    )

    client._history.append(HumanMessage(content="previous task"))
    client._message_queue.put(HumanMessage(content="stale tool update"))

    client._queue_human_input("find the bathtub")

    spatial_memory.clear_eval_memory.assert_called_once_with()
    assert client._history == []
    message = client._message_queue.get_nowait()
    assert message == HumanMessage(content="find the bathtub")
    assert client._message_queue.empty()


def test_agent_turn_publishes_busy_then_idle_for_eval_barrier(mocker, client) -> None:
    graph = mocker.Mock()
    graph.stream.return_value = []
    publish_idle = mocker.patch.object(client.agent_idle, "publish")
    mocker.patch.object(client.agent, "publish")
    tool = client._mcp_tool_to_langchain(_mcp_tool("relative_move"))
    tool.return_direct = True
    client._eval_turn_cancel.set()

    client._process_message(graph, HumanMessage(content="go to the couch"))

    idle_states = [call.args[0] for call in publish_idle.call_args_list]
    assert idle_states[0] is False
    assert idle_states[-1] is True
    assert tool.return_direct is False
    assert not client._eval_turn_cancel.is_set()


def test_eval_idle_heartbeat_republishes_busy_state(mocker, client) -> None:
    publish_idle = mocker.patch.object(client.agent_idle, "publish")
    mocker.patch.object(
        client._eval_idle_stop,
        "wait",
        side_effect=[False, True],
    )
    client._set_eval_idle(False)
    publish_idle.reset_mock()

    client._eval_idle_heartbeat()

    publish_idle.assert_called_once_with(False)
