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

from dimos_lcm.actionlib_msgs import GoalID, GoalStatus
from dimos_lcm.foxglove_msgs import Grid
from dimos_lcm.std_msgs import Float32MultiArray, Time
import pytest
from pytest_mock import MockerFixture
import typer

from dimos.cli.topic import (
    _build_eval_context,
    _decode_typed_lcm_message,
    _format_message,
    _split_topic,
    topic_echo,
    topic_send,
)
from dimos.core.global_config import global_config
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.Transform import Transform
from dimos.msgs.geometry_msgs.Twist import Twist
from dimos.msgs.geometry_msgs.Vector3 import Vector3
from dimos.msgs.sensor_msgs.CameraInfo import CameraInfo
from dimos.msgs.std_msgs.String import String
from dimos.msgs.tf2_msgs.TFMessage import TFMessage
from dimos.utils.testing.collector import CallbackCollector


def test_decode_typed_lcm_message_resolves_message_submodule() -> None:
    msg = CameraInfo(
        width=1920,
        height=1080,
        distortion_model="plumb_bob",
        frame_id="camera_optical",
    )

    decoded = _decode_typed_lcm_message(
        "/camera_info#sensor_msgs.CameraInfo",
        msg.lcm_encode(),
    )

    assert isinstance(decoded, CameraInfo)
    assert decoded.width == 1920
    assert decoded.height == 1080
    assert decoded.frame_id == "camera_optical"
    assert decoded.distortion_model == "plumb_bob"


def test_format_message_lists_fields_of_messages_without_str() -> None:
    status = GoalStatus(goal_id=GoalID(stamp=Time(sec=1, nsec=2), id="goal"), status=3, text="done")

    assert _format_message(String("hello")) == "String(data='hello')"
    assert _format_message(status) == (
        "GoalStatus(goal_id=GoalID(stamp=Time(sec=1, nsec=2), id='goal'), status=3, text='done')"
    )


def test_format_message_shortens_long_lists_and_bytes() -> None:
    array = Float32MultiArray(data_length=10, data=[float(i) for i in range(10)])
    grid = Grid(data_length=4096, data=bytes(4096))

    assert _format_message(array).endswith(
        "data=[0.0, 1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, ... (2 more)])"
    )
    assert _format_message(grid).endswith("data=<4096 bytes>)")


def test_build_eval_context_maps_message_names_to_classes() -> None:
    context = _build_eval_context()

    for name in ("Twist", "Vector3", "PoseStamped"):
        cls = context[name]
        assert isinstance(cls, type)
        assert cls.__name__ == name


@pytest.fixture(
    params=[
        ("lcm", "", "test_topic"),
        ("lcm", "", "/test_topic"),
        ("lcm", "", "/test_topic#geometry_msgs.Twist"),
        ("zenoh", "", "test_topic"),
        ("zenoh", "", "/test_topic"),
        ("zenoh", "", "/test_topic#geometry_msgs.Twist"),
        ("zenoh", "", "dimos/test_topic"),
        ("zenoh", "", "dimos/test_topic/geometry_msgs.Twist"),
        ("zenoh", "review", "dimos-domains/review/test_topic/geometry_msgs.Twist"),
    ],
    ids=[
        "lcm-bare",
        "lcm-name",
        "lcm-full",
        "zenoh-bare",
        "zenoh-name",
        "zenoh-hash",
        "zenoh-prefix",
        "zenoh-full",
        "zenoh-domain",
    ],
)
def topic_form(request: pytest.FixtureRequest, monkeypatch: pytest.MonkeyPatch) -> str:
    backend, domain, topic = request.param
    monkeypatch.setattr(global_config, "transport", backend)
    monkeypatch.setattr(global_config, "zenoh_domain", domain)
    return topic


def test_topic_send_delivers(topic_form: str) -> None:
    collector = CallbackCollector(1)
    with make_transport("/test_topic", Twist) as receiver:
        receiver.subscribe(lambda msg: collector(msg, None))
        topic_send(topic_form, "Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))")
        collector.wait()

    assert collector.results[0][0] == Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))


@pytest.mark.parametrize("type_name", [None, "Twist"])
def test_topic_echo_delivers(topic_form: str, type_name: str | None, mocker: MockerFixture) -> None:
    collector = CallbackCollector(1)
    message = Twist(Vector3(0.5, 0, 0), Vector3(0, 0, 0))
    # numpy print options are per thread, so render the expected text on the printing thread
    mocker.patch(
        "dimos.cli.topic.print",
        side_effect=lambda text, **kwargs: collector(text, _format_message(message)),
    )

    with make_transport("/test_topic", Twist) as publisher:

        def publish_and_wait(_: str) -> None:
            publisher.broadcast(None, message)
            collector.wait()

        mocker.patch("dimos.cli.topic._listen_forever", side_effect=publish_and_wait)
        topic_echo(topic_form, type_name)

    printed, expected = collector.results[0]
    assert printed == expected


@pytest.mark.parametrize(
    ("transport", "topic", "expected"),
    [
        ("zenoh", "cmd_vel", ("cmd_vel", None)),
        ("zenoh", "/cmd_vel", ("/cmd_vel", None)),
        ("zenoh", "/cmd_vel#geometry_msgs.Twist", ("/cmd_vel", Twist)),
        ("zenoh", "dimos/cmd_vel", ("cmd_vel", None)),
        ("zenoh", "dimos/cmd_vel/geometry_msgs.Twist", ("cmd_vel", Twist)),
        ("zenoh", "dimos/robot0/cmd_vel/geometry_msgs.Twist", ("robot0/cmd_vel", Twist)),
        ("lcm", "/cmd_vel", ("/cmd_vel", None)),
        ("lcm", "/cmd_vel#geometry_msgs.Twist", ("/cmd_vel", Twist)),
    ],
)
def test_split_topic(
    monkeypatch: pytest.MonkeyPatch, transport: str, topic: str, expected: tuple[str, type | None]
) -> None:
    monkeypatch.setattr(global_config, "transport", transport)

    assert _split_topic(topic) == expected


def test_split_topic_rejects_unknown_type(capsys: pytest.CaptureFixture[str]) -> None:
    with pytest.raises(typer.Exit):
        _split_topic("/cmd_vel#geometry_msgs.Nope")

    assert "geometry_msgs.Nope" in capsys.readouterr().err


def test_topic_echo_reports_invalid_zenoh_key(
    monkeypatch: pytest.MonkeyPatch, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(global_config, "transport", "zenoh")

    with pytest.raises(typer.Exit):
        topic_echo("cmd?vel", None)

    assert "Invalid topic 'cmd?vel'" in capsys.readouterr().err


@pytest.mark.parametrize(
    ("backend", "topic"),
    [("lcm", "/test_tf#tf2_msgs.TFMessage"), ("zenoh", "dimos/test_tf/tf2_msgs.TFMessage")],
)
def test_topic_send_accepts_equivalent_wire_type(
    monkeypatch: pytest.MonkeyPatch, backend: str, topic: str
) -> None:
    monkeypatch.setattr(global_config, "transport", backend)
    collector = CallbackCollector(1)
    with make_transport("/test_tf", TFMessage) as receiver:
        receiver.subscribe(lambda msg: collector(msg, None))
        topic_send(topic, "Transform(translation=Vector3(1, 2, 3), ts=0.0)")
        collector.wait()

    received = collector.results[0][0]
    assert isinstance(received, TFMessage)
    assert received.transforms == [Transform(translation=Vector3(1, 2, 3), ts=0.0)]


@pytest.mark.parametrize("expression", ["Vector3()", "42"])
def test_topic_send_rejects_different_wire_type(
    monkeypatch: pytest.MonkeyPatch,
    mocker: MockerFixture,
    capsys: pytest.CaptureFixture[str],
    expression: str,
) -> None:
    monkeypatch.setattr(global_config, "transport", "zenoh")
    create_transport = mocker.patch("dimos.cli.topic.make_transport")

    with pytest.raises(typer.Exit) as exc:
        topic_send("dimos/test_topic/geometry_msgs.Twist", expression)

    assert exc.value.exit_code == 1
    assert "Topic expects Twist" in capsys.readouterr().err
    create_transport.assert_not_called()


def test_topic_echo_closes_typed_transport(
    monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture
) -> None:
    monkeypatch.setattr(global_config, "transport", "lcm")
    transport = make_transport("/test_topic_echo", Twist)
    with transport:
        stop = mocker.spy(transport, "stop")
        mocker.patch("dimos.cli.topic.make_transport", return_value=transport)
        mocker.patch("dimos.cli.topic.time.sleep", side_effect=KeyboardInterrupt)

        topic_echo("/test_topic_echo#geometry_msgs.Twist", None)

        stop.assert_called_once_with()


def test_topic_send_reports_invalid_zenoh_key(
    monkeypatch: pytest.MonkeyPatch, mocker: MockerFixture, capsys: pytest.CaptureFixture[str]
) -> None:
    monkeypatch.setattr(global_config, "transport", "zenoh")
    create_transport = mocker.patch("dimos.cli.topic.make_transport")

    with pytest.raises(typer.Exit):
        topic_send("cmd?vel", "Twist()")

    assert "Invalid topic 'cmd?vel'" in capsys.readouterr().err
    create_transport.assert_not_called()
