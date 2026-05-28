from __future__ import annotations

import asyncio
from io import BytesIO
from pathlib import Path
from types import ModuleType, SimpleNamespace
from typing import ClassVar
from unittest.mock import AsyncMock, patch
import base64
import sys

from fastapi import FastAPI
from fastapi.testclient import TestClient

import policy
import tts


def _package(name: str) -> ModuleType:
    module = sys.modules.get(name)
    if module is None:
        module = ModuleType(name)
        sys.modules[name] = module
    module.__path__ = []  # type: ignore[attr-defined]
    return module


def _module(name: str, **attrs: object) -> ModuleType:
    module = ModuleType(name)
    for attr, value in attrs.items():
        setattr(module, attr, value)
    sys.modules[name] = module
    return module


class _FastAPIServer:
    def __init__(self, **_: object) -> None:
        self.app = FastAPI()

    def run(self, **_: object) -> None:
        return None


class _Record3DSource:
    def __init__(self, *_: object, **__: object) -> None:
        return None


class _UnitreeWebRTCConnection:
    def __init__(self, *_: object, **__: object) -> None:
        return None

    def stop(self) -> None:
        return None


class _Image:
    def to_base64(self, *_: object, **__: object) -> str:
        return ""


class _PointCloud2:
    def as_numpy(self) -> tuple[object, object]:
        return ([], None)


def _setup_logger() -> SimpleNamespace:
    return SimpleNamespace(
        info=lambda *_args, **_kwargs: None,
        exception=lambda *_args, **_kwargs: None,
        warning=lambda *_args, **_kwargs: None,
        error=lambda *_args, **_kwargs: None,
        debug=lambda *_args, **_kwargs: None,
    )


for package in [
    "dimos",
    "dimos.experimental",
    "dimos.experimental.fetch",
    "dimos.msgs",
    "dimos.msgs.geometry_msgs",
    "dimos.msgs.sensor_msgs",
    "dimos.robot",
    "dimos.robot.unitree",
    "dimos.utils",
    "dimos.web",
    "dimos.web.dimos_interface",
    "dimos.web.dimos_interface.api",
]:
    _package(package)

sys.modules["dimos.experimental.fetch.policy"] = policy
sys.modules["dimos.experimental.fetch.tts"] = tts
_module(
    "cv2",
    COLORMAP_TURBO=0,
    FONT_HERSHEY_SIMPLEX=0,
    IMWRITE_JPEG_QUALITY=1,
    applyColorMap=lambda values, _map: values,
    circle=lambda *_args, **_kwargs: None,
    imencode=lambda *_args, **_kwargs: (True, SimpleNamespace(tobytes=lambda: b"jpg")),
    line=lambda *_args, **_kwargs: None,
    putText=lambda *_args, **_kwargs: None,
)
_module("dimos.experimental.fetch.record3d_source", Record3DSource=_Record3DSource)
_module(
    "dimos.msgs.geometry_msgs.Twist",
    Twist=lambda **kwargs: SimpleNamespace(**kwargs),
)
_module(
    "dimos.msgs.geometry_msgs.Vector3",
    Vector3=lambda *args: args,
)
_module("dimos.msgs.sensor_msgs.Image", Image=_Image)
_module("dimos.msgs.sensor_msgs.PointCloud2", PointCloud2=_PointCloud2)
_module(
    "dimos.robot.unitree.connection",
    UnitreeWebRTCConnection=_UnitreeWebRTCConnection,
)
_module("dimos.utils.logging_config", setup_logger=_setup_logger)
_module("dimos.utils.path_utils", get_project_root=lambda: Path.cwd())
_module("dimos.web.dimos_interface.api.server", FastAPIServer=_FastAPIServer)
_module("unitree_webrtc_connect", constants=SimpleNamespace(RTC_TOPIC={"SPORT_MOD": "sport"}))
_module("unitree_webrtc_connect.constants", RTC_TOPIC={"SPORT_MOD": "sport"})

import iphone_middleware


_LOOP = None


def _run(coro):
    """Run a coroutine on a single reused loop, left set as current, so this
    module never closes/clears the loop that sibling test modules rely on."""
    global _LOOP
    if _LOOP is None or _LOOP.is_closed():
        _LOOP = asyncio.new_event_loop()
    asyncio.set_event_loop(_LOOP)
    return _LOOP.run_until_complete(coro)


def test_realtime_session_config_defaults() -> None:
    session = iphone_middleware._build_realtime_session_config(
        model=iphone_middleware.DEFAULT_REALTIME_MODEL,
        voice="echo",
        reasoning_effort=iphone_middleware.DEFAULT_REALTIME_REASONING_EFFORT,
    )

    assert session["type"] == "realtime"
    assert session["model"] == "gpt-realtime-2"
    assert session["output_modalities"] == ["audio"]
    assert session["audio"]["output"]["voice"] == "echo"
    assert session["reasoning"]["effort"] == "low"
    assert "Speak only the exact Fetch dog line" in session["instructions"]


def test_realtime_client_secret_requires_openai_key(monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    middleware = iphone_middleware.FetchIphoneMiddleware(
        tts_provider="openai",
        enable_realtime=True,
    )

    with patch("iphone_middleware.OpenAI") as openai_cls:
        response = TestClient(middleware.server.app).post("/realtime/client-secret")

    openai_cls.assert_not_called()
    assert response.status_code == 503
    assert "OPENAI_API_KEY" in response.json()["error"]


def test_realtime_client_secret_disabled_by_default(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "openai-key")
    middleware = iphone_middleware.FetchIphoneMiddleware()

    with patch("iphone_middleware.OpenAI") as openai_cls:
        response = TestClient(middleware.server.app).post("/realtime/client-secret")

    openai_cls.assert_not_called()
    assert response.status_code == 404
    assert "Realtime is disabled" in response.json()["error"]


def test_realtime_client_secret_uses_session_config(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "openai-key")
    middleware = iphone_middleware.FetchIphoneMiddleware(
        tts_provider="openai",
        enable_realtime=True,
        realtime_model="gpt-realtime-2",
        realtime_reasoning_effort="low",
        tts_voice="echo",
    )

    with patch("iphone_middleware.OpenAI") as openai_cls:
        openai_client = openai_cls.return_value
        openai_client.realtime.client_secrets.create.return_value = SimpleNamespace(
            model_dump=lambda mode="json": {"client_secret": {"value": "rt-secret"}}
        )

        response = TestClient(middleware.server.app).post("/realtime/client-secret")

    assert response.status_code == 200
    openai_cls.assert_called_once_with(
        timeout=policy.DEFAULT_REQUEST_TIMEOUT_S,
        max_retries=policy.DEFAULT_MAX_RETRIES,
    )
    call_kwargs = openai_client.realtime.client_secrets.create.call_args.kwargs
    session = call_kwargs["session"]
    assert session["model"] == "gpt-realtime-2"
    assert session["audio"]["output"]["voice"] == "echo"
    assert session["reasoning"]["effort"] == "low"
    assert response.json()["client_secret"]["value"] == "rt-secret"
    assert response.json()["model"] == "gpt-realtime-2"


def test_hello_defaults_to_cartesia_speak_route() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware()

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["tts_provider"] == "cartesia"
    assert hello["audio_route"] == "speak"
    assert hello["realtime_enabled"] is False


def test_cli_defaults_to_cartesia_tts(monkeypatch) -> None:
    monkeypatch.setattr(sys, "argv", ["iphone_middleware"])

    args = iphone_middleware._parse_args()

    assert args.tts_provider == "cartesia"
    assert args.tts_model == "tts-1"


def test_hello_conversation_enabled() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware(
        conversation_mode="gemini_live",
    )

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["conversation_enabled"] is True
    assert hello["audio_route"] == "gemini_live_conversation"
    assert hello["conversation_model"] == tts.DEFAULT_GEMINI_TTS_MODEL


def test_hello_conversation_disabled_by_default() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware()

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["conversation_enabled"] is False
    assert hello["audio_route"] == "speak"


def test_hello_advertises_gemini_speak_route() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware(
        tts_provider="gemini",
        enable_realtime=True,
    )

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["tts_provider"] == "gemini"
    assert hello["audio_route"] == "speak"
    assert hello["realtime_enabled"] is False


def test_hello_advertises_openai_realtime_when_enabled() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware(
        tts_provider="openai",
        enable_realtime=True,
    )

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["tts_provider"] == "openai"
    assert hello["audio_route"] == "realtime_then_speak"
    assert hello["realtime_enabled"] is True


class _FakeConversationSession:
    instances: ClassVar[list["_FakeConversationSession"]] = []

    def __init__(self, **kwargs: object) -> None:
        self.kwargs = kwargs
        self.opened = False
        self.closed = False
        self.mic: list[bytes] = []
        self.finished = False
        self._run = asyncio.Event()
        _FakeConversationSession.instances.append(self)

    async def open(self) -> None:
        self.opened = True

    async def run(self) -> None:
        await self._run.wait()

    async def close(self) -> None:
        self.closed = True
        self._run.set()

    def push_mic(self, pcm: bytes) -> None:
        self.mic.append(pcm)


def test_conversation_start_opens_session() -> None:
    _FakeConversationSession.instances = []
    middleware = iphone_middleware.FetchIphoneMiddleware(conversation_mode="gemini_live")

    with patch("iphone_middleware.LiveConversationSession", _FakeConversationSession):
        with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
            ws.receive_json()  # hello
            ws.send_json({"type": "conversation_start", "context": "nice straw hat"})
            state = ws.receive_json()

    assert state["type"] == "conversation_state"
    assert state["state"] == "active"
    assert len(_FakeConversationSession.instances) == 1
    session = _FakeConversationSession.instances[0]
    assert session.opened is True
    assert session.kwargs["system_context"] == "nice straw hat"
    assert session.kwargs["model"] == tts.DEFAULT_GEMINI_TTS_MODEL


def test_mic_chunk_forwarded_to_session() -> None:
    _FakeConversationSession.instances = []
    middleware = iphone_middleware.FetchIphoneMiddleware(conversation_mode="gemini_live")

    with patch("iphone_middleware.LiveConversationSession", _FakeConversationSession):
        with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
            ws.receive_json()  # hello
            ws.send_json({"type": "conversation_start", "context": ""})
            ws.receive_json()  # conversation_state active
            ws.send_json({"type": "mic_chunk", "data": "aGVsbG8="})
            ws.send_json({"type": "conversation_stop"})

    session = _FakeConversationSession.instances[0]
    assert session.mic == [b"hello"]
    assert session.closed is True


def test_hello_advertises_provider_availability(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "openai-key")
    monkeypatch.setenv("GEMINI_API_KEY", "gemini-key")
    middleware = iphone_middleware.FetchIphoneMiddleware()

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["openai_available"] is True
    assert hello["gemini_available"] is True


def test_hello_reports_missing_keys(monkeypatch) -> None:
    monkeypatch.delenv("OPENAI_API_KEY", raising=False)
    monkeypatch.delenv("GEMINI_API_KEY", raising=False)
    monkeypatch.setenv("GOOGLE_API_KEY", "google-key")
    middleware = iphone_middleware.FetchIphoneMiddleware()

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["openai_available"] is False
    assert hello["gemini_available"] is True  # GOOGLE_API_KEY counts as Gemini


def test_conversation_start_forwards_voice_and_model() -> None:
    _FakeConversationSession.instances = []
    middleware = iphone_middleware.FetchIphoneMiddleware(conversation_mode="gemini_live")

    with patch("iphone_middleware.LiveConversationSession", _FakeConversationSession):
        with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
            ws.receive_json()  # hello
            ws.send_json({
                "type": "conversation_start",
                "context": "hi",
                "voice": "nova",
                "model": "gemini-2.5-flash-native-audio-preview-12-2025",
            })
            ws.receive_json()  # conversation_state active

    session = _FakeConversationSession.instances[0]
    assert session.kwargs["voice"] == "nova"
    assert session.kwargs["model"] == "gemini-2.5-flash-native-audio-preview-12-2025"


def test_speak_provider_override_to_openai(monkeypatch) -> None:
    monkeypatch.setenv("OPENAI_API_KEY", "openai-key")
    middleware = iphone_middleware.FetchIphoneMiddleware(tts_provider="gemini", tts_voice="echo")
    gemini_tts = AsyncMock(return_value=b"RIFF")

    with (
        patch("iphone_middleware.gemini_live_tts", new=gemini_tts),
        patch("iphone_middleware.OpenAI") as openai_cls,
    ):
        openai_client = openai_cls.return_value
        openai_client.audio.speech.create.return_value = SimpleNamespace(content=b"mp3bytes")
        response = TestClient(middleware.server.app).post(
            "/speak", json={"text": "hi", "provider": "openai", "voice": "nova"}
        )

    gemini_tts.assert_not_awaited()
    assert response.status_code == 200
    assert response.headers["content-type"] == "audio/mpeg"
    assert openai_client.audio.speech.create.call_args.kwargs["voice"] == "nova"


def test_speak_cartesia_provider(monkeypatch) -> None:
    monkeypatch.setenv("CARTESIA_API_KEY", "cartesia-key")
    middleware = iphone_middleware.FetchIphoneMiddleware(tts_voice="echo")
    cartesia = AsyncMock(return_value=b"RIFFwav")

    with patch("iphone_middleware.cartesia_tts", new=cartesia):
        response = TestClient(middleware.server.app).post(
            "/speak", json={"text": "hi", "provider": "cartesia"}
        )

    assert response.status_code == 200
    assert response.headers["content-type"] == "audio/wav"
    assert response.content == b"RIFFwav"
    cartesia.assert_awaited_once_with("hi", voice=tts.map_voice("echo", "cartesia"))


def test_speak_defaults_to_cartesia(monkeypatch) -> None:
    monkeypatch.setenv("CARTESIA_API_KEY", "cartesia-key")
    middleware = iphone_middleware.FetchIphoneMiddleware(tts_voice="echo")
    cartesia = AsyncMock(return_value=b"RIFFwav")

    with patch("iphone_middleware.cartesia_tts", new=cartesia):
        response = TestClient(middleware.server.app).post("/speak", json={"text": "hi"})

    assert response.status_code == 200
    assert response.headers["content-type"] == "audio/wav"
    assert response.content == b"RIFFwav"
    cartesia.assert_awaited_once_with("hi", voice=tts.map_voice("echo", "cartesia"))


def test_hello_advertises_cartesia_availability(monkeypatch) -> None:
    monkeypatch.setenv("CARTESIA_API_KEY", "cartesia-key")
    middleware = iphone_middleware.FetchIphoneMiddleware()

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        hello = ws.receive_json()

    assert hello["cartesia_available"] is True


def test_speak_provider_override_to_gemini(monkeypatch) -> None:
    monkeypatch.setenv("GEMINI_API_KEY", "gemini-key")
    middleware = iphone_middleware.FetchIphoneMiddleware(tts_provider="openai", tts_voice="echo")
    gemini_tts = AsyncMock(return_value=b"RIFFwav")

    with (
        patch("iphone_middleware.gemini_live_tts", new=gemini_tts),
        patch("iphone_middleware.OpenAI") as openai_cls,
    ):
        response = TestClient(middleware.server.app).post(
            "/speak", json={"text": "hi", "provider": "gemini", "voice": "echo"}
        )

    openai_cls.assert_not_called()
    gemini_tts.assert_awaited_once_with("hi", voice="Charon")
    assert response.status_code == 200
    assert response.headers["content-type"] == "audio/wav"


def test_route_frame_safe_swallows_errors_and_notifies_client() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware()
    sent: list[dict] = []

    async def send_json(message: dict) -> None:
        sent.append(message)

    async def boom(*_args: object, **_kwargs: object) -> dict:
        raise RuntimeError("analyze boom")

    middleware._analyze_message = boom  # type: ignore[assignment]
    _run(middleware._route_frame_safe(send_json, {"type": "frame"}, "frame"))

    assert any(m.get("type") == "error" for m in sent)


def test_conversation_frame_preserves_centering_motion() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware()
    sent: list[dict] = []
    pushed: list[dict] = []

    async def send_json(message: dict) -> None:
        sent.append(message)

    async def analyze(*_args: object, **_kwargs: object) -> dict:
        return {
            "type": "decision",
            "state": "wait_for_coke",
            "simulated_cmd_vel": {"linear_x": 0.0, "angular_z": 0.28, "duration_s": 0.45},
        }

    middleware._analyze_message = analyze  # type: ignore[assignment]
    middleware._conv_session = SimpleNamespace(
        finished=False,
        push_frame_state=lambda decision: pushed.append(decision),
    )
    _run(middleware._route_frame(send_json, {"type": "dog_frame"}, "dog_frame"))

    assert pushed
    assert sent[0]["simulated_cmd_vel"] == {
        "linear_x": 0.0,
        "angular_z": 0.28,
        "duration_s": 0.45,
    }


def test_conversation_start_rejected_when_disabled() -> None:
    middleware = iphone_middleware.FetchIphoneMiddleware()

    with TestClient(middleware.server.app).websocket_connect("/fetch/ws") as ws:
        ws.receive_json()  # hello
        ws.send_json({"type": "conversation_start", "context": ""})
        message = ws.receive_json()

    assert message["type"] == "error"
    assert "disabled" in message["message"]


def test_speak_uses_gemini_tts_without_openai_client(monkeypatch) -> None:
    monkeypatch.setenv("GEMINI_API_KEY", "gemini-key")
    middleware = iphone_middleware.FetchIphoneMiddleware(
        tts_provider="gemini",
        tts_voice="echo",
    )
    gemini_tts = AsyncMock(return_value=b"RIFFwav")

    with (
        patch("iphone_middleware.gemini_live_tts", new=gemini_tts),
        patch("iphone_middleware.OpenAI") as openai_cls,
    ):
        response = TestClient(middleware.server.app).post(
            "/speak",
            json={"text": "Hello beach!"},
        )

    openai_cls.assert_not_called()
    gemini_tts.assert_awaited_once_with(
        "Hello beach!",
        voice="Charon",
    )
    assert response.status_code == 200
    assert response.content == b"RIFFwav"
    assert response.headers["content-type"] == "audio/wav"


def test_save_photo_writes_local_preview_icloud_and_google_drive(monkeypatch, tmp_path) -> None:
    capture_dir = tmp_path / "captures"
    icloud_dir = tmp_path / "iCloud Drive" / "fetch"
    google_drive_dir = tmp_path / "Google Drive" / "robodog-fetch"
    monkeypatch.setattr(iphone_middleware, "CAPTURE_DIR", capture_dir)
    monkeypatch.setattr(iphone_middleware, "ICLOUD_FETCH_DIR", icloud_dir)
    monkeypatch.setattr(iphone_middleware, "GOOGLE_DRIVE_FETCH_DIR", google_drive_dir)
    from PIL import Image as PILImage

    source = PILImage.new("RGB", (640, 360), (80, 90, 100))
    buffer = BytesIO()
    source.save(buffer, format="JPEG")
    image_bytes = buffer.getvalue()
    encoded = base64.b64encode(image_bytes).decode("ascii")
    middleware = iphone_middleware.FetchIphoneMiddleware()

    response = TestClient(middleware.server.app).post(
        "/photos/save",
        json={"image_data_url": f"data:image/jpeg;base64,{encoded}"},
    )

    assert response.status_code == 200
    payload = response.json()
    assert payload["saved"] is True
    assert payload["url"].startswith("/fetch/static/captures/fetch-")
    saved_bytes = Path(payload["path"]).read_bytes()
    assert saved_bytes.startswith(b"\xff\xd8")
    assert saved_bytes != image_bytes
    assert Path(payload["icloud_path"]).read_bytes() == saved_bytes
    assert Path(payload["google_drive_path"]).read_bytes() == saved_bytes
    assert Path(payload["path"]).parent == capture_dir
    assert Path(payload["icloud_path"]).parent == icloud_dir
    assert Path(payload["google_drive_path"]).parent == google_drive_dir
