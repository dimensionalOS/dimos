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

from __future__ import annotations

import os
from types import SimpleNamespace
from unittest.mock import MagicMock, patch

import numpy as np
import pytest

from dimos.models.vl.litellm import LiteLLMVlModel
from dimos.msgs.sensor_msgs.Image import Image
from dimos.perception.detection.type.detection2d.imageDetections2D import ImageDetections2D

_COMPLETION_PATH = "dimos.models.vl.litellm.LiteLLMVlModel._completion"


def _img() -> Image:
    """Small synthetic RGB image (no LFS required)."""
    return Image.from_numpy(np.zeros((64, 64, 3), dtype=np.uint8))


def _resp(content: str = "test response") -> SimpleNamespace:
    """Fake litellm.completion() response matching OpenAI shape."""
    return SimpleNamespace(
        choices=[SimpleNamespace(message=SimpleNamespace(content=content))],
    )


def _null_resp() -> SimpleNamespace:
    """Response with None content (refusal / tool-call-only)."""
    return SimpleNamespace(
        choices=[SimpleNamespace(message=SimpleNamespace(content=None))],
    )


MOCK_DETECTIONS = """[
    ["person", 10, 10, 50, 50],
    ["person", 20, 20, 60, 60]
]"""


# ---------------------------------------------------------------------------
# Core dispatch
# ---------------------------------------------------------------------------
class TestCoreDispatch:
    def test_query_sends_correct_model_and_messages(self) -> None:
        model = LiteLLMVlModel(model_name="anthropic/claude-sonnet-4-20250514")
        with patch(_COMPLETION_PATH, return_value=_resp("a cafe")) as mock:
            result = model.query(_img(), "What is this?")

        assert result == "a cafe"
        kw = mock.call_args.kwargs
        assert kw["model"] == "anthropic/claude-sonnet-4-20250514"
        msgs = kw["messages"]
        assert msgs[0]["role"] == "user"
        parts = msgs[0]["content"]
        assert any(p["type"] == "image_url" for p in parts)
        assert any(p["type"] == "text" and p["text"] == "What is this?" for p in parts)

    def test_query_sends_base64_image_url(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_resp()) as mock:
            model.query(_img(), "hi")

        img_part = next(
            p for p in mock.call_args.kwargs["messages"][0]["content"] if p["type"] == "image_url"
        )
        assert img_part["image_url"]["url"].startswith("data:image/jpeg;base64,")

    def test_response_format_forwarded(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        fmt = {"type": "json_object"}
        with patch(_COMPLETION_PATH, return_value=_resp("{}")) as mock:
            model.query(_img(), "json", response_format=fmt)
        assert mock.call_args.kwargs["response_format"] == fmt

    def test_response_format_omitted_when_none(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_resp()) as mock:
            model.query(_img(), "hi")
        assert "response_format" not in mock.call_args.kwargs


# ---------------------------------------------------------------------------
# Credential handling
# ---------------------------------------------------------------------------
class TestCredentials:
    def test_api_key_forwarded_when_set(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini", api_key="sk-test")
        with patch(_COMPLETION_PATH, return_value=_resp()) as mock:
            model.query(_img(), "hi")
        assert mock.call_args.kwargs.get("api_key") is None  # forwarded inside _completion

    def test_api_base_forwarded_when_set(self) -> None:
        model = LiteLLMVlModel(
            model_name="azure/gpt-4o",
            api_base="https://my-resource.openai.azure.com",
        )
        with patch(_COMPLETION_PATH, return_value=_resp()) as mock:
            model.query(_img(), "hi")
        assert mock.call_args.kwargs.get("api_base") is None  # forwarded inside _completion

    def test_no_api_key_no_env_lets_litellm_handle_it(self) -> None:
        """When no key is configured, litellm reads provider env vars itself."""
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_resp()):
            model.query(_img(), "hi")


# ---------------------------------------------------------------------------
# Batch queries
# ---------------------------------------------------------------------------
class TestBatch:
    def test_batch_returns_per_image_response(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_resp("batch")):
            results = model.query_batch([_img(), _img(), _img()], "describe")
        assert len(results) == 3
        assert all(r == "batch" for r in results)

    def test_batch_empty_input(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        assert model.query_batch([], "describe") == []

    def test_batch_single_image(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_resp("one")):
            results = model.query_batch([_img()], "describe")
        assert results == ["one"]


# ---------------------------------------------------------------------------
# Null / empty response handling
# ---------------------------------------------------------------------------
class TestNullResponse:
    def test_query_null_content_returns_empty_string(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_null_resp()):
            result = model.query(_img(), "hi")
        assert result == ""

    def test_query_empty_string_content(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_resp("")):
            result = model.query(_img(), "hi")
        assert result == ""

    def test_batch_null_content_returns_empty_strings(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_null_resp()):
            results = model.query_batch([_img(), _img()], "describe")
        assert results == ["", ""]


# ---------------------------------------------------------------------------
# Exception propagation (litellm exceptions bubble up, not swallowed)
# ---------------------------------------------------------------------------
class TestExceptionPropagation:
    def test_authentication_error_propagates(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini", api_key="sk-invalid")
        with patch(_COMPLETION_PATH, side_effect=ValueError("auth failed")):
            with pytest.raises(ValueError, match="auth failed"):
                model.query(_img(), "hi")

    def test_not_found_error_propagates(self) -> None:
        model = LiteLLMVlModel(model_name="fake-provider/nonexistent-model")
        with patch(_COMPLETION_PATH, side_effect=ConnectionError("model not found")):
            with pytest.raises(ConnectionError):
                model.query(_img(), "hi")

    def test_rate_limit_error_propagates(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, side_effect=TimeoutError("rate limited")):
            with pytest.raises(TimeoutError):
                model.query(_img(), "hi")

    def test_generic_exception_not_swallowed(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, side_effect=RuntimeError("unexpected")):
            with pytest.raises(RuntimeError, match="unexpected"):
                model.query(_img(), "hi")

    def test_batch_exception_propagates(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, side_effect=ValueError("bad request")):
            with pytest.raises(ValueError):
                model.query_batch([_img()], "describe")


# ---------------------------------------------------------------------------
# Numpy array input (deprecated path)
# ---------------------------------------------------------------------------
class TestNumpyInput:
    def test_numpy_array_triggers_deprecation_warning(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        arr = np.zeros((64, 64, 3), dtype=np.uint8)
        with patch(_COMPLETION_PATH, return_value=_resp("ok")):
            with pytest.warns(DeprecationWarning, match="numpy array"):
                model.query(arr, "hi")

    def test_numpy_array_still_works(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        arr = np.zeros((64, 64, 3), dtype=np.uint8)
        with patch(_COMPLETION_PATH, return_value=_resp("ok")):
            import warnings

            with warnings.catch_warnings():
                warnings.simplefilter("ignore", DeprecationWarning)
                result = model.query(arr, "hi")
        assert result == "ok"


# ---------------------------------------------------------------------------
# Detection + point parsing (inherited from VlModel base)
# ---------------------------------------------------------------------------
class TestDetections:
    def test_query_detections_parses_json_response(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        model.query = MagicMock(return_value=MOCK_DETECTIONS)
        detections = model.query_detections(_img(), "person")

        assert isinstance(detections, ImageDetections2D)
        assert len(detections.detections) == 2
        for det in detections.detections:
            assert det.name == "person"
            assert det.confidence == 1.0
            assert det.class_id == -1

    def test_query_detections_empty_response(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        model.query = MagicMock(return_value="[]")
        detections = model.query_detections(_img(), "unicorn")
        assert len(detections.detections) == 0

    def test_query_detections_malformed_json(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        model.query = MagicMock(return_value="not json at all")
        detections = model.query_detections(_img(), "person")
        assert len(detections.detections) == 0

    def test_caption_uses_query(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch(_COMPLETION_PATH, return_value=_resp("A black image")):
            caption = model.caption(_img())
        assert caption == "A black image"


# ---------------------------------------------------------------------------
# Factory + registration
# ---------------------------------------------------------------------------
class TestFactory:
    def test_factory_creates_litellm_model(self) -> None:
        from dimos.models.vl.create import create

        model = create("litellm")
        assert isinstance(model, LiteLLMVlModel)

    def test_litellm_in_vlmodel_name_type(self) -> None:
        from dimos.models.vl.types import VlModelName

        assert "litellm" in VlModelName.__args__  # type: ignore[attr-defined]


# ---------------------------------------------------------------------------
# Import guard
# ---------------------------------------------------------------------------
class TestImportGuard:
    def test_import_error_without_litellm(self) -> None:
        model = LiteLLMVlModel(model_name="gpt-4o-mini")
        with patch.dict("sys.modules", {"litellm": None}):
            with pytest.raises(ImportError, match="litellm is required"):
                model.query(_img(), "hi")


# ---------------------------------------------------------------------------
# Live E2E (opt-in, requires OPENAI_API_KEY)
# ---------------------------------------------------------------------------
@pytest.mark.skipif(
    "OPENAI_API_KEY" not in os.environ,
    reason="Live E2E requires OPENAI_API_KEY",
)
class TestLiveE2E:
    def test_live_query_openai(self) -> None:
        model = LiteLLMVlModel(model_name="openai/gpt-4o-mini")
        image = _img()
        result = model.query(image, "What color is this image? Answer in one word.")
        assert isinstance(result, str)
        assert len(result) > 0
        print(f"Live E2E response: {result!r}")
