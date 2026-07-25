from __future__ import annotations

from unittest.mock import patch

from dimos.models.vl.qwen import QwenVlModel


def test_qwen_vl_accepts_siliconflow_environment_overrides() -> None:
    environment = {
        "DIMOS_QWEN_VL_API_KEY": "test-siliconflow-key",
        "DIMOS_QWEN_VL_BASE_URL": "https://api.siliconflow.cn/v1",
        "DIMOS_QWEN_VL_MODEL": "Qwen/Qwen3-VL-8B-Instruct",
    }
    with (
        patch.dict("os.environ", environment, clear=False),
        patch("dimos.models.vl.qwen.OpenAI") as client_factory,
    ):
        model = QwenVlModel()
        _client = model._client
        assert model.model_name == "Qwen/Qwen3-VL-8B-Instruct"

    client_factory.assert_called_once_with(
        base_url="https://api.siliconflow.cn/v1",
        api_key="test-siliconflow-key",
    )


def test_qwen_vl_keeps_alibaba_defaults_when_no_override_is_configured() -> None:
    with (
        patch.dict(
            "os.environ",
            {"ALIBABA_API_KEY": "test-alibaba-key"},
            clear=True,
        ),
        patch("dimos.models.vl.qwen.OpenAI") as client_factory,
    ):
        model = QwenVlModel()
        _client = model._client

    client_factory.assert_called_once_with(
        base_url="https://dashscope-intl.aliyuncs.com/compatible-mode/v1",
        api_key="test-alibaba-key",
    )
    assert model.model_name == "qwen2.5-vl-72b-instruct"


def test_qwen_vl_requires_a_provider_key() -> None:
    with patch.dict("os.environ", {}, clear=True):
        model = QwenVlModel()

        try:
            _client = model._client
        except ValueError as error:
            assert "API key" in str(error)
        else:
            raise AssertionError("QwenVlModel accepted a missing API key")
