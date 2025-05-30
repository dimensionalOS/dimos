import pytest
from pydantic import BaseModel
from openai._types import NOT_GIVEN

from dimos.agents.cerebras_agent import CerebrasAgent


# Define dummy and real Pydantic models
class DummyResponseModel:
    pass


class RealResponseModel(BaseModel):
    items: list[int]


@pytest.fixture
def agent_error(monkeypatch):
    """
    Returns a CerebrasAgent that will error with an unsupported JSON schema.
    """
    agent = CerebrasAgent(
        dev_name="test_error_agent",
        input_query_stream=None,
        skills=None,
        response_model=RealResponseModel,
    )

    # Monkey-patch the Cerebras API to raise the exact schema error
    def fake_create(**kwargs):
        raise ValueError(
            "Error code: 400 - {'message': \"Unsupported JSON schema fields: {'prefixItems', 'minItems', 'maxItems'}\", 'type': 'invalid_request_error', 'param': 'response_format', 'code': 'wrong_api_format'}"
        )

    monkeypatch.setattr(agent.client.chat.completions, "create", fake_create)
    return agent


def test_schema_error_through_pipeline(agent_error):
    """
    Exercising the full LLMAgent pipeline (run_observable_query) should surface
    the schema validation error from the Cerebras API.
    """
    results = []
    errors = []

    # Kick off the end-to-end observable query
    observable = agent_error.run_observable_query(query_text="ping")
    observable.subscribe(on_next=lambda x: results.append(x), on_error=lambda e: errors.append(e))

    # We expect no successful results, and exactly one error containing our message
    assert results == [], f"Unexpected results: {results}"
    assert len(errors) == 1, f"Expected one error, got: {errors}"
    assert "Unsupported JSON schema fields" in str(errors[0]), (
        f"Error message did not mention schema fields: {errors[0]}"
    )


if __name__ == "__main__":
    pytest.main([__file__])
