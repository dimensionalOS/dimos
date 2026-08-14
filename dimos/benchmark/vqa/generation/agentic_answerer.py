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

# Copyright 2026 Dimensional Inc.
"""Private bounded LangChain oracle for generic VQA question proposals."""

from __future__ import annotations

import json
import re
from typing import TYPE_CHECKING, Any

from dimos.benchmark.vqa.contracts import (
    AcceptedOracleResult,
    AnswerContract,
    BooleanAnswerContract,
    ChoiceAnswerContract,
    OracleToolResult,
    OracleTrace,
    QuestionProposal,
    RejectedOracleResult,
)
from dimos.benchmark.vqa.generation.agentic_tools import VqaPrimitiveToolRegistry

if TYPE_CHECKING:
    from langchain_core.language_models.chat_models import BaseChatModel


class AgenticAnswerer:
    """Use direct local tools only, then validate a model's final JSON response."""

    def __init__(
        self,
        model: BaseChatModel,
        max_tool_calls: int = 25,
    ) -> None:
        if max_tool_calls < 1:
            raise ValueError("max_tool_calls must be positive")
        self._model = model
        self._max_tool_calls = max_tool_calls

    def answer(
        self, proposal: QuestionProposal, registry: VqaPrimitiveToolRegistry
    ) -> AcceptedOracleResult | RejectedOracleResult:
        from langchain_core.messages import HumanMessage, SystemMessage, ToolMessage

        tools = registry.tools()
        model = self._model.bind_tools(tools)
        messages: list[Any] = [
            SystemMessage(
                "You are a private VQA oracle. Use only supplied local tools. Do not invent "
                "evidence. Detection IDs can only be passed to segment_object. Mask IDs can only "
                "be passed to ground_mask. Detection evidence supports visual presence, count, and "
                "image side; grounded evidence also supplies range and point support. If grounding "
                "fails, reject rather than trying unrelated tools. Finish with JSON only: either "
                '{"answer": value, "evidence_ids": [..]} or '
                '{"status": "rejected", "reason": "non-empty reason", "evidence_ids": [..]}. '
                "Rejected results must not include an answer."
            ),
            HumanMessage(_proposal_prompt(proposal)),
        ]
        trace: list[OracleTrace] = []
        calls = 0
        while calls < self._max_tool_calls:
            response = model.invoke(messages)
            messages.append(response)
            tool_calls = getattr(response, "tool_calls", [])
            if not tool_calls:
                return _validated_result(
                    proposal,
                    _response_text(response.content),
                    registry.results,
                    trace,
                )
            for call in tool_calls:
                if calls >= self._max_tool_calls:
                    break
                name = call.get("name")
                call_id = call.get("id")
                if not isinstance(call_id, str) or not call_id:
                    return _rejected(proposal, "malformed_tool_call_id", registry.results, trace)
                tool = next((item for item in tools if item.name == name), None)
                calls += 1
                if tool is None:
                    trace.append(OracleTrace("tool_error", f"unsupported_tool:{name}"))
                    messages.append(
                        ToolMessage(
                            content=json.dumps(
                                {"status": "error", "error": "unsupported_tool", "tool": name}
                            ),
                            tool_call_id=call_id,
                        )
                    )
                    continue
                try:
                    output = tool.invoke(call.get("args", {}))
                except Exception as exc:
                    trace.append(OracleTrace("tool_error", f"{name}:{exc}"))
                    messages.append(
                        ToolMessage(
                            content=json.dumps(
                                {"status": "error", "error": "tool_error", "detail": str(exc)}
                            ),
                            tool_call_id=call_id,
                        )
                    )
                    continue
                trace.append(OracleTrace("tool", str(name)))
                messages.append(ToolMessage(content=str(output), tool_call_id=call_id))
        return _rejected(proposal, "tool_call_limit", registry.results, trace)


def create_openai_oracle(model: str, max_tool_calls: int = 25) -> AgenticAnswerer:
    """Construct the private-only OpenAI tool-calling oracle."""
    from langchain_openai import ChatOpenAI

    return AgenticAnswerer(
        ChatOpenAI(model=model),
        max_tool_calls=max_tool_calls,
    )


def validate_oracle_answer(
    proposal: QuestionProposal,
    answer: Any,
    evidence_ids: Any,
    results: tuple[OracleToolResult, ...],
) -> str:
    """Deterministically validate an answer format and citations."""
    return _resolve_oracle_answer(proposal, answer, evidence_ids, results)[0]


def _resolve_oracle_answer(
    proposal: QuestionProposal,
    answer: Any,
    evidence_ids: Any,
    results: tuple[OracleToolResult, ...],
) -> tuple[str, AnswerContract]:
    """Return a validated answer and the public contract resolved from private evidence."""
    if (
        not isinstance(evidence_ids, list)
        or not evidence_ids
        or not all(isinstance(item, str) for item in evidence_ids)
    ):
        raise ValueError("answer requires non-empty evidence_ids")
    known_ids = {item.id for result in results for item in result.evidence}
    if not set(evidence_ids).issubset(known_ids):
        raise ValueError("answer cites unknown evidence")
    contract = proposal.answer_contract
    if isinstance(contract, BooleanAnswerContract):
        normalized = _normalize_boolean_answer(answer)
        if normalized is None:
            raise ValueError("boolean answer must be yes or no")
        return normalized, contract
    if isinstance(contract, ChoiceAnswerContract):
        normalized = _normalize_choice_answer(answer, contract.choices)
        if normalized is None:
            raise ValueError("choice answer is not allowed")
        return normalized, contract
    raise ValueError("unsupported answer contract")


def _normalize_boolean_answer(answer: Any) -> str | None:
    if isinstance(answer, bool):
        return "yes" if answer else "no"
    if isinstance(answer, str):
        normalized = answer.strip().casefold()
        if normalized in ("yes", "true"):
            return "yes"
        if normalized in ("no", "false"):
            return "no"
    return None


def _normalize_choice_answer(answer: Any, choices: tuple[str, ...]) -> str | None:
    if not isinstance(answer, str):
        return None
    normalized_choices: dict[str, str] = {}
    for choice in choices:
        key = choice.strip().rstrip(".,;:").casefold()
        if key in normalized_choices:
            return None
        normalized_choices[key] = choice
    return normalized_choices.get(answer.strip().rstrip(".,;:").casefold())


def _validated_result(
    proposal: QuestionProposal,
    response: str,
    results: tuple[OracleToolResult, ...],
    trace: list[OracleTrace],
) -> AcceptedOracleResult | RejectedOracleResult:
    try:
        payload = _parse_json_object(response)
        if payload.get("status") == "rejected":
            if set(payload) - {"status", "reason", "evidence_ids"}:
                raise ValueError("rejected final response contains unsupported fields")
            reason = payload.get("reason")
            evidence_ids = payload.get("evidence_ids")
            if not isinstance(reason, str) or not reason:
                raise ValueError("rejected final response requires non-empty reason")
            if evidence_ids is not None and (
                not isinstance(evidence_ids, list)
                or not all(isinstance(item, str) for item in evidence_ids)
            ):
                raise ValueError("rejected evidence_ids must be a list of strings")
            return _rejected(proposal, reason, results, trace)
        answer, answer_contract = _resolve_oracle_answer(
            proposal, payload.get("answer"), payload.get("evidence_ids"), results
        )
        evidence_ids = tuple(payload["evidence_ids"])
    except (ValueError, json.JSONDecodeError, AttributeError) as exc:
        return _rejected(proposal, f"invalid_final_answer:{exc}", results, trace)
    return AcceptedOracleResult(
        proposal, answer, answer_contract, evidence_ids, results, tuple(trace)
    )


def _rejected(
    proposal: QuestionProposal,
    reason: str,
    results: tuple[OracleToolResult, ...],
    trace: list[OracleTrace],
) -> RejectedOracleResult:
    return RejectedOracleResult(proposal, reason, results, tuple(trace))


def _proposal_prompt(proposal: QuestionProposal) -> str:
    return (
        f"Question: {proposal.question}\nAnswer contract: {_contract_prompt(proposal.answer_contract)}\n"
        f"Suggested object queries: {', '.join(proposal.object_queries) or 'none'}"
    )


def _contract_prompt(contract: AnswerContract) -> str:
    if isinstance(contract, BooleanAnswerContract):
        return "boolean: yes or no"
    if isinstance(contract, ChoiceAnswerContract):
        return f"choice: {', '.join(contract.choices)}"
    raise ValueError("unsupported answer contract")


def _parse_json_object(response: str) -> dict[str, Any]:
    stripped = re.sub(r"^```(?:json)?\s*|\s*```$", "", response.strip(), flags=re.IGNORECASE)
    start, end = stripped.find("{"), stripped.rfind("}")
    if start < 0 or end < start:
        raise json.JSONDecodeError("expected JSON object", stripped, 0)
    payload: Any = json.loads(stripped[start : end + 1])
    if not isinstance(payload, dict):
        raise ValueError("final response must be an object")
    return payload


def _response_text(content: Any) -> str:
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "".join(str(item.get("text", "")) for item in content if isinstance(item, dict))
    return str(content)
