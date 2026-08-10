# Copyright 2026 Dimensional Inc.
"""Private bounded LangChain oracle for generic VQA question proposals."""

from __future__ import annotations

from dataclasses import asdict, dataclass, replace
import json
import re
from typing import TYPE_CHECKING, Any, Protocol

from dimos.benchmark.vqa.generation.oracle_tools import LocalOracleToolRegistry
from dimos.benchmark.vqa.models import (
    AcceptedOracleResult,
    AnswerContract,
    BooleanAnswerContract,
    ChoiceAnswerContract,
    DeferredHeightChoiceContract,
    OracleToolResult,
    OracleTrace,
    QuestionProposal,
    RejectedOracleResult,
    ResolvedAnswerContract,
)

if TYPE_CHECKING:
    from langchain_core.language_models.chat_models import BaseChatModel


@dataclass(frozen=True)
class SemanticEvidenceValidation:
    """Private verdict on whether cited tool evidence supports an oracle answer."""

    accepted: bool
    reason: str


class SemanticEvidenceValidator(Protocol):
    """Validate an answer only against the frozen question and cited local evidence."""

    def validate(
        self,
        proposal: QuestionProposal,
        answer: str,
        cited_results: tuple[OracleToolResult, ...],
    ) -> SemanticEvidenceValidation: ...


class OpenAISemanticEvidenceValidator:
    """Private no-tools model judge for semantic grounding of a proposed answer."""

    def __init__(self, model: BaseChatModel) -> None:
        self._model = model

    def validate(
        self,
        proposal: QuestionProposal,
        answer: str,
        cited_results: tuple[OracleToolResult, ...],
    ) -> SemanticEvidenceValidation:
        from langchain_core.messages import HumanMessage, SystemMessage

        response = self._model.invoke(
            [
                SystemMessage(
                    "You validate a private VQA oracle answer. Decide only whether the cited "
                    "structured local-tool evidence supports the answer to the frozen question. "
                    "Reject claims requiring measurements not present in the evidence; for example, "
                    "height is not supported by range or side alone. A cited measurement bucket must "
                    "exactly match the selected choice. Return strict JSON only: "
                    '{"accepted": true|false, "reason": "concise reason"}. Do not call tools.'
                ),
                HumanMessage(
                    json.dumps(
                        {
                            "question": proposal.question,
                            "answer": answer,
                            "answer_contract": asdict(proposal.answer_contract),
                            "cited_evidence": [asdict(result) for result in cited_results],
                        }
                    )
                ),
            ]
        )
        try:
            payload = _parse_strict_json_object(_response_text(response.content))
            accepted, reason = payload.get("accepted"), payload.get("reason")
            if not isinstance(accepted, bool) or not isinstance(reason, str) or not reason:
                raise ValueError(
                    "validator response requires boolean accepted and non-empty reason"
                )
        except (ValueError, json.JSONDecodeError, AttributeError) as exc:
            return SemanticEvidenceValidation(False, f"invalid_validator_response:{exc}")
        return SemanticEvidenceValidation(accepted, reason)


class PrivateToolCallingOracle:
    """Use direct local tools only, then validate a model's final JSON response."""

    def __init__(
        self,
        model: BaseChatModel,
        max_tool_calls: int = 8,
        semantic_validator: SemanticEvidenceValidator | None = None,
    ) -> None:
        if max_tool_calls < 1:
            raise ValueError("max_tool_calls must be positive")
        self._model = model
        self._max_tool_calls = max_tool_calls
        self._semantic_validator = semantic_validator

    def answer(
        self, proposal: QuestionProposal, registry: LocalOracleToolRegistry
    ) -> AcceptedOracleResult | RejectedOracleResult:
        from langchain_core.messages import HumanMessage, SystemMessage, ToolMessage

        tools = registry.tools()
        model = self._model.bind_tools(tools)
        messages: list[Any] = [
            SystemMessage(
                "You are a private VQA oracle. Use only supplied local tools. Do not invent "
                'evidence. Finish with JSON only: {"answer": value, "evidence_ids": [..]}.'
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
                    self._semantic_validator,
                )
            for call in tool_calls:
                if calls >= self._max_tool_calls:
                    break
                name = call.get("name")
                tool = next((item for item in tools if item.name == name), None)
                if tool is None:
                    return _rejected(proposal, "unsupported_tool", registry.results, trace)
                try:
                    output = tool.invoke(call.get("args", {}))
                except (TypeError, ValueError) as exc:
                    return _rejected(proposal, f"tool_error:{exc}", registry.results, trace)
                calls += 1
                trace.append(OracleTrace("tool", str(name)))
                messages.append(ToolMessage(content=str(output), tool_call_id=call["id"]))
        return _rejected(proposal, "tool_call_limit", registry.results, trace)


def create_openai_oracle(model: str, max_tool_calls: int = 8) -> PrivateToolCallingOracle:
    """Construct the private-only OpenAI tool-calling oracle."""
    from langchain_openai import ChatOpenAI

    return PrivateToolCallingOracle(
        ChatOpenAI(model=model),
        max_tool_calls=max_tool_calls,
        semantic_validator=OpenAISemanticEvidenceValidator(ChatOpenAI(model=model)),
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
) -> tuple[str, ResolvedAnswerContract]:
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
        if answer not in ("yes", "no"):
            raise ValueError("boolean answer must be yes or no")
        return str(answer), contract
    if isinstance(contract, ChoiceAnswerContract):
        if not isinstance(answer, str) or answer not in contract.choices:
            raise ValueError("choice answer is not allowed")
        cited = set(evidence_ids)
        measured_choices = {
            result.choice
            for result in results
            if result.choice is not None and any(item.id in cited for item in result.evidence)
        }
        if measured_choices and answer not in measured_choices:
            raise ValueError("choice answer does not match cited measurement bucket")
        return answer, contract
    if isinstance(contract, DeferredHeightChoiceContract):
        bucket_results = [
            result
            for result in results
            if result.tool == "bucket_measurement" and result.choice is not None and result.choices
        ]
        if len(bucket_results) != 1:
            raise ValueError("deferred height answer requires exactly one measurement bucket")
        bucket = bucket_results[0]
        if not any(item.id in evidence_ids for item in bucket.evidence):
            raise ValueError("deferred height answer must cite its measurement")
        if answer != bucket.choice:
            raise ValueError("deferred height answer does not match measurement bucket")
        if bucket.choice not in bucket.choices:
            raise ValueError("measurement bucket choice is not public")
        return bucket.choice, ChoiceAnswerContract(bucket.choices)
    raise ValueError("unsupported answer contract")


def _validated_result(
    proposal: QuestionProposal,
    response: str,
    results: tuple[OracleToolResult, ...],
    trace: list[OracleTrace],
    semantic_validator: SemanticEvidenceValidator | None,
) -> AcceptedOracleResult | RejectedOracleResult:
    try:
        payload = _parse_json_object(response)
        answer, answer_contract = _resolve_oracle_answer(
            proposal, payload.get("answer"), payload.get("evidence_ids"), results
        )
        evidence_ids = tuple(payload["evidence_ids"])
    except (ValueError, json.JSONDecodeError, AttributeError) as exc:
        return _rejected(proposal, f"invalid_final_answer:{exc}", results, trace)
    cited_results = _cited_results(evidence_ids, results)
    if semantic_validator is None:
        return _rejected(proposal, "semantic_validator_not_configured", results, trace)
    resolved_proposal = replace(proposal, answer_contract=answer_contract)
    verdict = semantic_validator.validate(resolved_proposal, answer, cited_results)
    trace.append(
        OracleTrace(
            "semantic_validation",
            f"{'accepted' if verdict.accepted else 'rejected'}:{verdict.reason}",
        )
    )
    if not verdict.accepted:
        return _rejected(proposal, f"unsupported_evidence:{verdict.reason}", results, trace)
    return AcceptedOracleResult(
        proposal, answer, answer_contract, evidence_ids, results, tuple(trace)
    )


def _cited_results(
    evidence_ids: tuple[str, ...], results: tuple[OracleToolResult, ...]
) -> tuple[OracleToolResult, ...]:
    cited = set(evidence_ids)
    return tuple(
        replace(
            result,
            evidence=tuple(evidence for evidence in result.evidence if evidence.id in cited),
        )
        for result in results
        if any(evidence.id in cited for evidence in result.evidence)
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
    if isinstance(contract, DeferredHeightChoiceContract):
        return (
            "deferred height choice: call measure_height, then bucket_measurement, and return "
            "the exact choice from that result"
        )
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


def _parse_strict_json_object(response: str) -> dict[str, Any]:
    payload: Any = json.loads(response)
    if not isinstance(payload, dict):
        raise ValueError("validator response must be an object")
    return payload


def _response_text(content: Any) -> str:
    if isinstance(content, str):
        return content
    if isinstance(content, list):
        return "".join(str(item.get("text", "")) for item in content if isinstance(item, dict))
    return str(content)
