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

"""Read-only IOPub observation for persistent code-policy kernels."""

from __future__ import annotations

import base64
from collections.abc import Iterable
from datetime import UTC, datetime
import hashlib
import json
import os
from pathlib import Path
import sys
import time
from typing import Annotated, Any, Literal, Protocol, TextIO

from jupyter_client.session import Session
import nbformat
from pydantic import BaseModel, ConfigDict, Field
import zmq

from dimos.agents.code_policy import (
    CodePolicyExecutionRecord,
    CodePolicyObserverDescriptor,
    CodePolicyObserverProbeReceipt,
    CodePolicyObserverState,
)
from dimos.constants import STATE_DIR

SupportedMessageType = Literal[
    "clear_output",
    "display_data",
    "error",
    "execute_input",
    "execute_result",
    "status",
    "stream",
]
_SUPPORTED_MESSAGE_TYPES = {
    "clear_output",
    "display_data",
    "error",
    "execute_input",
    "execute_result",
    "status",
    "stream",
}
DEFAULT_EVENT_LIMIT = 256_000
DEFAULT_EXECUTION_LIMIT = 2_000_000
DEFAULT_MIME_LIMIT = 1_000_000
DEFAULT_RECORDING_LIMIT = 64_000_000
EVENT_SCHEMA_VERSION = 1


class ObservedIOPubMessage(BaseModel):
    """Validated, credential-free projection of one supported IOPub message."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)

    observed_at: datetime
    message_type: SupportedMessageType
    parent_message_id: str
    parent_session_id: str
    execution_count: Annotated[int, Field(ge=0)] | None
    content: dict[str, Any]


class IOPubSubscriber:
    """A deliberately narrow Jupyter IOPub-only subscriber."""

    def __init__(
        self,
        descriptor: CodePolicyObserverDescriptor,
        *,
        context: Any | None = None,
    ) -> None:
        self.descriptor = descriptor
        self._owns_context = context is None
        self._context = context if context is not None else zmq.Context()
        self._socket = self._context.socket(zmq.SUB)
        self._socket.setsockopt(zmq.SUBSCRIBE, b"")
        self._socket.setsockopt(zmq.LINGER, 0)
        self._socket.connect(f"{descriptor.transport}://{descriptor.ip}:{descriptor.iopub_port}")
        self._session = Session(
            key=base64.b64decode(descriptor.key_base64),
            signature_scheme=descriptor.signature_scheme,
        )
        self._closed = False

    def receive(self, timeout_s: float = 0.1) -> ObservedIOPubMessage | None:
        """Receive one verified policy-client message, or ``None`` on timeout/filter."""
        if self._closed:
            return None
        timeout_ms = max(0, int(timeout_s * 1000))
        if not self._socket.poll(timeout_ms, zmq.POLLIN):
            return None
        frames = self._socket.recv_multipart()
        try:
            _identities, message_frames = self._session.feed_identities(frames)
            message = self._session.deserialize(message_frames, content=True, copy=True)
        except ValueError:
            return None
        message_type = message.get("header", {}).get("msg_type")
        parent = message.get("parent_header", {})
        if (
            message_type not in _SUPPORTED_MESSAGE_TYPES
            or parent.get("session") != self.descriptor.jupyter_client_session_id
            or not parent.get("msg_id")
        ):
            return None
        content = message.get("content", {})
        execution_count = content.get("execution_count")
        return ObservedIOPubMessage(
            observed_at=datetime.now(UTC),
            message_type=message_type,
            parent_message_id=parent["msg_id"],
            parent_session_id=parent["session"],
            execution_count=(execution_count if isinstance(execution_count, int) else None),
            content=dict(content),
        )

    def close(self) -> None:
        """Close only local observer resources; never signal the kernel."""
        if self._closed:
            return
        self._closed = True
        self._socket.close(linger=0)
        if self._owns_context:
            self._context.term()


class ObservationEvent(BaseModel):
    """Versioned append-only recording envelope."""

    model_config = ConfigDict(extra="forbid", frozen=True, strict=True)

    schema_version: Literal[1] = 1
    sequence: Annotated[int, Field(gt=0)]
    observed_at: datetime
    kind: Literal["iopub", "lifecycle", "record"]
    code_policy_session_id: str
    kernel_generation: Annotated[int, Field(ge=0)]
    parent_message_id: str | None = None
    message_type: SupportedMessageType | None = None
    execution_count: Annotated[int, Field(ge=0)] | None = None
    content: dict[str, Any]
    truncated: bool = False
    artifact_refs: tuple[str, ...] = ()


class ObserverHost(Protocol):
    def prepare_observer(self) -> CodePolicyObserverState: ...

    def get_observer_state(
        self, known_generation: int | None = None
    ) -> CodePolicyObserverState: ...

    def issue_observer_probe(self, kernel_generation: int) -> CodePolicyObserverProbeReceipt: ...

    def get_execution_records(
        self, session_id: str | None = None
    ) -> tuple[CodePolicyExecutionRecord, ...]: ...


class ObserverView(Protocol):
    """Presentation seam for one observer invocation."""

    def start(self, recording_path: Path) -> None: ...

    def render(self, event: ObservationEvent) -> None: ...

    def connected(self, descriptor: CodePolicyObserverDescriptor, *, initial: bool) -> None: ...

    def close(self) -> None: ...


class ObservationStore:
    """Private append-only JSONL store with atomic notebook materialization."""

    def __init__(
        self,
        output_parent: Path | None = None,
        *,
        event_limit: int = DEFAULT_EVENT_LIMIT,
        execution_limit: int = DEFAULT_EXECUTION_LIMIT,
        mime_limit: int = DEFAULT_MIME_LIMIT,
        recording_limit: int = DEFAULT_RECORDING_LIMIT,
    ) -> None:
        parent = output_parent or STATE_DIR / "code-policy-watch"
        parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        stamp = datetime.now(UTC).strftime("%Y%m%dT%H%M%S.%fZ")
        self.path = _reserve_directory(parent, f"observation-{stamp}")
        self.events_path = self.path / "iopub-events.jsonl"
        self.notebook_path = self.path / "code-policy-transcript.ipynb"
        self.blobs_path = self.path / "blobs"
        self.blobs_path.mkdir(mode=0o700)
        self.events_path.touch(mode=0o600, exist_ok=False)
        self._event_limit = max(1, event_limit)
        self._execution_limit = max(1, execution_limit)
        self._mime_limit = max(1, mime_limit)
        self._recording_limit = max(1, recording_limit)
        self._sequence = 0
        self._bytes_written = 0
        self._execution_bytes: dict[str, int] = {}

    def append_iopub(
        self,
        descriptor: CodePolicyObserverDescriptor,
        message: ObservedIOPubMessage,
    ) -> ObservationEvent:
        content, truncated, refs = self._bounded_content(
            message.content, execution_key=message.parent_message_id
        )
        return self._append(
            kind="iopub",
            descriptor=descriptor,
            parent_message_id=message.parent_message_id,
            message_type=message.message_type,
            execution_count=message.execution_count,
            content=content,
            truncated=truncated,
            artifact_refs=refs,
        )

    def append_lifecycle(
        self,
        descriptor: CodePolicyObserverDescriptor,
        event: str,
        **details: Any,
    ) -> ObservationEvent:
        return self._append(
            kind="lifecycle",
            descriptor=descriptor,
            content={"event": event, **details},
        )

    def append_record(
        self,
        descriptor: CodePolicyObserverDescriptor,
        record: CodePolicyExecutionRecord,
    ) -> ObservationEvent:
        content = record.model_dump(mode="json")
        bounded, truncated, refs = self._bounded_content(content)
        return self._append(
            kind="record",
            descriptor=descriptor,
            parent_message_id=record.jupyter_message_id,
            execution_count=record.jupyter_execution_count,
            content=bounded,
            truncated=truncated,
            artifact_refs=refs,
        )

    def finalize(self, *, complete: bool = True) -> Path:
        materialize_notebook(self.events_path, self.notebook_path, complete=complete)
        return self.notebook_path

    def _append(
        self,
        *,
        kind: Literal["iopub", "lifecycle", "record"],
        descriptor: CodePolicyObserverDescriptor,
        content: dict[str, Any],
        parent_message_id: str | None = None,
        message_type: SupportedMessageType | None = None,
        execution_count: int | None = None,
        truncated: bool = False,
        artifact_refs: tuple[str, ...] = (),
    ) -> ObservationEvent:
        self._sequence += 1
        event = ObservationEvent(
            sequence=self._sequence,
            observed_at=datetime.now(UTC),
            kind=kind,
            code_policy_session_id=descriptor.code_policy_session_id,
            kernel_generation=descriptor.kernel_generation,
            parent_message_id=parent_message_id,
            message_type=message_type,
            execution_count=execution_count,
            content=content,
            truncated=truncated,
            artifact_refs=artifact_refs,
        )
        line = event.model_dump_json() + "\n"
        encoded = line.encode()
        if self._bytes_written + len(encoded) > self._recording_limit:
            raise RuntimeError("code-policy observation recording limit reached")
        with self.events_path.open("ab") as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        self._bytes_written += len(encoded)
        return event

    def _bounded_content(
        self,
        content: dict[str, Any],
        *,
        execution_key: str | None = None,
    ) -> tuple[dict[str, Any], bool, tuple[str, ...]]:
        normalized = json.loads(json.dumps(content, default=str, sort_keys=True))
        refs: list[str] = []
        truncated = False
        data = normalized.get("data")
        if isinstance(data, dict):
            for mime_type, value in tuple(data.items()):
                payload = json.dumps(value, default=str, sort_keys=True).encode()
                if len(payload) <= self._mime_limit:
                    continue
                ref = self._store_blob(payload[: self._mime_limit], digest_source=payload)
                data[mime_type] = {
                    "truncation": (
                        f"MIME payload exceeded {self._mime_limit} bytes; "
                        "bounded prefix retained as artifact"
                    ),
                    "original_bytes": len(payload),
                    "artifact": ref,
                }
                refs.append(ref)
                truncated = True

        encoded = json.dumps(normalized, sort_keys=True).encode()
        if execution_key is not None:
            used = self._execution_bytes.get(execution_key, 0)
            remaining = max(0, self._execution_limit - used)
            self._execution_bytes[execution_key] = min(self._execution_limit, used + len(encoded))
            if len(encoded) > remaining:
                ref = self._store_blob(encoded[:remaining], digest_source=encoded)
                refs.append(ref)
                return (
                    {
                        "truncation": (
                            f"execution exceeded {self._execution_limit} bytes; "
                            "bounded prefix retained as artifact"
                        ),
                        "original_bytes": len(encoded),
                    },
                    True,
                    tuple(refs),
                )
        if len(encoded) <= self._event_limit:
            return normalized, truncated, tuple(refs)
        ref = self._store_blob(encoded[: self._event_limit], digest_source=encoded)
        refs.append(ref)
        return (
            {
                "truncation": (
                    f"content exceeded {self._event_limit} bytes; bounded prefix "
                    "is retained as an artifact"
                ),
                "original_bytes": len(encoded),
            },
            True,
            tuple(refs),
        )

    def _store_blob(self, retained: bytes, *, digest_source: bytes) -> str:
        digest = hashlib.sha256(digest_source).hexdigest()
        blob = self.blobs_path / f"{digest}.json"
        try:
            with blob.open("xb") as stream:
                stream.write(retained)
            blob.chmod(0o600)
        except FileExistsError:
            pass
        return str(blob.relative_to(self.path))


class TerminalRenderer:
    """Append-only notebook-style terminal renderer."""

    def __init__(self, stream: TextIO | None = None) -> None:
        self._stream = stream or sys.stdout

    def render(self, event: ObservationEvent) -> None:
        if event.kind == "lifecycle":
            self._write(f"\n--- {event.content.get('event', 'lifecycle')} ---\n")
            return
        if event.kind == "record":
            status = event.content.get("status", "completed")
            duration = event.content.get("monotonic_duration_s")
            suffix = f" in {duration:.2f}s" if isinstance(duration, float) else ""
            self._write(f"\n[{status}{suffix}]\n")
            return
        content = event.content
        if event.message_type == "execute_input":
            count = event.execution_count if event.execution_count is not None else "?"
            self._write(f"\nIn [{count}]:\n{content.get('code', '')}\n")
        elif event.message_type == "stream":
            self._write(str(content.get("text", "")))
        elif event.message_type in {"execute_result", "display_data"}:
            text = content.get("data", {}).get("text/plain")
            self._write(f"{text}\n" if text is not None else "[rich output retained]\n")
        elif event.message_type == "error":
            traceback = content.get("traceback", [])
            self._write("\n".join(str(line) for line in traceback) + "\n")
        if event.truncated:
            self._write("[observation content truncated]\n")

    def _write(self, value: str) -> None:
        self._stream.write(value)
        self._stream.flush()


class TerminalObserverView:
    """Default notebook-style terminal presentation."""

    def __init__(self, stream: TextIO | None = None) -> None:
        self._stream = stream or sys.stdout
        self._renderer = TerminalRenderer(self._stream)

    def start(self, recording_path: Path) -> None:
        print(f"recording: {recording_path.resolve()}", file=self._stream, flush=True)
        print(
            "warning: recording may contain code, environment, perception, map, and robot data",
            file=self._stream,
            flush=True,
        )

    def render(self, event: ObservationEvent) -> None:
        self._renderer.render(event)

    def connected(self, descriptor: CodePolicyObserverDescriptor, *, initial: bool) -> None:
        print(
            f"session={descriptor.code_policy_session_id} "
            f"client={descriptor.jupyter_client_session_id} "
            f"generation={descriptor.kernel_generation}",
            file=self._stream,
            flush=True,
        )
        if initial:
            print("ready", file=self._stream, flush=True)

    def close(self) -> None:
        return None


def select_code_policy_module(candidates: Iterable[str], requested: str | None = None) -> str:
    """Select exactly one compatible module name, failing closed."""
    compatible = sorted(
        name
        for name in set(candidates)
        if name == "CodePolicyModule" or name.endswith(".CodePolicyModule")
    )
    if requested is not None:
        if requested not in compatible:
            raise LookupError(
                f"requested code-policy module {requested!r} is not available; "
                f"candidates: {compatible or 'none'}"
            )
        return requested
    if len(compatible) != 1:
        raise LookupError(
            f"expected exactly one CodePolicyModule; candidates: {compatible or 'none'}"
        )
    return compatible[0]


def watch_code_policy(
    host: ObserverHost,
    *,
    output_parent: Path | None = None,
    startup_timeout_s: float = 10.0,
    poll_interval_s: float = 0.25,
    stream: TextIO | None = None,
    view: ObserverView | None = None,
) -> Path:
    """Attach, verify readiness, then observe until interrupted or module stop."""
    store = ObservationStore(output_parent)
    observer_view = view or TerminalObserverView(stream)
    observer_view.start(store.path)
    subscriber: IOPubSubscriber | None = None
    complete = False
    reconciled: set[str] = set()
    try:
        state = CodePolicyObserverState.model_validate(host.prepare_observer())
        descriptor = _ready_descriptor(state)
        subscriber = IOPubSubscriber(descriptor)
        observer_view.render(store.append_lifecycle(descriptor, "attaching"))
        _await_probe(
            host,
            subscriber,
            descriptor,
            startup_timeout_s=startup_timeout_s,
        )
        observer_view.connected(descriptor, initial=True)
        observer_view.render(store.append_lifecycle(descriptor, "ready"))
        last_poll = time.monotonic()
        while True:
            message = subscriber.receive(timeout_s=min(poll_interval_s, 0.1))
            if message is not None:
                event = store.append_iopub(descriptor, message)
                observer_view.render(event)
                if (
                    message.message_type == "status"
                    and message.content.get("execution_state") == "idle"
                ):
                    _reconcile(host, descriptor, store, observer_view, reconciled)
            if time.monotonic() - last_poll < poll_interval_s:
                continue
            last_poll = time.monotonic()
            state = CodePolicyObserverState.model_validate(
                host.get_observer_state(descriptor.kernel_generation)
            )
            _reconcile(host, descriptor, store, observer_view, reconciled)
            if state.availability == "stopped":
                observer_view.render(store.append_lifecycle(descriptor, "module-stopped"))
                complete = True
                break
            if state.availability == "unavailable":
                observer_view.render(
                    store.append_lifecycle(
                        descriptor,
                        "observation-gap",
                        reason="kernel-unavailable",
                        next_session=state.code_policy_session_id,
                    )
                )
                state = CodePolicyObserverState.model_validate(host.prepare_observer())
            if state.availability in {"replaced", "ready"} and state.descriptor is not None:
                replacement = state.descriptor
                if replacement.kernel_generation != descriptor.kernel_generation:
                    observer_view.render(
                        store.append_lifecycle(
                            descriptor,
                            "observation-gap",
                            next_generation=replacement.kernel_generation,
                        )
                    )
                    subscriber.close()
                    descriptor = replacement
                    subscriber = IOPubSubscriber(descriptor)
                    _await_probe(
                        host,
                        subscriber,
                        descriptor,
                        startup_timeout_s=startup_timeout_s,
                    )
                    observer_view.connected(descriptor, initial=False)
                    observer_view.render(store.append_lifecycle(descriptor, "reconnected"))
            else:
                observer_view.render(store.append_lifecycle(descriptor, "observer-disconnected"))
                break
    except KeyboardInterrupt:
        if "descriptor" in locals():
            observer_view.render(store.append_lifecycle(descriptor, "detached-by-operator"))
        complete = True
    finally:
        if subscriber is not None:
            subscriber.close()
        try:
            store.finalize(complete=complete)
        finally:
            observer_view.close()
    return store.path


def materialize_notebook(events_path: Path, notebook_path: Path, *, complete: bool) -> None:
    """Recover an ordered notebook from all valid JSONL prefixes."""
    events = _read_valid_events(events_path)
    cells: list[Any] = []
    code_cells: dict[str, Any] = {}
    pending_clear: set[str] = set()
    for event in events:
        if event.kind == "lifecycle":
            cells.append(
                nbformat.v4.new_markdown_cell(  # type: ignore[no-untyped-call]
                    f"**Observer lifecycle:** {event.content.get('event', 'unknown')}"
                )
            )
            continue
        parent = event.parent_message_id
        if event.kind == "record":
            if parent in code_cells:
                code_cells[parent].metadata["dimos_execution_record"] = event.content
            continue
        if event.message_type == "execute_input" and parent is not None:
            source = event.content.get("code")
            if source is None and event.truncated:
                source = "# [source truncated; see observer artifact references]"
            cell = nbformat.v4.new_code_cell(  # type: ignore[no-untyped-call]
                source=str(source or ""),
                execution_count=event.execution_count,
            )
            cells.append(cell)
            code_cells[parent] = cell
            _append_truncation_cell(cells, event, events_path.parent)
            continue
        cell = code_cells.get(parent or "")
        if cell is None:
            continue
        if event.message_type == "clear_output":
            if event.content.get("wait"):
                pending_clear.add(parent or "")
            else:
                cell.outputs = []
                pending_clear.discard(parent or "")
            continue
        output = _notebook_output(event)
        if output is not None:
            if parent in pending_clear:
                cell.outputs = []
                pending_clear.remove(parent)
            cell.outputs.append(output)
        _append_truncation_cell(cells, event, events_path.parent)
    if not complete:
        cells.append(
            nbformat.v4.new_markdown_cell(  # type: ignore[no-untyped-call]
                "**Incomplete observation:** recording ended without a clean detach."
            )
        )
    notebook = nbformat.v4.new_notebook(  # type: ignore[no-untyped-call]
        cells=cells,
        metadata={"dimos": {"observer_schema_version": EVENT_SCHEMA_VERSION}},
    )
    temporary = notebook_path.with_suffix(".ipynb.tmp")
    nbformat.write(notebook, temporary)  # type: ignore[no-untyped-call]
    temporary.chmod(0o600)
    os.replace(temporary, notebook_path)


def _notebook_output(event: ObservationEvent) -> Any | None:
    content = event.content
    if event.message_type == "stream":
        return nbformat.v4.new_output(  # type: ignore[no-untyped-call]
            "stream",
            name=content.get("name", "stdout"),
            text=str(content.get("text", "")),
        )
    if event.message_type == "error":
        return nbformat.v4.new_output(  # type: ignore[no-untyped-call]
            "error",
            ename=str(content.get("ename", "Error")),
            evalue=str(content.get("evalue", "")),
            traceback=[str(line) for line in content.get("traceback", [])],
        )
    if event.message_type in {"execute_result", "display_data"}:
        output_type = event.message_type
        kwargs: dict[str, Any] = {
            "data": content.get("data", {}),
            "metadata": content.get("metadata", {}),
        }
        if output_type == "execute_result":
            kwargs["execution_count"] = event.execution_count
        return nbformat.v4.new_output(  # type: ignore[no-untyped-call]
            output_type, **kwargs
        )
    return None


def _append_truncation_cell(
    cells: list[Any], event: ObservationEvent, recording_path: Path
) -> None:
    if not event.truncated:
        return
    missing = [
        reference for reference in event.artifact_refs if not (recording_path / reference).is_file()
    ]
    detail = f" Artifact references: {', '.join(event.artifact_refs) or 'none'}."
    if missing:
        detail += f" Missing artifacts: {', '.join(missing)}."
    cells.append(
        nbformat.v4.new_markdown_cell(  # type: ignore[no-untyped-call]
            f"**Truncated observation.**{detail}"
        )
    )


def _read_valid_events(path: Path) -> list[ObservationEvent]:
    events: list[ObservationEvent] = []
    with path.open() as stream:
        for line in stream:
            try:
                events.append(ObservationEvent.model_validate_json(line))
            except ValueError:
                break
    return events


def _reserve_directory(parent: Path, stem: str) -> Path:
    for suffix in range(10_000):
        candidate = parent / (stem if suffix == 0 else f"{stem}-{suffix}")
        try:
            candidate.mkdir(mode=0o700)
        except FileExistsError:
            continue
        return candidate
    raise RuntimeError(f"could not reserve observation directory under {parent}")


def _ready_descriptor(state: CodePolicyObserverState) -> CodePolicyObserverDescriptor:
    if state.availability != "ready" or state.descriptor is None:
        raise RuntimeError(f"code-policy observation is {state.availability}")
    return state.descriptor


def _await_probe(
    host: ObserverHost,
    subscriber: IOPubSubscriber,
    descriptor: CodePolicyObserverDescriptor,
    *,
    startup_timeout_s: float,
) -> None:
    deadline = time.monotonic() + startup_timeout_s
    next_probe = 0.0
    probe_ids: set[str] = set()
    while time.monotonic() < deadline:
        now = time.monotonic()
        if now >= next_probe:
            receipt = CodePolicyObserverProbeReceipt.model_validate(
                host.issue_observer_probe(descriptor.kernel_generation)
            )
            probe_ids.add(receipt.message_id)
            next_probe = now + 0.25
        message = subscriber.receive(timeout_s=min(0.1, max(0.0, deadline - now)))
        if (
            message is not None
            and message.parent_message_id in probe_ids
            and message.message_type == "status"
            and message.content.get("execution_state") == "idle"
        ):
            return
    raise TimeoutError("timed out waiting for verified code-policy IOPub readiness")


def _reconcile(
    host: ObserverHost,
    descriptor: CodePolicyObserverDescriptor,
    store: ObservationStore,
    view: ObserverView,
    reconciled: set[str],
) -> None:
    records = host.get_execution_records(descriptor.code_policy_session_id)
    for value in records:
        record = CodePolicyExecutionRecord.model_validate(value)
        message_id = record.jupyter_message_id
        if message_id is None or message_id in reconciled:
            continue
        event = store.append_record(descriptor, record)
        view.render(event)
        reconciled.add(message_id)
