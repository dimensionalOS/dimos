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

"""Data-only topic wire v1; mirror of web/shared/topics.ts."""

from typing import Annotated, Any, Literal

from pydantic import AfterValidator, BaseModel, BeforeValidator, ConfigDict, Field, field_validator

TOPIC_VERSION = 1
MAX_TOPIC_KEY_BYTES = 1024
MAX_TOPIC_PAYLOAD_BYTES = 8 * 1024 * 1024
MAX_TOPIC_ROUTES = 256
MAX_TOPIC_KEYS = 128
MAX_TOPIC_PAGE = 32
TOPIC_ROUTE_PREFIX = "@t:"
TOPIC_PUBLISH_CHANNEL = "@put"
TOPIC_GUARD_CHANNEL = "@guard"
MAX_TOPIC_GUARD_BYTES = 8192
MAX_TOPIC_GUARDS = 32
TOPIC_PUBLISH_STREAM_MAGIC = b"DIMP"
TOPIC_GUARD_STREAM_MAGIC = b"DIMG"


def _key_bytes(value: str) -> str:
    if len(value.encode("utf-8")) > MAX_TOPIC_KEY_BYTES:
        raise ValueError("topic key exceeds byte bound")
    return value


def _integer(value: int | float) -> int | float:
    if int(value) != value:
        raise ValueError("expected integer")
    return value


def _version(value: Any) -> int:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or value != TOPIC_VERSION:
        raise ValueError("unsupported topic version")
    return TOPIC_VERSION


TopicSource = Annotated[str, Field(pattern=r"^[A-Za-z0-9][A-Za-z0-9_.-]{0,63}$")]
TopicId = Annotated[str, Field(min_length=1, max_length=64)]
TopicKey = Annotated[str, Field(min_length=1, max_length=1024), AfterValidator(_key_bytes)]
TopicPrefix = Annotated[str, Field(max_length=1024), AfterValidator(_key_bytes)]
TopicRoute = Annotated[str, Field(pattern=r"^@t:[A-Za-z0-9_-]{1,48}$")]
TopicGeneration = Annotated[int | float, Field(ge=0, le=9007199254740991), AfterValidator(_integer)]
TopicRate = Annotated[int | float, Field(gt=0, le=1000)]
TopicEncoding = Annotated[str, Field(max_length=256)]
TopicProfile = Annotated[str, Field(min_length=1, max_length=64)]
TopicCursor = Annotated[str, Field(max_length=128)]
TopicVersion = Annotated[Literal[1], BeforeValidator(_version)]
TopicGuardHandle = Annotated[str, Field(pattern=r"^@p:[A-Za-z0-9_-]{1,48}$")]
TopicGuardTimeout = Annotated[int | float, Field(ge=100, le=1000), AfterValidator(_integer)]
TopicTicket = Annotated[int | float, Field(ge=1, le=9007199254740991), AfterValidator(_integer)]


class _TopicModel(BaseModel):
    model_config = ConfigDict(strict=True, allow_inf_nan=False)


class TopicCapabilities(_TopicModel):
    read: bool
    write: bool
    profiles: list[TopicProfile] = Field(max_length=16)
    guardedPublishers: bool = False


class TopicSourceInfo(_TopicModel):
    source: TopicSource
    gen: TopicGeneration
    read: bool
    write: bool
    profiles: list[TopicProfile] = Field(max_length=16)
    guardedPublishers: bool = False


class TopicSources(_TopicModel):
    t: Literal["topic_sources"] = "topic_sources"
    v: TopicVersion = 1
    sources: list[TopicSourceInfo] = Field(max_length=64)


class TopicSub(_TopicModel):
    t: Literal["topic_sub"] = "topic_sub"
    id: TopicId
    source: TopicSource
    keyExpr: TopicKey
    delivery: Literal["latest", "reliable"]
    profile: TopicProfile
    maxHz: TopicRate


class TopicOpen(_TopicModel):
    t: Literal["topic_open"] = "topic_open"
    route: TopicRoute
    gen: TopicGeneration
    keyExpr: TopicKey
    delivery: Literal["latest", "reliable"]
    profile: TopicProfile
    maxHz: TopicRate


class TopicOpened(_TopicModel):
    t: Literal["topic_opened"] = "topic_opened"
    route: TopicRoute
    gen: TopicGeneration


class TopicReady(_TopicModel):
    t: Literal["topic_ready"] = "topic_ready"
    id: TopicId
    source: TopicSource
    route: TopicRoute
    gen: TopicGeneration


class TopicActivate(_TopicModel):
    t: Literal["topic_activate"] = "topic_activate"
    route: TopicRoute
    gen: TopicGeneration
    maxHz: TopicRate | None = None

    @field_validator("maxHz", mode="before")
    @classmethod
    def rate_is_absent_or_numeric(cls, value: Any) -> Any:
        if value is None:
            raise ValueError("maxHz must be numeric when supplied")
        return value


class TopicClose(_TopicModel):
    t: Literal["topic_close"] = "topic_close"
    route: TopicRoute
    gen: TopicGeneration


class TopicUnsub(_TopicModel):
    t: Literal["topic_unsub"] = "topic_unsub"
    id: TopicId


class TopicCredit(_TopicModel):
    t: Literal["topic_credit"] = "topic_credit"
    route: TopicRoute
    gen: TopicGeneration
    frames: TopicGeneration
    bytes: TopicGeneration


class TopicError(_TopicModel):
    t: Literal["topic_error"] = "topic_error"
    id: TopicId
    code: Annotated[str, Field(min_length=1, max_length=64)]
    message: TopicEncoding
    outcome: Literal["rejected", "unknown", "interrupted"]


class TopicList(_TopicModel):
    t: Literal["topic_list"] = "topic_list"
    id: TopicId
    source: TopicSource
    prefix: TopicPrefix
    cursor: TopicCursor
    limit: Annotated[int | float, Field(ge=1, le=MAX_TOPIC_PAGE), AfterValidator(_integer)]


class TopicDescriptor(_TopicModel):
    key: TopicKey
    encoding: TopicEncoding
    type: TopicEncoding
    availability: Literal["known", "observed", "advertised"]


class TopicPage(_TopicModel):
    t: Literal["topic_page"] = "topic_page"
    id: TopicId
    source: TopicSource
    gen: TopicGeneration
    entries: list[TopicDescriptor] = Field(max_length=MAX_TOPIC_PAGE)
    cursor: TopicCursor


class TopicPubResult(_TopicModel):
    t: Literal["topic_pub_result"] = "topic_pub_result"
    id: TopicId
    outcome: Literal["accepted", "rejected", "unknown"]
    code: Annotated[str, Field(max_length=64)]
    message: TopicEncoding
    providerTs: int | float


class TopicPublishMeta(_TopicModel):
    id: TopicId
    source: TopicSource
    gen: TopicGeneration
    key: TopicKey
    encoding: TopicEncoding


class TopicGuardOpenMeta(TopicPublishMeta):
    action: Literal["open"]
    group: TopicId
    timeoutMs: TopicGuardTimeout
    takeover: bool
    # Only the authenticated relay supplies these; browser values are replaced.
    owner: TopicId
    handle: TopicGuardHandle


class TopicGuardCommandMeta(_TopicModel):
    action: Literal["command"]
    source: TopicSource
    gen: TopicGeneration
    handle: TopicGuardHandle
    ticket: TopicTicket


class TopicGuardReady(_TopicModel):
    t: Literal["topic_guard_ready"] = "topic_guard_ready"
    id: TopicId
    handle: TopicGuardHandle
    gen: TopicGeneration
    ticket: TopicTicket
    timeoutMs: TopicGuardTimeout


class TopicGuardHeartbeat(_TopicModel):
    t: Literal["topic_guard_heartbeat"] = "topic_guard_heartbeat"
    handle: TopicGuardHandle
    gen: TopicGeneration
    ticket: TopicTicket


class TopicGuardTicket(_TopicModel):
    t: Literal["topic_guard_ticket"] = "topic_guard_ticket"
    handle: TopicGuardHandle
    gen: TopicGeneration
    ticket: TopicTicket


class TopicGuardStop(_TopicModel):
    t: Literal["topic_guard_stop"] = "topic_guard_stop"
    handle: TopicGuardHandle
    gen: TopicGeneration


class TopicGuardClosed(_TopicModel):
    t: Literal["topic_guard_closed"] = "topic_guard_closed"
    handle: TopicGuardHandle
    gen: TopicGeneration
    code: Annotated[str, Field(min_length=1, max_length=64)]


class TopicGuardResult(_TopicModel):
    t: Literal["topic_guard_result"] = "topic_guard_result"
    handle: TopicGuardHandle
    gen: TopicGeneration
    seq: TopicTicket
    outcome: Literal["accepted", "rejected", "unknown"]
    code: Annotated[str, Field(max_length=64)]
    providerTs: int | float


TopicMsg = (
    TopicSources
    | TopicSub
    | TopicOpen
    | TopicOpened
    | TopicReady
    | TopicActivate
    | TopicClose
    | TopicCredit
    | TopicUnsub
    | TopicError
    | TopicList
    | TopicPage
    | TopicPubResult
    | TopicGuardReady
    | TopicGuardHeartbeat
    | TopicGuardTicket
    | TopicGuardStop
    | TopicGuardClosed
    | TopicGuardResult
)
