# Copyright 2025 Dimensional Inc.
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

import pickle
import subprocess
import sys
import threading
import time
import traceback
from abc import abstractmethod
from dataclasses import dataclass
from types import FunctionType
from typing import (
    Any,
    Callable,
    Generic,
    Optional,
    Protocol,
    Sequence,
    TypedDict,
    TypeVar,
    runtime_checkable,
)

from dimos.protocol.pubsub.spec import PickleEncoderMixin, PubSub
from dimos.protocol.rpc.spec import Args, RPCClient, RPCInspectable, RPCServer, RPCSpec
from dimos.protocol.service.spec import Service

MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")

# (name, true_if_response_topic) -> TopicT
TopicGen = Callable[[str, bool], TopicT]
MsgGen = Callable[[str, list], MsgT]


class RPCReq(TypedDict):
    id: float | None
    name: str
    args: Args


class RPCRes(TypedDict):
    id: float
    res: Any


class PubSubRPCMixin(RPCSpec, PubSub[TopicT, MsgT], Generic[TopicT, MsgT]):
    @abstractmethod
    def topicgen(self, name: str, req_or_res: bool) -> TopicT: ...

    @abstractmethod
    def _decodeRPCRes(self, msg: MsgT) -> RPCRes: ...

    @abstractmethod
    def _decodeRPCReq(self, msg: MsgT) -> RPCReq: ...

    @abstractmethod
    def _encodeRPCReq(self, res: RPCReq) -> MsgT: ...

    @abstractmethod
    def _encodeRPCRes(self, res: RPCRes) -> MsgT: ...

    def call(self, name: str, arguments: Args, cb: Optional[Callable]):
        if cb is None:
            return self.call_nowait(name, arguments)

        return self.call_cb(name, arguments, cb)

    def call_cb(self, name: str, arguments: Args, cb: Callable) -> Any:
        topic_req = self.topicgen(name, False)
        topic_res = self.topicgen(name, True)
        msg_id = float(time.time())

        req: RPCReq = {"name": name, "args": arguments, "id": msg_id}

        def receive_response(msg: MsgT, _: TopicT):
            res = self._decodeRPCRes(msg)
            if res.get("id") != msg_id:
                return
            time.sleep(0.01)
            if unsub is not None:
                unsub()
            cb(res.get("res"))

        unsub = self.subscribe(topic_res, receive_response)

        self.publish(topic_req, self._encodeRPCReq(req))
        return unsub

    def call_nowait(self, name: str, arguments: Args) -> None:
        topic_req = self.topicgen(name, False)
        req: RPCReq = {"name": name, "args": arguments, "id": None}
        self.publish(topic_req, self._encodeRPCReq(req))

    def serve_rpc(self, f: FunctionType, name: Optional[str] = None):
        if not name:
            name = f.__name__

        topic_req = self.topicgen(name, False)
        topic_res = self.topicgen(name, True)

        def receive_call(msg: MsgT, _: TopicT) -> None:
            req = self._decodeRPCReq(msg)

            if req.get("name") != name:
                return
            args = req.get("args")
            if args is None:
                return
            response = f(*args[0], **args[1])

            req_id = req.get("id")
            if req_id is not None:
                self.publish(topic_res, self._encodeRPCRes({"id": req_id, "res": response}))

        return self.subscribe(topic_req, receive_call)


# simple PUBSUB RPC implementation that doesn't encode
# special request/response messages, assumes pubsub implementation
# supports generic dictionary pubsub
class PassThroughPubSubRPC(PubSubRPCMixin[TopicT, dict], Generic[TopicT]):
    def _encodeRPCReq(self, req: RPCReq) -> dict:
        return dict(req)

    def _decodeRPCRes(self, msg: dict) -> RPCRes:
        return msg  # type: ignore[return-value]

    def _encodeRPCRes(self, res: RPCRes) -> dict:
        return dict(res)

    def _decodeRPCReq(self, msg: dict) -> RPCReq:
        return msg  # type: ignore[return-value]
