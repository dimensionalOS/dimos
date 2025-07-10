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
from dimos.protocol.rpc.spec import RPC, RPCClient, RPCServer
from dimos.protocol.service.spec import Service

MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")

# (name, true_if_response_topic) -> TopicT
TopicGen = Callable[[str, bool], TopicT]
MsgGen = Callable[[str, list], MsgT]


class RPCInspectable(Protocol):
    @classmethod
    @property
    def rpcs() -> dict[str, Callable]: ...


class RPCReq(TypedDict):
    id: float | None
    name: str
    args: list


class RPCRes(TypedDict):
    id: float
    res: Any


class PubSubRPCMixin(RPC, Generic[TopicT]):
    @abstractmethod
    def _decodeRPCRes(self, msg: MsgT) -> RPCRes: ...

    @abstractmethod
    def _decodeRPCReq(self, msg: MsgT) -> RPCReq: ...

    @abstractmethod
    def _encodeRPCReq(self, res: RPCReq) -> MsgT: ...

    @abstractmethod
    def _encodeRPCRes(self, res: RPCRes) -> MsgT: ...

    def call(self, name: str, arguments: list, cb: Optional[Callable]):
        if cb is None:
            return self.call_nowait(name, arguments)

        return self.call_cb(name, arguments, cb)

    def call_cb(self, name: str, arguments: list, cb: Callable) -> Any:
        topic_req = self.topicgen(name, False)
        topic_res = self.topicgen(name, True)

        unsub = None
        msg_id = int(time.time())

        req = {"name": name, "args": arguments, "id": msg_id}

        def receive_response(msg: MsgT, _: TopicT):
            res = self._decodeRPCRes(msg)
            if res.get("id") != msg_id:
                return
            time.sleep(0.01)
            unsub()
            cb(res.get("res"))

        unsub = self.subscribe(topic_res, receive_response)

        self.publish(topic_req, self._encodeRPCReq(req))
        return unsub

    def call_nowait(self, name: str, arguments: list) -> None:
        topic_req = self.topicgen(name, False)
        req = {"name": name, "args": arguments, "id": None}
        self.publish(topic_req, self._encodeRPCReq(req))

    def serve_rpc(self, f: Callable, name: str = None):
        if not name:
            name = f.__name__

        topic_req = self.topicgen(name, False)
        topic_res = self.topicgen(name, True)

        def receive_call(msg: MsgT, _: TopicT) -> RPCRes:
            req = self._decodeRPCReq(msg)

            if req.get("name") != name:
                return
            response = f(*req.get("args"))

            self.publish(topic_res, self._encodeRPCRes({"id": req.get("id"), "res": response}))

        self.subscribe(topic_req, receive_call)


# simple PUBSUB RPC implementation that doesn't encode
# special request/response messages, assumes pubsub implementation
# supports generic dictionary pubsub
class PassThroughPubSubRPC(PubSubRPCMixin):
    def _encodeRPCReq(self, req: RPCReq) -> MsgT:
        return req

    def _decodeRPCRes(self, msg: MsgT) -> RPCRes:
        return msg

    def _encodeRPCRes(self, res: RPCRes) -> MsgT:
        return res

    def _decodeRPCReq(self, msg: MsgT) -> RPCReq:
        return msg
