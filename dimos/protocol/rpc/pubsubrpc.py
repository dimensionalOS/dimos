# Copyright 2025-2026 Dimensional Inc.
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

from abc import abstractmethod
from collections.abc import Callable
from concurrent.futures import ThreadPoolExecutor
import threading
import time
from typing import (
    TYPE_CHECKING,
    Any,
    Generic,
    TypedDict,
    TypeVar,
    cast,
)

from dimos.constants import LCM_MAX_CHANNEL_NAME_LENGTH
from dimos.protocol.encode.cbor import decode, encode
from dimos.protocol.pubsub.impl.lcmpubsub import CborLCM, Topic
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception
from dimos.protocol.rpc.spec import DEFAULT_RPC_TIMEOUT, DEFAULT_RPC_TIMEOUTS, Args, RPCSpec
from dimos.protocol.service.call_pool import CallPool
from dimos.protocol.service.lcmservice import LCMService
from dimos.utils.generic import short_id
from dimos.utils.logging_config import setup_logger

if TYPE_CHECKING:
    from types import FunctionType

logger = setup_logger()

MsgT = TypeVar("MsgT")
TopicT = TypeVar("TopicT")

# (name, true_if_response_topic) -> TopicT
TopicGen = Callable[[str, bool], TopicT]
MsgGen = Callable[[str, list], MsgT]  # type: ignore[type-arg]


# Bump when the envelope below changes. A peer on another version is logged and
# ignored instead of being misparsed.
RPC_WIRE_VERSION = 1


class RPCReq(TypedDict):
    v: int
    id: float | None
    name: str
    # encode((args, kwargs)). The envelope holds only primitives, so it always decodes;
    # the handler decodes the body and answers a bad one with the error instead of the
    # pubsub dropping the whole request.
    args: bytes


class RPCRes(TypedDict, total=False):
    v: int
    id: float
    res: bytes  # encode(result)
    exception: bytes  # encode(serialize_exception(e))


def _decode_reply(res: RPCRes) -> Any:
    """The result or the remote exception, or the decode failure itself so the caller
    raises instead of timing out."""
    try:
        exc_data = res.get("exception")
        if exc_data is not None:
            return deserialize_exception(decode(exc_data))
        return decode(res["res"])
    except Exception as e:
        return e


class PubSubRPCMixin(RPCSpec, PubSub[TopicT, MsgT], Generic[TopicT, MsgT]):
    def __init__(
        self,
        *args: Any,
        rpc_timeouts: dict[str, float] | None = None,
        default_rpc_timeout: float = DEFAULT_RPC_TIMEOUT,
        **kwargs: Any,
    ) -> None:
        if rpc_timeouts is None:
            rpc_timeouts = dict(DEFAULT_RPC_TIMEOUTS)
        super().__init__(*args, **kwargs)
        self.rpc_timeouts = dict(rpc_timeouts)
        self.default_rpc_timeout = default_rpc_timeout
        # Thread pool for RPC handler execution (prevents deadlock in nested calls)
        self._call_pool = CallPool(max_workers=50)

        # Shared response subscriptions: one per RPC name instead of one per call
        # Maps str(topic_res) -> (subscription, {msg_id -> callback})
        self._response_subs: dict[str, tuple[Any, dict[float, Callable[..., Any]]]] = {}
        self._response_subs_lock = threading.RLock()

        # Message ID counter for unique IDs even with concurrent calls
        self._msg_id_counter = 0
        self._msg_id_lock = threading.Lock()

    @abstractmethod
    def topicgen(self, name: str, req_or_res: bool) -> TopicT: ...

    def _wire_version_ok(self, msg: Any, topic: TopicT) -> bool:
        version = msg.get("v") if isinstance(msg, dict) else None
        if version == RPC_WIRE_VERSION:
            return True
        logger.error(
            "Ignoring RPC message from another wire version: is another dimos version on the bus?",
            topic=str(topic),
            version=version,
            expected=RPC_WIRE_VERSION,
        )
        return False

    def _get_call_thread_pool(self) -> ThreadPoolExecutor:
        return self._call_pool.get()

    def _shutdown_thread_pool(self) -> None:
        self._call_pool.shutdown()

    def start(self) -> None:
        self._call_pool.start()
        super().start()

    def stop(self) -> None:
        """Stop the RPC service and cleanup thread pool.

        Subclasses that override this method should call super().stop()
        to ensure the thread pool is properly shutdown.
        """
        with self._call_pool.stop_scope():
            self._shutdown_thread_pool()

            # Cleanup shared response subscriptions
            with self._response_subs_lock:
                for unsub, _ in self._response_subs.values():
                    unsub()
                self._response_subs.clear()

            # Call parent stop if it exists
            if hasattr(super(), "stop"):
                super().stop()  # type: ignore[misc]

    def call(self, name: str, arguments: Args, cb: Callable | None):  # type: ignore[no-untyped-def, type-arg]
        if cb is None:
            return self.call_nowait(name, arguments)

        return self.call_cb(name, arguments, cb)

    def call_cb(self, name: str, arguments: Args, cb: Callable[..., Any]) -> Any:
        topic_req = self.topicgen(name, False)
        topic_res = self.topicgen(name, True)

        # Encode before registering the callback: an unencodable argument raises to
        # the caller and leaves nothing pending.
        body = encode(arguments)

        # Generate unique msg_id: timestamp + counter for concurrent calls
        with self._msg_id_lock:
            self._msg_id_counter += 1
            msg_id = time.time() + (self._msg_id_counter / 1_000_000)

        req: RPCReq = {"v": RPC_WIRE_VERSION, "name": name, "args": body, "id": msg_id}

        # Get or create shared subscription for this RPC's response topic
        topic_res_key = str(topic_res)
        with self._response_subs_lock:
            if topic_res_key not in self._response_subs:
                # Create shared handler that routes to callbacks by msg_id
                callbacks_dict: dict[float, Callable[..., Any]] = {}

                def shared_response_handler(msg: MsgT, topic: TopicT) -> None:
                    if not self._wire_version_ok(msg, topic):
                        return
                    res = cast("RPCRes", msg)
                    res_id = res.get("id")
                    if res_id is None:
                        return

                    # Look up callback for this msg_id
                    with self._response_subs_lock:
                        callback = callbacks_dict.pop(res_id, None)

                    if callback is None:
                        return  # No callback registered (already handled or timed out)

                    callback(_decode_reply(res))

                # Create single shared subscription
                unsub = self.subscribe(topic_res, shared_response_handler)
                self._response_subs[topic_res_key] = (unsub, callbacks_dict)

            # Register this call's callback
            _, callbacks_dict = self._response_subs[topic_res_key]
            callbacks_dict[msg_id] = cb

        # Unsubscribe function that removes this callback from the dict
        def unsubscribe_callback() -> None:
            with self._response_subs_lock:
                if topic_res_key in self._response_subs:
                    _, callbacks_dict = self._response_subs[topic_res_key]
                    callbacks_dict.pop(msg_id, None)

        try:
            self.publish(topic_req, cast("MsgT", req))
        except Exception:
            unsubscribe_callback()
            raise

        return unsubscribe_callback

    def call_nowait(self, name: str, arguments: Args) -> None:
        topic_req = self.topicgen(name, False)
        req: RPCReq = {"v": RPC_WIRE_VERSION, "name": name, "args": encode(arguments), "id": None}
        self.publish(topic_req, cast("MsgT", req))

    def serve_rpc(self, f: FunctionType, name: str | None = None):  # type: ignore[no-untyped-def, override]
        if not name:
            name = f.__name__

        topic_req = self.topicgen(name, False)
        topic_res = self.topicgen(name, True)

        def receive_call(msg: MsgT, topic: TopicT) -> None:
            if not self._wire_version_ok(msg, topic):
                return
            req = cast("RPCReq", msg)

            if req.get("name") != name:
                return

            body = req.get("args")
            if body is None:
                return
            req_id = req.get("id")

            # Execute RPC handler in a separate thread to avoid deadlock when
            # the handler makes nested RPC calls. The body is decoded here so that a
            # payload this process cannot decode is answered with the error.
            def execute_and_respond() -> None:
                try:
                    args = decode(body)
                    response = f(*args[0], **args[1])
                    if req_id is not None:
                        res: RPCRes = {"v": RPC_WIRE_VERSION, "id": req_id, "res": encode(response)}
                        self.publish(topic_res, cast("MsgT", res))
                except Exception as e:
                    logger.exception(f"Exception in RPC handler for {name}: {e}", exc_info=e)
                    if req_id is not None:
                        failure: RPCRes = {
                            "v": RPC_WIRE_VERSION,
                            "id": req_id,
                            "exception": encode(serialize_exception(e)),
                        }
                        self.publish(topic_res, cast("MsgT", failure))

            # Always use thread pool to execute RPC handlers (prevents deadlock)
            self._get_call_thread_pool().submit(execute_and_respond)

        return self.subscribe(topic_req, receive_call)


class LCMRPC(PubSubRPCMixin[Topic, Any], CborLCM):
    def __init__(
        self,
        rpc_timeouts: dict[str, float] | None = None,
        default_rpc_timeout: float = DEFAULT_RPC_TIMEOUT,
        **kwargs: Any,
    ) -> None:
        if rpc_timeouts is None:
            rpc_timeouts = dict(DEFAULT_RPC_TIMEOUTS)
        CborLCM.__init__(self, **kwargs)
        PubSubRPCMixin.__init__(
            self, rpc_timeouts=rpc_timeouts, default_rpc_timeout=default_rpc_timeout, **kwargs
        )

    def start(self) -> None:
        # LCM must finish stopping dispatch before the pool can accept calls.
        LCMService.start(self)

    def _shutdown_thread_pool(self) -> None:
        LCMService._shutdown_thread_pool(self)

    def topicgen(self, name: str, req_or_res: bool) -> Topic:
        suffix = "res" if req_or_res else "req"
        topic = f"/rpc/{name}/{suffix}"
        if len(topic) > LCM_MAX_CHANNEL_NAME_LENGTH:
            topic = f"/rpc/{short_id(name)}/{suffix}"
        return Topic(topic=topic)
