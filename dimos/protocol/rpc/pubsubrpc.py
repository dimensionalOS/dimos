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
)

from dimos.constants import LCM_MAX_CHANNEL_NAME_LENGTH
from dimos.protocol.pubsub.lcmpubsub import PickleLCM, Topic
from dimos.protocol.pubsub.shmpubsub import PickleSharedMemory
from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.rpc.rpc_utils import deserialize_exception, serialize_exception
from dimos.protocol.rpc.spec import Args, RPCSpec
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


class RPCReq(TypedDict):
    id: float | None
    name: str
    args: Args


class RPCRes(TypedDict, total=False):
    id: float
    res: Any
    exception: dict[str, Any] | None  # Contains exception info: type, message, traceback


class PubSubRPCMixin(RPCSpec, PubSub[TopicT, MsgT], Generic[TopicT, MsgT]):
    def __init__(self, *args: Any, **kwargs: Any) -> None:
        super().__init__(*args, **kwargs)
        # Thread pool for RPC handler execution (prevents deadlock in nested calls)
        self._call_thread_pool: ThreadPoolExecutor | None = None
        self._call_thread_pool_lock = threading.RLock()
        self._call_thread_pool_max_workers = 50

        # Shared response subscriptions: one per RPC name instead of one per call
        # Maps str(topic_res) -> (subscription, {msg_id -> callback})
        self._response_subs: dict[str, tuple[Any, dict[float, Callable[..., Any]]]] = {}
        self._response_subs_lock = threading.RLock()

        # Message ID counter for unique IDs even with concurrent calls
        self._msg_id_counter = 0
        self._msg_id_lock = threading.Lock()

    def __getstate__(self) -> dict[str, Any]:
        state: dict[str, Any]
        if hasattr(super(), "__getstate__"):
            state = super().__getstate__()  # type: ignore[assignment]
        else:
            state = self.__dict__.copy()

        # Exclude unpicklable attributes when serializing.
        state.pop("_call_thread_pool", None)
        state.pop("_call_thread_pool_lock", None)
        state.pop("_response_subs", None)
        state.pop("_response_subs_lock", None)
        state.pop("_msg_id_lock", None)

        return state

    def __setstate__(self, state: dict[str, Any]) -> None:
        if hasattr(super(), "__setstate__"):
            super().__setstate__(state)  # type: ignore[misc]
        else:
            self.__dict__.update(state)

        # Restore unserializable attributes.
        self._call_thread_pool = None
        self._call_thread_pool_lock = threading.RLock()
        self._response_subs = {}
        self._response_subs_lock = threading.RLock()
        self._msg_id_lock = threading.Lock()

    @abstractmethod
    def topicgen(self, name: str, req_or_res: bool) -> TopicT: ...

    def _encodeRPCReq(self, req: RPCReq) -> dict[str, Any]:
        return dict(req)

    def _decodeRPCRes(self, msg: dict[Any, Any]) -> RPCRes:
        return msg  # type: ignore[return-value]

    def _encodeRPCRes(self, res: RPCRes) -> dict[str, Any]:
        return dict(res)

    def _decodeRPCReq(self, msg: dict[Any, Any]) -> RPCReq:
        return msg  # type: ignore[return-value]

    def _get_call_thread_pool(self) -> ThreadPoolExecutor:
        """Get or create the thread pool for RPC handler execution (lazy initialization)."""
        with self._call_thread_pool_lock:
            if self._call_thread_pool is None:
                self._call_thread_pool = ThreadPoolExecutor(
                    max_workers=self._call_thread_pool_max_workers
                )
            return self._call_thread_pool

    def _shutdown_thread_pool(self) -> None:
        """Safely shutdown the thread pool with deadlock prevention."""
        with self._call_thread_pool_lock:
            if self._call_thread_pool:
                # Check if we're being called from within the thread pool
                # to avoid "cannot join current thread" error
                current_thread = threading.current_thread()
                is_pool_thread = False

                # Check if current thread is one of the pool's threads
                if hasattr(self._call_thread_pool, "_threads"):
                    is_pool_thread = current_thread in self._call_thread_pool._threads
                elif "ThreadPoolExecutor" in current_thread.name:
                    # Fallback: check thread name pattern
                    is_pool_thread = True

                # Don't wait if we're in a pool thread to avoid deadlock
                self._call_thread_pool.shutdown(wait=not is_pool_thread)
                self._call_thread_pool = None

    def stop(self) -> None:
        """Stop the RPC service and cleanup thread pool.

        Subclasses that override this method should call super().stop()
        to ensure the thread pool is properly shutdown.
        """
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

        # Generate unique msg_id: timestamp + counter for concurrent calls
        with self._msg_id_lock:
            self._msg_id_counter += 1
            msg_id = time.time() + (self._msg_id_counter / 1_000_000)

        req: RPCReq = {"name": name, "args": arguments, "id": msg_id}

        # Get or create shared subscription for this RPC's response topic
        topic_res_key = str(topic_res)
        with self._response_subs_lock:
            if topic_res_key not in self._response_subs:
                # Create shared handler that routes to callbacks by msg_id
                callbacks_dict: dict[float, Callable[..., Any]] = {}

                def shared_response_handler(msg: MsgT, _: TopicT) -> None:
                    res = self._decodeRPCRes(msg)  # type: ignore[arg-type]
                    res_id = res.get("id")
                    if res_id is None:
                        return

                    # Look up callback for this msg_id
                    with self._response_subs_lock:
                        callback = callbacks_dict.pop(res_id, None)

                    if callback is None:
                        return  # No callback registered (already handled or timed out)

                    # Check if response contains an exception
                    exc_data = res.get("exception")
                    if exc_data:
                        # Reconstruct the exception and pass it to the callback
                        from typing import cast

                        from dimos.protocol.rpc.rpc_utils import SerializedException

                        exc = deserialize_exception(cast("SerializedException", exc_data))
                        callback(exc)
                    else:
                        # Normal response - pass the result
                        callback(res.get("res"))

                # Create single shared subscription
                unsub = self.subscribe(topic_res, shared_response_handler)
                self._response_subs[topic_res_key] = (unsub, callbacks_dict)

            # Register this call's callback
            _, callbacks_dict = self._response_subs[topic_res_key]
            callbacks_dict[msg_id] = cb

        # Publish request
        self.publish(topic_req, self._encodeRPCReq(req))  # type: ignore[arg-type]

        # Return unsubscribe function that removes this callback from the dict
        def unsubscribe_callback() -> None:
            with self._response_subs_lock:
                if topic_res_key in self._response_subs:
                    _, callbacks_dict = self._response_subs[topic_res_key]
                    callbacks_dict.pop(msg_id, None)

        return unsubscribe_callback

    def call_nowait(self, name: str, arguments: Args) -> None:
        topic_req = self.topicgen(name, False)
        req: RPCReq = {"name": name, "args": arguments, "id": None}
        self.publish(topic_req, self._encodeRPCReq(req))  # type: ignore[arg-type]

    def serve_rpc(self, f: FunctionType, name: str | None = None):  # type: ignore[no-untyped-def, override]
        if not name:
            name = f.__name__

        topic_req = self.topicgen(name, False)
        topic_res = self.topicgen(name, True)

        def receive_call(msg: MsgT, _: TopicT) -> None:
            req = self._decodeRPCReq(msg)  # type: ignore[arg-type]

            if req.get("name") != name:
                return

            args = req.get("args")
            if args is None:
                return

            # Execute RPC handler in a separate thread to avoid deadlock when
            # the handler makes nested RPC calls.
            def execute_and_respond() -> None:
                try:
                    response = f(*args[0], **args[1])
                    req_id = req.get("id")
                    if req_id is not None:
                        self.publish(topic_res, self._encodeRPCRes({"id": req_id, "res": response}))  # type: ignore[arg-type]

                except Exception as e:
                    logger.exception(f"Exception in RPC handler for {name}: {e}", exc_info=e)
                    # Send exception data to client if this was a request with an ID
                    req_id = req.get("id")
                    if req_id is not None:
                        exc_data = serialize_exception(e)
                        # Type ignore: SerializedException is compatible with dict[str, Any]
                        self.publish(
                            topic_res,
                            self._encodeRPCRes({"id": req_id, "exception": exc_data}),  # type: ignore[typeddict-item, arg-type]
                        )

            # Always use thread pool to execute RPC handlers (prevents deadlock)
            self._get_call_thread_pool().submit(execute_and_respond)

        return self.subscribe(topic_req, receive_call)


class LCMRPC(PubSubRPCMixin[Topic, Any], PickleLCM):
    def __init__(self, **kwargs: Any) -> None:
        # Need to ensure PickleLCM gets initialized properly
        # This is due to the diamond inheritance pattern with multiple base classes
        PickleLCM.__init__(self, **kwargs)
        # Initialize PubSubRPCMixin's thread pool
        PubSubRPCMixin.__init__(self, **kwargs)

    def topicgen(self, name: str, req_or_res: bool) -> Topic:
        suffix = "res" if req_or_res else "req"
        topic = f"/rpc/{name}/{suffix}"
        if len(topic) > LCM_MAX_CHANNEL_NAME_LENGTH:
            topic = f"/rpc/{short_id(name)}/{suffix}"
        return Topic(topic=topic)


class ShmRPC(PubSubRPCMixin[str, Any], PickleSharedMemory):
    def __init__(self, prefer: str = "cpu", **kwargs: Any) -> None:
        # Need to ensure SharedMemory gets initialized properly
        # This is due to the diamond inheritance pattern with multiple base classes
        PickleSharedMemory.__init__(self, prefer=prefer, **kwargs)
        # Initialize PubSubRPCMixin's thread pool
        PubSubRPCMixin.__init__(self, **kwargs)

    def topicgen(self, name: str, req_or_res: bool) -> str:
        suffix = "res" if req_or_res else "req"
        return f"/rpc/{name}/{suffix}"
