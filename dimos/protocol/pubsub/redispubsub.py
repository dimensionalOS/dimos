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

from collections import defaultdict
from collections.abc import Callable
from dataclasses import dataclass, field
import json
import threading
import time
from types import TracebackType
from typing import Any

import redis  # type: ignore[import-not-found]

from dimos.protocol.pubsub.spec import PubSub
from dimos.protocol.service.spec import Service


@dataclass
class RedisConfig:
    host: str = "localhost"
    port: int = 6379
    db: int = 0
    kwargs: dict[str, Any] = field(default_factory=dict)


class Redis(PubSub[str, Any], Service[RedisConfig]):
    """Redis-based pub/sub implementation."""

    default_config = RedisConfig

    def __init__(self, **kwargs) -> None:  # type: ignore[no-untyped-def]
        super().__init__(**kwargs)

        # Redis connections
        self._client = None
        self._pubsub = None

        # Subscription management
        self._callbacks: dict[str, list[Callable[[Any, str], None]]] = defaultdict(list)
        self._listener_thread = None
        self._running = False

    def start(self) -> None:
        """Start the Redis pub/sub service."""
        if self._running:
            return
        self._connect()  # type: ignore[no-untyped-call]

    def stop(self) -> None:
        """Stop the Redis pub/sub service."""
        self.close()

    def _connect(self):  # type: ignore[no-untyped-def]
        """Connect to Redis and set up pub/sub."""
        try:
            self._client = redis.Redis(
                host=self.config.host,
                port=self.config.port,
                db=self.config.db,
                decode_responses=True,
                **self.config.kwargs,
            )
            # Test connection
            self._client.ping()  # type: ignore[attr-defined]

            self._pubsub = self._client.pubsub()  # type: ignore[attr-defined]
            self._running = True

            # Start listener thread
            self._listener_thread = threading.Thread(target=self._listen_loop, daemon=True)  # type: ignore[assignment]
            self._listener_thread.start()  # type: ignore[attr-defined]

        except Exception as e:
            raise ConnectionError(
                f"Failed to connect to Redis at {self.config.host}:{self.config.port}: {e}"
            )

    def _listen_loop(self) -> None:
        """Listen for messages from Redis and dispatch to callbacks."""
        while self._running:
            try:
                if not self._pubsub:
                    break
                message = self._pubsub.get_message(timeout=0.1)
                if message and message["type"] == "message":
                    topic = message["channel"]
                    data = message["data"]

                    # Try to deserialize JSON, fall back to raw data
                    try:
                        data = json.loads(data)
                    except (json.JSONDecodeError, TypeError):
                        pass

                    # Call all callbacks for this topic
                    for callback in self._callbacks.get(topic, []):
                        try:
                            callback(data, topic)
                        except Exception as e:
                            # Log error but continue processing other callbacks
                            print(f"Error in callback for topic {topic}: {e}")

            except Exception as e:
                if self._running:  # Only log if we're still supposed to be running
                    print(f"Error in Redis listener loop: {e}")
                time.sleep(0.1)  # Brief pause before retrying

    def publish(self, topic: str, message: Any) -> None:
        """Publish a message to a topic."""
        if not self._client:
            raise RuntimeError("Redis client not connected")

        # Serialize message as JSON if it's not a string
        if isinstance(message, str):
            data = message
        else:
            data = json.dumps(message)

        self._client.publish(topic, data)

    def subscribe(self, topic: str, callback: Callable[[Any, str], None]) -> Callable[[], None]:
        """Subscribe to a topic with a callback."""
        if not self._pubsub:
            raise RuntimeError("Redis pubsub not initialized")

        # If this is the first callback for this topic, subscribe to Redis channel
        if topic not in self._callbacks or not self._callbacks[topic]:
            self._pubsub.subscribe(topic)

        # Add callback to our list
        self._callbacks[topic].append(callback)

        # Return unsubscribe function
        def unsubscribe() -> None:
            self.unsubscribe(topic, callback)

        return unsubscribe

    def unsubscribe(self, topic: str, callback: Callable[[Any, str], None]) -> None:
        """Unsubscribe a callback from a topic."""
        if topic in self._callbacks:
            try:
                self._callbacks[topic].remove(callback)

                # If no more callbacks for this topic, unsubscribe from Redis channel
                if not self._callbacks[topic]:
                    if self._pubsub:
                        self._pubsub.unsubscribe(topic)
                    del self._callbacks[topic]

            except ValueError:
                pass  # Callback wasn't in the list

    def close(self) -> None:
        """Close Redis connections and stop listener thread."""
        self._running = False

        if self._listener_thread and self._listener_thread.is_alive():
            self._listener_thread.join(timeout=1.0)

        if self._pubsub:
            try:
                self._pubsub.close()
            except Exception:
                pass
            self._pubsub = None

        if self._client:
            try:
                self._client.close()
            except Exception:
                pass
            self._client = None

        self._callbacks.clear()

    def __enter__(self):  # type: ignore[no-untyped-def]
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc_val: BaseException | None,
        exc_tb: TracebackType | None,
    ) -> None:
        self.close()
