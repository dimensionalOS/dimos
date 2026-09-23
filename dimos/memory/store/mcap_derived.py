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

"""Streams dimos WRITES into an mcap: the envelope they use and their vector index.

A recorded channel carries one ROS message and nothing else -- no pose, no tags,
no embedding -- because that is all a recorder had. A stream built FROM the
recording has all three, and they have to go somewhere. Here they go in the same
message, on a channel of dimos's own, in the recording itself; a companion
database beside the mcap was the alternative and it goes stale the moment either
half is moved or copied on its own.

The envelope is deliberately dull, so a reader that is not dimos can still get at
the payload knowing only this docstring::

    u32  header_length
    ...  header, UTF-8 JSON: {"pose": [x,y,z,qx,qy,qz,qw], "tags": {...}, "dim": n}
    u32  payload_length
    ...  payload, as the stream's own codec encodes it
    ...  the embedding vector, float32, ``dim`` of them, when the header has a dim

Absent keys mean absent things: no ``pose`` key is no pose, no ``dim`` key is no
embedding. The channel's ``message_encoding`` is ``dimos/observation`` and its
schema names the stream's payload type, so ``codec_id`` in the registry is what
says how to read the payload bytes.
"""

from __future__ import annotations

from collections.abc import Iterator
import json
import struct
from typing import TYPE_CHECKING, Any

import numpy as np

from dimos.memory.store.mcap_append import ChannelSpec
from dimos.memory.vectorstore.base import VectorStore, VectorStoreConfig

if TYPE_CHECKING:
    from dimos.memory.store.mcap_append import McapAppender
    from dimos.memory.type.observation import Observation
    from dimos.models.embedding.base import Embedding

ENVELOPE_ENCODING = "dimos/observation"
SCHEMA_ENCODING = "dimos/observation-envelope"

# One Metadata record holds the whole registry, rewritten whenever a stream is
# added. mcap does not deduplicate metadata records, so the last one with this
# name is the live one and every reader here takes it that way.
REGISTRY_METADATA = "dimos.streams"


def topic_for(name: str) -> str:
    """The channel a dimos stream is written to.

    A leading slash and the name as it stands, so that ``_slug`` -- which drops
    the slash and turns the rest into underscores -- reads the same name back.
    An underscore in the name therefore stays an underscore in the topic.
    """
    return "/" + name


def encode_envelope(
    payload: bytes,
    *,
    pose: tuple[float, ...] | None,
    tags: dict[str, Any],
    vector: np.ndarray | None,
) -> bytes:
    header: dict[str, Any] = {}
    if pose is not None:
        header["pose"] = [float(v) for v in pose]
    if tags:
        header["tags"] = tags
    raw = b""
    if vector is not None:
        flat = np.ascontiguousarray(vector, dtype=np.float32).ravel()
        header["dim"] = int(flat.size)
        raw = flat.tobytes()
    head = json.dumps(header, separators=(",", ":"), sort_keys=True).encode("utf-8")
    return b"".join(
        (struct.pack("<I", len(head)), head, struct.pack("<I", len(payload)), payload, raw)
    )


def decode_envelope(data: bytes) -> tuple[dict[str, Any], bytes, np.ndarray | None]:
    """``(header, payload, vector)``. Raises ValueError on anything malformed."""
    if len(data) < 4:
        raise ValueError("observation envelope is shorter than its header length")
    (head_len,) = struct.unpack_from("<I", data, 0)
    at = 4 + head_len
    if at + 4 > len(data):
        raise ValueError("observation envelope header runs past the message")
    header = json.loads(data[4:at].decode("utf-8"))
    (payload_len,) = struct.unpack_from("<I", data, at)
    at += 4
    payload = data[at : at + payload_len]
    if len(payload) != payload_len:
        raise ValueError("observation envelope payload runs past the message")
    at += payload_len
    dim = header.get("dim")
    vector = None
    if dim:
        vector = np.frombuffer(data, dtype=np.float32, count=int(dim), offset=at)
    return header, bytes(payload), vector


def channel_spec(name: str, payload_module: str) -> ChannelSpec:
    """The channel a new dimos stream gets. The schema NAMES the payload type.

    Naming it costs nothing and means a stranger opening the file in Foxglove
    sees ``dimos.msgs.sensor_msgs.PointCloud2.PointCloud2`` rather than an
    anonymous blob, even though nothing outside dimos can decode it.
    """
    return ChannelSpec(
        topic_for(name),
        ENVELOPE_ENCODING,
        schema_name=payload_module,
        schema_encoding=SCHEMA_ENCODING,
        schema_data=json.dumps({"payload": payload_module}).encode("utf-8"),
        metadata={"dimos.stream": name},
    )


class McapVectorStoreConfig(VectorStoreConfig):
    pass


class McapVectorStore(VectorStore):
    """Brute-force nearest-neighbour over vectors already in the mcap.

    The vectors are written by the observation store, inside each observation's
    envelope -- this holds no storage of its own, only the search. ``put`` is
    therefore an index update, not a write; the bytes are on disk already.

    Brute force rather than an ANN index because an mcap has no vector index to
    build one in, and a recording's image index is tens of thousands of vectors:
    10k x 1152 float32 is 44 MB and one matrix multiply. It is linear in the
    number of vectors, so a store with millions of them wants a real index and
    this is the wrong class for it.
    """

    config: McapVectorStoreConfig

    def __init__(self, *, load: Any = None, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        # () -> Iterator[(id, vector)]; called once, on the first search, so that
        # opening a recording does not read every embedding it holds.
        self._load = load
        self._loaded = False
        self._ids: list[int] = []
        self._rows: list[np.ndarray] = []
        self._matrix: np.ndarray | None = None

    def _ensure_loaded(self) -> None:
        if self._loaded:
            return
        self._loaded = True
        if self._load is None:
            return
        for key, vector in self._load():
            self._ids.append(key)
            self._rows.append(np.asarray(vector, dtype=np.float32))
        self._matrix = None

    def put(self, stream_name: str, key: int, embedding: Embedding) -> None:
        self._ensure_loaded()
        self._ids.append(key)
        self._rows.append(np.asarray(embedding.to_numpy(), dtype=np.float32).ravel())
        self._matrix = None

    def search(self, stream_name: str, query: Embedding, k: int | None) -> list[tuple[int, float]]:
        self._ensure_loaded()
        if not self._rows:
            return []
        if self._matrix is None:
            self._matrix = np.vstack(self._rows)
            norms = np.linalg.norm(self._matrix, axis=1, keepdims=True)
            self._matrix = self._matrix / np.maximum(norms, 1e-12)
        q = np.asarray(query.to_numpy(), dtype=np.float32).ravel()
        q = q / max(float(np.linalg.norm(q)), 1e-12)
        sims = self._matrix @ q
        order = np.argsort(-sims)
        if k is not None:
            order = order[:k]
        return [(self._ids[i], float(sims[i])) for i in order]

    def delete(self, stream_name: str, key: int) -> None:
        self._ensure_loaded()
        keep = [i for i, each in enumerate(self._ids) if each != key]
        if len(keep) == len(self._ids):
            return
        self._ids = [self._ids[i] for i in keep]
        self._rows = [self._rows[i] for i in keep]
        self._matrix = None


def write_observation(
    appender: McapAppender,
    channel_id: int,
    obs: Observation[Any],
    payload: bytes,
) -> bytes:
    """Put one observation on its channel -- pose, tags and embedding included.

    Returns the envelope, so the caller can serve the observation back before the
    appender has flushed it.
    """
    embedding = getattr(obs, "embedding", None)
    vector = None
    if embedding is not None:
        vector = np.asarray(embedding.to_numpy(), dtype=np.float32).ravel()
    envelope = encode_envelope(payload, pose=obs.pose_tuple, tags=obs.tags, vector=vector)
    appender.add_message(channel_id, log_time=int(obs.ts * 1e9), data=envelope)
    return envelope


def iter_vectors(messages: Iterator[bytes]) -> Iterator[tuple[int, np.ndarray]]:
    """``(id, vector)`` for every envelope that carries one, ids counting from 0.

    Ids count message position on the channel, which is the id the observation
    store hands out, because a dimos stream is only ever appended to.
    """
    for i, data in enumerate(messages):
        _header, _payload, vector = decode_envelope(data)
        if vector is not None:
            yield i, vector
