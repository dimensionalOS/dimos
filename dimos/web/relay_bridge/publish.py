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

"""Viewer publishes: tx channel values the relay forwards from the browser,
decoded and published on the bridge's Out ports with an ack or nack per
request (docs/web/bridge.md, "Publishing from the browser")."""

from __future__ import annotations

from collections.abc import Callable, Mapping
import json
import time
from typing import Any

from dimos.core.stream import Out
from dimos.utils.logging_config import setup_logger
from dimos.web.codecs import PublishContext
from dimos.web.relay_bridge.channels import RuntimeChannelSpec
from dimos.web.relay_bridge.protocol import MAX_REQUEST_ID_LEN, DataFrame, Msg, PubAck, PubNack

logger = setup_logger()


def _matches_message_type(value: Any, message_type: type[Any]) -> bool:
    """Decoded-result check with JSON's number/bool subtleties: bool is exact
    (Python bool subclasses int), int excludes bool, float accepts int (JSON
    has one number type) but not bool."""
    if message_type is bool:
        return isinstance(value, bool)
    if message_type is int:
        return isinstance(value, int) and not isinstance(value, bool)
    if message_type is float:
        return isinstance(value, (int, float)) and not isinstance(value, bool)
    return isinstance(value, message_type)


# Publish values nesting deeper than this are rejected before json.loads; the
# SDK enforces the same cap, so both ends agree on what "too deep" means.
_MAX_PUB_DEPTH = 100


def _pub_depth_ok(payload: bytes) -> bool:
    """True when the JSON payload's bracket nesting stays within
    _MAX_PUB_DEPTH (string contents are skipped, so braces in text never
    count). Keeps deep-but-valid JSON from reaching json.loads, whose own
    depth bound is a RecursionError near the interpreter limit."""
    depth = 0
    in_string = False
    escaped = False
    for byte in payload:
        if in_string:
            if escaped:
                escaped = False
            elif byte == 0x5C:  # backslash
                escaped = True
            elif byte == 0x22:  # quote
                in_string = False
        elif byte == 0x22:  # quote
            in_string = True
        elif byte in (0x5B, 0x7B):  # [ {
            depth += 1
            if depth > _MAX_PUB_DEPTH:
                return False
        elif byte in (0x5D, 0x7D):  # ] }
            depth -= 1
    return True


def _parse_pub_meta(meta: dict[str, Any] | None) -> tuple[str, str, float, float | None] | None:
    """(request id, principal, relayTs, clientTs) from a publish frame's
    relay-stamped meta; None when the shape is unusable (a compliant relay
    never produces one)."""
    if not isinstance(meta, dict):
        return None
    request_id = meta.get("id")
    principal = meta.get("principal")
    relay_ts = meta.get("relayTs")
    client_ts = meta.get("clientTs")
    if not isinstance(request_id, str) or not 1 <= len(request_id) <= MAX_REQUEST_ID_LEN:
        return None
    if not isinstance(principal, str):
        return None
    if isinstance(relay_ts, bool) or not isinstance(relay_ts, (int, float)):
        return None
    if client_ts is not None and (
        isinstance(client_ts, bool) or not isinstance(client_ts, (int, float))
    ):
        return None
    return request_id, principal, float(relay_ts), None if client_ts is None else float(client_ts)


class PublishHandler:
    """Decodes forwarded viewer publishes and publishes them on the channel's
    Out port. `specs` are the publish tx channels (decoder resolved) and
    `outputs` the bridge's Out ports, both by stream name."""

    def __init__(
        self,
        robot_id: str,
        specs: Mapping[str, RuntimeChannelSpec],
        outputs: Mapping[str, Out[Any]],
    ) -> None:
        self._robot_id = robot_id
        self._specs = specs
        self._outputs = outputs
        # Frames dropped for an unusable meta shape (no correlatable request
        # id to nack with); a compliant relay never produces one.
        self.invalid = 0

    def on_frame(self, frame: DataFrame, send: Callable[[Msg], None]) -> None:
        """One forwarded viewer publish from the carrier: decode the JSON
        value, publish on the channel's Out port, then acknowledge through
        `send` (a robot-opened one-shot @control stream) - pub_ack only after
        Out.publish() returned, pub_nack for decode/publish failures (bounded
        messages, no tracebacks). Failures never raise (an exception would
        recycle the whole relay session) and never touch other channels.
        """
        ch = frame.header.ch
        parsed = _parse_pub_meta(frame.header.meta)
        if parsed is None:
            # No trustworthy request id to nack with; the relay's publish
            # timeout settles the viewer side.
            self.invalid += 1
            logger.warning(f"dropping publish frame on {ch!r}: unusable meta")
            return
        request_id, principal, relay_ts, client_ts = parsed

        def nack(code: str, error: Exception | str) -> None:
            message = error if isinstance(error, str) else f"{type(error).__name__}: {error}"
            send(PubNack(id=request_id, code=code, message=message[:200]))

        spec = self._specs.get(ch)
        if spec is None:
            nack("unknown_channel", f"no publishable channel {ch!r}")
            return
        if not _pub_depth_ok(frame.payload):
            nack("decode_failed", f"value nests deeper than {_MAX_PUB_DEPTH} levels")
            return
        try:
            value = json.loads(frame.payload)
        except (ValueError, RecursionError) as e:
            # RecursionError as backstop: escaping here would recycle the
            # whole relay session over one request's payload.
            nack("decode_failed", e)
            return
        assert spec.decoder is not None  # _adopt_authored_specs required it
        context = PublishContext(
            robot=self._robot_id,
            ch=ch,
            relay_ts=relay_ts,
            request_id=request_id,
            principal=principal,
            client_ts=client_ts,
        )
        try:
            if spec.decoder_takes_context:
                result = spec.decoder(value, context)
            else:
                result = spec.decoder(value)
        except Exception as e:
            nack("decode_failed", e)
            return
        if not _matches_message_type(result, spec.message_type):
            nack(
                "decode_failed",
                f"decoder returned {type(result).__name__}, not {spec.message_type.__name__}",
            )
            return
        try:
            self._outputs[ch].publish(result)
        except Exception as e:
            nack("publish_failed", e)
            return
        send(PubAck(id=request_id, ch=ch, relayTs=relay_ts, bridgeTs=time.time()))
