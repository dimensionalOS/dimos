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

"""Inter-drone radio: a Buzz/Nostr-style agent-to-agent channel.

Design follows Block's Buzz (agents as first-class chat participants with
self-sovereign cryptographic identity, messages as signed events on a shared
channel). Events mirror the Nostr envelope::

    {id, pubkey, created_at, kind, tags, content, sig}

- ``tags`` carry machine-precise facts (own position, claimed search sector,
  target sightings); ``content`` carries free-form agent-to-agent text.
- Every event is signed with the drone's own Ed25519 keypair (Nostr proper
  uses secp256k1/Schnorr; the envelope and trust model are the same, the
  curve is substituted to avoid an extra native dependency).
- Verification is trust-on-first-use: the first pubkey seen for a sender
  is pinned; later events must verify against it.

The module deliberately publishes on the UNPREFIXED shared topic ``/radio``
even when the rest of the stack is namespaced: the radio is the one channel
that is *supposed* to cross the isolation boundary. Everything a drone knows
about its partner comes from these events — that is the whole point of the
demo (epistemic isolation: interpret the other drone only through comms).

Message kinds:
- kind 1 (chat): free text + optional structured tags → injected into the
  partner agent's context ("[RADIO] ..." message).
- kind 2 (beacon): periodic position broadcast. Never reaches the LLM;
  updates the local belief of where the partner is (ghost overlay).
- kind 3 (sighting): target seen at coords; injected into the partner's
  context AND recorded as the believed target position.
"""

from dataclasses import dataclass, field
import json
import threading
import time
from typing import Any

from cryptography.exceptions import InvalidSignature
from cryptography.hazmat.primitives.asymmetric.ed25519 import (
    Ed25519PrivateKey,
    Ed25519PublicKey,
)
import hashlib

from dimos.agents.annotation import skill
from dimos.core.core import rpc
from dimos.core.module import Module, ModuleConfig
from dimos.core.stream import In, Out
from dimos.core.transport import PubSubTransport
from dimos.core.transport_factory import make_transport
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.utils.logging_config import setup_logger

logger = setup_logger()

RADIO_TOPIC = "/radio"

KIND_CHAT = 1
KIND_BEACON = 2
KIND_SIGHTING = 3


def _event_id(pubkey: str, created_at: float, kind: int, tags: list[list[Any]], content: str) -> str:
    payload = json.dumps([0, pubkey, created_at, kind, tags, content], separators=(",", ":"))
    return hashlib.sha256(payload.encode()).hexdigest()


@dataclass
class PeerBelief:
    """What this drone believes about a peer — built ONLY from radio events."""

    name: str
    pubkey: str | None = None
    position: tuple[float, float, float] | None = None
    position_at: float = 0.0
    sector: list[float] | None = None  # [x_min, y_min, x_max, y_max]
    target_sighting: tuple[float, float] | None = None
    sighting_at: float = 0.0
    last_heard: float = 0.0
    messages: int = 0


@dataclass
class RadioStats:
    sent_events: int = 0
    sent_bytes: int = 0
    recv_events: int = 0
    recv_bytes: int = 0
    started_at: float = field(default_factory=time.time)

    def summary(self) -> str:
        dt = max(time.time() - self.started_at, 1e-6)
        return (
            f"sent {self.sent_events} events / {self.sent_bytes} B, "
            f"recv {self.recv_events} events / {self.recv_bytes} B, "
            f"avg TX {self.sent_bytes * 8 / dt / 1000:.2f} kbit/s"
        )


class RadioConfig(ModuleConfig):
    drone_name: str = "drone"
    # Position beacon period (s); 0 disables. Kind-2 events, LLM never sees them.
    beacon_period: float = 4.0
    # Wire format: "hybrid" (Nostr-style: tags + text), "text" (content only),
    # "struct" (tags only). Used by the comms-scheme comparison.
    wire_format: str = "hybrid"


class RadioModule(Module):
    """Signed agent-to-agent radio over the shared /radio LCM topic."""

    config: RadioConfig

    odom: In[PoseStamped]
    human_input: Out[str]  # inbox injection into this stack's McpClient

    _radio: PubSubTransport[str] | None = None
    _key: Ed25519PrivateKey
    _pubkey_hex: str
    _latest_odom: PoseStamped | None = None
    _beacon_thread: threading.Thread | None = None
    _stop_event: threading.Event
    _my_sector: list[float] | None = None

    def __init__(self, **kwargs: Any) -> None:
        super().__init__(**kwargs)
        self._key = Ed25519PrivateKey.generate()
        self._pubkey_hex = self._key.public_key().public_bytes_raw().hex()
        self._stop_event = threading.Event()
        self.peers: dict[str, PeerBelief] = {}
        self.stats = RadioStats()

    # -- lifecycle ------------------------------------------------------------

    @rpc
    def start(self) -> None:
        super().start()
        self.stats = RadioStats()
        self._radio = make_transport(RADIO_TOPIC)
        self._radio.start()
        self._radio.subscribe(self._on_radio_raw)
        from reactivex.disposable import Disposable

        self.register_disposable(Disposable(self.odom.subscribe(self._on_odom)))
        if self.config.beacon_period > 0:
            self._beacon_thread = threading.Thread(target=self._beacon_loop, daemon=True)
            self._beacon_thread.start()
        logger.info(
            f"[radio:{self.config.drone_name}] up, pubkey {self._pubkey_hex[:16]}…, "
            f"format={self.config.wire_format}"
        )

    @rpc
    def stop(self) -> None:
        self._stop_event.set()
        if self._beacon_thread and self._beacon_thread.is_alive():
            self._beacon_thread.join(timeout=2.0)
        if self._radio:
            self._radio.stop()
        logger.info(f"[radio:{self.config.drone_name}] stats: {self.stats.summary()}")
        super().stop()

    def _on_odom(self, odom: PoseStamped) -> None:
        self._latest_odom = odom

    # -- wire -----------------------------------------------------------------

    def _position_tag(self) -> list[Any] | None:
        if self._latest_odom is None:
            return None
        p = self._latest_odom.position
        return ["pos", round(p.x, 2), round(p.y, 2), round(p.z, 2)]

    def _publish_event(self, kind: int, tags: list[list[Any]], content: str) -> dict[str, Any]:
        created_at = round(time.time(), 3)
        fmt = self.config.wire_format
        if fmt == "text":
            # Everything as prose; structured tags folded into the text.
            folded = content
            for t in tags:
                folded += f" [{' '.join(str(v) for v in t)}]"
            tags, content = [["sender", self.config.drone_name]], folded
        elif fmt == "struct":
            tags, content = [["sender", self.config.drone_name], *tags], ""
        else:  # hybrid (default)
            tags = [["sender", self.config.drone_name], *tags]

        event: dict[str, Any] = {
            "pubkey": self._pubkey_hex,
            "created_at": created_at,
            "kind": kind,
            "tags": tags,
            "content": content,
        }
        event["id"] = _event_id(self._pubkey_hex, created_at, kind, tags, content)
        event["sig"] = self._key.sign(event["id"].encode()).hex()
        wire = json.dumps(event, separators=(",", ":"))
        assert self._radio is not None
        self._radio.publish(wire)
        self.stats.sent_events += 1
        self.stats.sent_bytes += len(wire)
        return event

    def _on_radio_raw(self, wire: str) -> None:
        try:
            event = json.loads(wire)
        except (TypeError, ValueError):
            return
        if event.get("pubkey") == self._pubkey_hex:
            return  # own echo
        tags = event.get("tags", [])
        sender = next((t[1] for t in tags if t and t[0] == "sender"), None)
        if sender is None:
            return
        self.stats.recv_events += 1
        self.stats.recv_bytes += len(wire)

        # Verify signature against pinned pubkey (trust on first use).
        belief = self.peers.setdefault(sender, PeerBelief(name=sender))
        pubkey_hex = event.get("pubkey", "")
        if belief.pubkey is None:
            belief.pubkey = pubkey_hex
        elif belief.pubkey != pubkey_hex:
            logger.warning(f"[radio] pubkey mismatch for {sender} — dropping event")
            return
        try:
            Ed25519PublicKey.from_public_bytes(bytes.fromhex(pubkey_hex)).verify(
                bytes.fromhex(event.get("sig", "")), event.get("id", "").encode()
            )
        except (InvalidSignature, ValueError):
            logger.warning(f"[radio] bad signature from {sender} — dropping event")
            return

        belief.last_heard = time.time()
        belief.messages += 1

        # Structured tags → belief updates (the ghost overlay reads these).
        for t in tags:
            if not t:
                continue
            if t[0] == "pos" and len(t) >= 4:
                belief.position = (float(t[1]), float(t[2]), float(t[3]))
                belief.position_at = time.time()
            elif t[0] == "sector" and len(t) >= 5:
                belief.sector = [float(v) for v in t[1:5]]
            elif t[0] == "sighting" and len(t) >= 3:
                belief.target_sighting = (float(t[1]), float(t[2]))
                belief.sighting_at = time.time()

        kind = event.get("kind", KIND_CHAT)
        if kind == KIND_BEACON:
            return  # silent: updates belief only, never reaches the LLM

        # Chat + sighting events reach the agent as context messages.
        content = event.get("content", "")
        extra = []
        if belief.position is not None:
            extra.append(
                f"their position ({belief.position[0]:.1f}, {belief.position[1]:.1f})"
            )
        for t in tags:
            if t and t[0] == "sighting" and len(t) >= 3:
                extra.append(f"TARGET SIGHTED at ({float(t[1]):.1f}, {float(t[2]):.1f})")
            if t and t[0] == "sector" and len(t) >= 5:
                extra.append(
                    f"they claim sector [{t[1]}, {t[2]}] to [{t[3]}, {t[4]}]"
                )
        suffix = f" ({'; '.join(extra)})" if extra else ""
        self.human_input.publish(f"[RADIO from {sender}] {content}{suffix}")

    # -- beacon ---------------------------------------------------------------

    def _beacon_loop(self) -> None:
        while not self._stop_event.wait(self.config.beacon_period):
            tags = []
            pos = self._position_tag()
            if pos:
                tags.append(pos)
            if self._my_sector:
                tags.append(["sector", *self._my_sector])
            if tags:
                try:
                    self._publish_event(KIND_BEACON, tags, "")
                except Exception:
                    logger.warning("beacon publish failed", exc_info=True)

    # -- skills ---------------------------------------------------------------

    @skill
    def send_radio(self, message: str) -> str:
        """Send a free-text radio message to the other drone. Your current
        position is attached automatically so they always know where you are."""
        tags = []
        pos = self._position_tag()
        if pos:
            tags.append(pos)
        self._publish_event(KIND_CHAT, tags, message)
        return "Radio message sent."

    @skill
    def claim_sector(self, x_min: float, y_min: float, x_max: float, y_max: float, message: str = "") -> str:
        """Claim a rectangular search sector over the radio so the other drone
        searches elsewhere. Include a short message explaining your plan."""
        self._my_sector = [x_min, y_min, x_max, y_max]
        tags = [["sector", x_min, y_min, x_max, y_max]]
        pos = self._position_tag()
        if pos:
            tags.append(pos)
        self._publish_event(KIND_CHAT, tags, message or f"Claiming sector ({x_min},{y_min})..({x_max},{y_max}).")
        return f"Sector ({x_min},{y_min})..({x_max},{y_max}) claimed and broadcast."

    @skill
    def report_sighting(self, x: float, y: float, message: str = "") -> str:
        """Broadcast that you SEE the target at map coordinates (x, y). Use the
        moment you spot it so the other drone can converge."""
        tags = [["sighting", round(x, 2), round(y, 2)]]
        pos = self._position_tag()
        if pos:
            tags.append(pos)
        self._publish_event(
            KIND_SIGHTING, tags, message or f"Target in sight at ({x:.1f}, {y:.1f})!"
        )
        return "Sighting broadcast."

    @skill
    def radio_status(self) -> str:
        """What you know about the other drone from radio traffic (their last
        reported position, claimed sector, sightings), plus link stats."""
        if not self.peers:
            return "No radio contact with any other drone yet. " + self.stats.summary()
        lines = []
        now = time.time()
        for peer in self.peers.values():
            desc = [f"{peer.name}: last heard {now - peer.last_heard:.0f}s ago"]
            if peer.position:
                desc.append(
                    f"position ({peer.position[0]:.1f}, {peer.position[1]:.1f}) "
                    f"as of {now - peer.position_at:.0f}s ago"
                )
            if peer.sector:
                desc.append(f"claimed sector {peer.sector}")
            if peer.target_sighting:
                desc.append(
                    f"reported target at ({peer.target_sighting[0]:.1f}, {peer.target_sighting[1]:.1f}) "
                    f"{now - peer.sighting_at:.0f}s ago"
                )
            lines.append("; ".join(desc))
        return " | ".join(lines) + ". Link: " + self.stats.summary()
