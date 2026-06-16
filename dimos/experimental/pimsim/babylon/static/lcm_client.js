// Copyright 2025-2026 Dimensional Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

// Browser-side LCM client. Talks to the Deno LCM<->WS bridge using the
// same wire format the rest of the dimos bus uses, via @dimos/msgs.
//
// Pinned to specific versions so we don't silently inherit upstream
// schema/wire changes.

import * as msgs from "https://esm.sh/jsr/@dimos/msgs@0.1.4";
const { decode, decodeChannel, encodePacket } = msgs;

const subscribers = new Map(); // channel string -> Set<callback>
const payloadSubscribers = new Map(); // channel string -> Set<callback>
let socket = null;
let reconnectTimer = null;
let bridgeUrl = null;
const listeners = new Set(); // status listeners

function notifyStatus(status, ready) {
  for (const cb of listeners) {
    try { cb({ status, ready }); } catch (e) { console.error("[lcm] status listener", e); }
  }
}

function defaultBridgeUrl() {
  const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
  return `${proto}//${window.location.host}/lcm-ws`;
}

function connect() {
  if (reconnectTimer != null) {
    clearTimeout(reconnectTimer);
    reconnectTimer = null;
  }
  bridgeUrl = bridgeUrl || defaultBridgeUrl();
  socket = new WebSocket(bridgeUrl);
  socket.binaryType = "arraybuffer";

  socket.onopen = () => notifyStatus("live", true);
  socket.onerror = (e) => {
    console.error("[lcm] socket error", e);
    notifyStatus("error", false);
  };
  socket.onclose = () => {
    notifyStatus("reconnecting", false);
    reconnectTimer = setTimeout(connect, 1000);
  };
  socket.onmessage = (event) => {
    if (!(event.data instanceof ArrayBuffer)) return;
    let channel, payload;
    try {
      ({ channel, payload } = decodeChannel(new Uint8Array(event.data)));
    } catch (err) {
      console.error("[lcm] decode failed", err);
      return;
    }

    const subs = subscribers.get(channel);
    if (subs && subs.size > 0) {
      let data;
      try {
        data = decode(payload);
      } catch (err) {
        console.error(`[lcm] payload decode failed for ${channel}`, err);
        return;
      }
      for (const cb of subs) {
        try { cb(data, channel); } catch (e) { console.error(`[lcm] handler for ${channel}`, e); }
      }
    }

    const rawSubs = payloadSubscribers.get(channel);
    if (!rawSubs || rawSubs.size === 0) return;
    for (const cb of rawSubs) {
      try { cb(payload, channel); } catch (e) { console.error(`[lcm] raw handler for ${channel}`, e); }
    }
  };
}

function channelOf(topic, msgClass) {
  // Mirrors @dimos/lcm: actual LCM channel is "<topic>#<package.MessageType>"
  return `${topic}#${msgClass._NAME}`;
}

function subscribe(topic, msgClass, callback) {
  const channel = channelOf(topic, msgClass);
  let subs = subscribers.get(channel);
  if (!subs) {
    subs = new Set();
    subscribers.set(channel, subs);
  }
  subs.add(callback);
  return () => {
    const s = subscribers.get(channel);
    if (s) {
      s.delete(callback);
      if (s.size === 0) subscribers.delete(channel);
    }
  };
}

function subscribePayload(topic, msgClass, callback) {
  return subscribeChannel(channelOf(topic, msgClass), callback);
}

// Raw-channel variant for dimos types @dimos/msgs can't decode (e.g.
// pimsim.EntityStateBatch — length-prefixed JSON on the wire). The
// callback receives the raw payload bytes.
function subscribeChannel(channel, callback) {
  let subs = payloadSubscribers.get(channel);
  if (!subs) {
    subs = new Set();
    payloadSubscribers.set(channel, subs);
  }
  subs.add(callback);
  return () => {
    const s = payloadSubscribers.get(channel);
    if (s) {
      s.delete(callback);
      if (s.size === 0) payloadSubscribers.delete(channel);
    }
  };
}

function publish(topic, message) {
  const cls = message?.constructor;
  if (!cls || !cls._NAME) {
    console.error(`[lcm] publish: message has no _NAME`, message);
    return false;
  }
  if (!socket || socket.readyState !== WebSocket.OPEN) {
    console.warn(`[lcm] publish dropped (socket not open): ${topic}#${cls._NAME}`);
    return false;
  }
  const channel = channelOf(topic, cls);
  let packet;
  try {
    packet = encodePacket(channel, message);
  } catch (err) {
    console.error(`[lcm] encode failed for ${channel}`, err);
    return false;
  }
  socket.send(packet);
  return true;
}

function onStatus(callback) {
  listeners.add(callback);
  return () => listeners.delete(callback);
}

function start(url) {
  if (url) bridgeUrl = url;
  if (socket && socket.readyState === WebSocket.OPEN) return;
  connect();
}

window.dimosMsgs = msgs;
window.dimosLcm = { subscribe, subscribePayload, subscribeChannel, publish, onStatus, start };

// Auto-start so app.js doesn't need to know about us.
start();
