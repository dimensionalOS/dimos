// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Transport seam for dimos C++ native modules. Mirrors the Rust
// `dimos_module::Transport` trait: the runtime talks to the wire only through
// this interface, so the pub/sub protocol is the sole coupling point.

#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace dimos::native {

/// Per-channel callback invoked with each inbound message's raw payload bytes.
/// Decoding and routing happen inside the callback, never in the transport.
using Dispatch = std::function<void(const uint8_t* data, std::size_t len)>;

/// Abstract transport supporting publishing and subscribing to topics.
class Transport {
public:
    virtual ~Transport() = default;

    /// Publish an owned payload on `channel`. Called from that channel's
    /// dedicated publish worker, so blocking here stalls only its own channel.
    virtual void publish(const std::string& channel, std::vector<uint8_t> data) = 0;

    /// Register `on_msg` to receive every payload delivered on `channel`.
    virtual void subscribe(const std::string& channel, Dispatch on_msg) = 0;

    /// Apply publisher QoS. `qos_json` is the raw `qos` object from the stdin
    /// config (or empty when absent). Transports without per-topic QoS (LCM)
    /// leave this as the default no-op.
    virtual void set_publisher_qos(const std::string& qos_json) { (void)qos_json; }
};

}  // namespace dimos::native
