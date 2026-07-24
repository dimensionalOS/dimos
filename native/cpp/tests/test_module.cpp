// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <doctest/doctest.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

#include "dimos/native/module.hpp"
#include "dimos/native/transport.hpp"

using namespace dimos::native;
using Bytes = std::vector<uint8_t>;

namespace {

// Mock transport that records publishes, lets tests inject inbound messages,
// and can wedge one channel's publish to test head-of-line isolation.
struct MockTransport : Transport {
    std::mutex m;
    std::vector<std::pair<std::string, Bytes>> published;
    std::unordered_map<std::string, Dispatch> subs;
    std::string qos;
    std::atomic<bool> block_enabled{false};
    std::string block_channel;
    std::atomic<bool> release{false};

    void publish(const std::string& channel, Bytes data) override {
        if (block_enabled.load() && channel == block_channel) {
            while (!release.load()) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
        std::lock_guard<std::mutex> lock(m);
        published.emplace_back(channel, std::move(data));
    }
    void subscribe(const std::string& channel, Dispatch on_msg) override {
        std::lock_guard<std::mutex> lock(m);
        subs[channel] = std::move(on_msg);
    }
    void set_publisher_qos(const std::string& qos_json) override { qos = qos_json; }

    void deliver(const std::string& channel, const Bytes& bytes) {
        Dispatch cb;
        {
            std::lock_guard<std::mutex> lock(m);
            cb = subs.at(channel);
        }
        cb(bytes.data(), bytes.size());
    }
    bool has_published(const std::string& channel) {
        std::lock_guard<std::mutex> lock(m);
        for (const auto& p : published) {
            if (p.first == channel) {
                return true;
            }
        }
        return false;
    }
};

Bytes identity_decode(const uint8_t* d, std::size_t n) { return Bytes(d, d + n); }
Bytes identity_encode(const Bytes& v) { return v; }

// Minimal lcm-gen-shaped message, to exercise the default codecs.
struct Pod {
    std::int32_t v = 0;
    int getEncodedSize() const { return static_cast<int>(sizeof(std::int32_t)); }
    int encode(void* buf, int offset, int maxlen) const {
        if (maxlen - offset < static_cast<int>(sizeof(std::int32_t))) return -1;
        std::memcpy(static_cast<char*>(buf) + offset, &v, sizeof(std::int32_t));
        return static_cast<int>(sizeof(std::int32_t));
    }
    int decode(const void* buf, int offset, int maxlen) {
        if (maxlen - offset < static_cast<int>(sizeof(std::int32_t))) return -1;
        std::memcpy(&v, static_cast<const char*>(buf) + offset, sizeof(std::int32_t));
        return static_cast<int>(sizeof(std::int32_t));
    }
};

struct Sink {
    std::vector<int> got;
    void on(const Pod& p) { got.push_back(p.v); }
};

template <class F>
bool wait_until(F cond, std::chrono::milliseconds timeout = std::chrono::seconds(2)) {
    auto deadline = std::chrono::steady_clock::now() + timeout;
    while (!cond()) {
        if (std::chrono::steady_clock::now() > deadline) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    return true;
}

std::vector<std::thread> start_workers(Builder& builder, Transport& transport) {
    std::vector<std::thread> workers;
    for (const auto& queue : builder.publish_queues()) {
        workers.emplace_back(publish_worker_loop, queue.get(), &transport);
    }
    return workers;
}

void stop_workers(Builder& builder, std::vector<std::thread>& workers) {
    for (const auto& queue : builder.publish_queues()) {
        queue->stop();
    }
    for (std::thread& w : workers) {
        w.join();
    }
}

// RAII worker lifetime so a failing REQUIRE/CHECK cannot strand a spinning
// worker touching soon-destroyed state in the shared doctest binary.
struct WorkerGuard {
    Builder& builder;
    std::vector<std::thread> workers;
    WorkerGuard(Builder& b, Transport& t) : builder(b), workers(start_workers(b, t)) {}
    ~WorkerGuard() { stop_workers(builder, workers); }
};

}  // namespace

TEST_CASE("an inbound message routes through a handler to an output") {
    MockTransport transport;
    Notifier notifier;
    Builder builder({{"data", "/data"}, {"out", "/out"}}, &notifier);

    Bytes received;
    Output<Bytes> out = builder.output<Bytes>("out", identity_encode);
    builder.input<Bytes>("data", identity_decode, [&](Bytes m) {
        received = m;
        out.publish(m);
    });

    for (const auto& route : builder.routes()) {
        transport.subscribe(route.first, route.second);
    }
    WorkerGuard workers(builder, transport);

    transport.deliver("/data", {1, 2, 3});
    for (InputPort* port : builder.input_ports()) {
        port->drain_one();
    }

    CHECK(received == Bytes{1, 2, 3});
    CHECK(wait_until([&] { return transport.has_published("/out"); }));
}

TEST_CASE("member-function handler and default codecs route a message") {
    MockTransport transport;
    Notifier notifier;
    Builder builder({{"in", "/in"}, {"out", "/out"}}, &notifier);

    Sink sink;
    Output<Pod> out = builder.output<Pod>("out");        // encoder defaults to lcm_encode<Pod>
    builder.input<Pod>("in", &Sink::on, &sink);          // member fn + default decoder

    for (const auto& route : builder.routes()) {
        transport.subscribe(route.first, route.second);
    }
    WorkerGuard workers(builder, transport);

    Pod m;
    m.v = 9;
    transport.deliver("/in", dimos::native::lcm_encode(m));
    for (InputPort* port : builder.input_ports()) {
        port->drain_one();
    }
    REQUIRE(sink.got.size() == 1);
    CHECK(sink.got[0] == 9);

    out.publish(m);
    CHECK(wait_until([&] { return transport.has_published("/out"); }));
}

TEST_CASE("topic_for maps declared ports and falls back to /port") {
    Notifier notifier;
    Builder builder({{"cmd_vel", "/robot/cmd_vel"}}, &notifier);
    CHECK(builder.topic_for("cmd_vel") == "/robot/cmd_vel");
    CHECK(builder.topic_for("unmapped") == "/unmapped");
}

TEST_CASE("a full input queue drops newest and caps at capacity") {
    Notifier notifier;
    Builder builder({}, &notifier);
    std::vector<uint8_t> seen;
    builder.input<Bytes>("data", identity_decode, [&](Bytes b) { seen.push_back(b[0]); });

    Dispatch dispatch = builder.routes()[0].second;
    // Distinct values so the surviving set proves which end was dropped.
    for (std::size_t i = 0; i < kInputQueueCapacity + 10; ++i) {
        uint8_t byte = static_cast<uint8_t>(i);
        dispatch(&byte, 1);
    }

    InputPort* port = builder.input_ports()[0];
    while (port->drain_one()) {
    }
    REQUIRE(seen.size() == kInputQueueCapacity);
    // Drop-newest: the first `capacity` messages are kept, later ones dropped.
    for (std::size_t i = 0; i < kInputQueueCapacity; ++i) {
        CHECK(seen[i] == static_cast<uint8_t>(i));
    }
}

TEST_CASE("Notifier does not lose a notification delivered before the wait") {
    // The race the counter closes: a notify that lands after the loop snapshots
    // seq but before it starts waiting. A bare wait_for would block the full
    // timeout. The predicate sees the count moved and returns at once.
    Notifier notifier;
    std::uint64_t seq = notifier.seq();
    notifier.notify();

    auto start = std::chrono::steady_clock::now();
    notifier.wait_for(seq, std::chrono::seconds(5), [] { return false; });
    auto elapsed = std::chrono::steady_clock::now() - start;

    CHECK(elapsed < std::chrono::seconds(1));
}

TEST_CASE("Notifier wakes a waiter from another thread") {
    Notifier notifier;
    std::uint64_t seq = notifier.seq();
    std::thread waker([&] {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        notifier.notify();
    });

    auto start = std::chrono::steady_clock::now();
    notifier.wait_for(seq, std::chrono::seconds(5), [] { return false; });
    auto elapsed = std::chrono::steady_clock::now() - start;
    waker.join();

    CHECK(elapsed < std::chrono::seconds(1));
}

TEST_CASE("Notifier waits out the timeout when no notification arrives") {
    Notifier notifier;
    std::uint64_t seq = notifier.seq();
    auto start = std::chrono::steady_clock::now();
    notifier.wait_for(seq, std::chrono::milliseconds(50), [] { return false; });
    auto elapsed = std::chrono::steady_clock::now() - start;
    CHECK(elapsed >= std::chrono::milliseconds(40));
}

TEST_CASE("a throwing handler is isolated and the loop keeps draining") {
    Notifier notifier;
    Builder builder({}, &notifier);
    int handled = 0;
    builder.input<Bytes>("data", identity_decode, [&](Bytes m) {
        if (!m.empty() && m[0] == 0) {
            throw std::runtime_error("boom");
        }
        ++handled;
    });

    Dispatch dispatch = builder.routes()[0].second;
    uint8_t bad = 0;
    uint8_t good = 1;
    dispatch(&bad, 1);   // handler throws
    dispatch(&good, 1);  // handler succeeds

    InputPort* port = builder.input_ports()[0];
    CHECK(port->drain_one());  // throw is caught, still counts as drained
    CHECK(port->drain_one());  // subsequent message still processed
    CHECK(handled == 1);
}

TEST_CASE("a decode error drops the message and never reaches the handler") {
    Notifier notifier;
    Builder builder({}, &notifier);
    int handled = 0;
    builder.input<Bytes>(
        "data", [](const uint8_t*, std::size_t) -> Bytes { throw std::runtime_error("bad"); },
        [&](Bytes) { ++handled; });

    Dispatch dispatch = builder.routes()[0].second;
    uint8_t byte = 1;
    dispatch(&byte, 1);  // decode throws inside make_dispatch, message dropped

    InputPort* port = builder.input_ports()[0];
    CHECK_FALSE(port->drain_one());
    CHECK(handled == 0);
}

TEST_CASE("a full publish queue drops newest and caps at capacity") {
    PublishQueue queue("/out");
    // Distinct first-bytes so the surviving set proves which end was dropped.
    for (std::size_t i = 0; i < kPublishQueueCapacity + 5; ++i) {
        queue.push({static_cast<uint8_t>(i)});
    }
    queue.stop();

    std::vector<uint8_t> seen;
    Bytes out;
    while (queue.pop(out)) {
        seen.push_back(out[0]);
    }
    REQUIRE(seen.size() == kPublishQueueCapacity);
    // Drop-newest: the first `capacity` pushes are kept, later ones dropped.
    for (std::size_t i = 0; i < kPublishQueueCapacity; ++i) {
        CHECK(seen[i] == static_cast<uint8_t>(i));
    }
}

TEST_CASE("a blocked publish channel does not stall a sibling channel") {
    MockTransport transport;
    transport.block_channel = "/block";
    transport.block_enabled.store(true);

    Notifier notifier;
    Builder builder({{"block_out", "/block"}, {"fast_out", "/fast"}}, &notifier);
    Output<Bytes> block_out = builder.output<Bytes>("block_out", identity_encode);
    Output<Bytes> fast_out = builder.output<Bytes>("fast_out", identity_encode);

    WorkerGuard workers(builder, transport);
    // Release the wedge before the workers join, even if an assertion aborts.
    struct ReleaseGuard {
        MockTransport& t;
        ~ReleaseGuard() { t.release.store(true); }
    } release_guard{transport};

    block_out.publish({1});  // wedges the /block worker inside transport.publish
    fast_out.publish({2});

    CHECK(wait_until([&] { return transport.has_published("/fast"); }));
    CHECK_FALSE(transport.has_published("/block"));
}

TEST_CASE("publishing on a default-constructed Output throws") {
    Output<Bytes> out;
    CHECK_THROWS_AS(out.publish({1}), std::runtime_error);
}

TEST_CASE("parse_stdin_config extracts topics, config, and qos") {
    StdinConfig p = parse_stdin_config(
        R"({"topics":{"data":"/d"},"config":{"x":1},"qos":{"/d":{"reliability":"reliable"}}})");
    CHECK(p.topics.at("data") == "/d");
    CHECK(p.config.at("x") == 1);
    CHECK(p.qos.find("reliable") != std::string::npos);
}

TEST_CASE("parse_stdin_config tolerates a missing config and qos") {
    StdinConfig p = parse_stdin_config(R"({"topics":{}})");
    CHECK(p.config.is_null());
    CHECK(p.qos.empty());
}

namespace {
struct WaitModule : Module {
    void build(Builder&, Config&) override {}
    void invoke_default_handle() { default_handle(); }
};

// Save and restore the process-wide shutdown flag around a test that sets it.
struct ShutdownFlagGuard {
    bool prev = shutdown_flag().load();
    ~ShutdownFlagGuard() { shutdown_flag().store(prev); }
};
}  // namespace

TEST_CASE("default_handle returns without draining once shutdown is requested") {
    ShutdownFlagGuard guard;
    Notifier notifier;
    Builder builder({}, &notifier);
    int handled = 0;
    builder.input<Bytes>("data", identity_decode, [&](Bytes) { ++handled; });

    Dispatch dispatch = builder.routes()[0].second;
    uint8_t byte = 1;
    dispatch(&byte, 1);  // one message waiting to be drained

    WaitModule m;
    m.bind_runtime(&builder.input_ports(), &notifier);
    shutdown_flag().store(true);
    m.invoke_default_handle();  // returns immediately, loop body never runs

    CHECK(handled == 0);
}
