// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Module runtime for dimos C++ native modules, mirroring the Rust SDK. The
// transport receive thread decodes onto per-input bounded queues (drop-newest).
// Handlers run serialized on the handle() thread, a lock-free &mut self. One
// worker per output drains publishes so a slow channel blocks only itself.
//
// A module subclasses Module, declares Output<T> members, and wires config,
// outputs, and input handlers in build(). The default handle() dispatches inputs
// until shutdown. A driver with its own loop overrides it.

#pragma once

#include <nlohmann/json.hpp>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <functional>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>
#include <utility>
#include <vector>

#include "dimos/native/config.hpp"
#include "dimos/native/lcm_codec.hpp"
#include "dimos/native/log.hpp"
#include "dimos/native/transport.hpp"

namespace dimos::native {

template <class T>
using EncodeFn = std::function<std::vector<uint8_t>(const T&)>;
template <class T>
using DecodeFn = std::function<T(const uint8_t*, std::size_t)>;
template <class T>
using HandlerFn = std::function<void(T)>;

constexpr std::size_t kInputQueueCapacity = 128;
constexpr std::size_t kPublishQueueCapacity = 32;

// Process-wide shutdown flag, set from an async-signal-safe handler. A native
// module exits when the coordinator sends SIGTERM (or on Ctrl-C).
inline std::atomic<bool>& shutdown_flag() {
    static std::atomic<bool> flag{false};
    return flag;
}

extern "C" inline void dimos_native_handle_signal(int /*sig*/) {
    shutdown_flag().store(true, std::memory_order_relaxed);
}

inline void install_signal_handlers() {
    std::signal(SIGINT, dimos_native_handle_signal);
    std::signal(SIGTERM, dimos_native_handle_signal);
}

// Wakes the handle() dispatch loop when an input receives a message. The
// counter, bumped under the lock, lets the loop snapshot before draining and
// still see a notify that lands mid-round, so the wakeup is never lost.
// Single waiter, the dispatch loop.
class Notifier {
public:
    void notify() {
        {
            std::lock_guard<std::mutex> lock(m_);
            ++pending_;
        }
        cv_.notify_one();
    }

    // Snapshot the notification count. Take this before a drain round so a
    // message that arrives during the round still blocks the following wait.
    std::uint64_t seq() {
        std::lock_guard<std::mutex> lock(m_);
        return pending_;
    }

    // Wait until a notify lands after `seq`, `stop` fires, or the timeout elapses.
    void wait_for(std::uint64_t seq, std::chrono::milliseconds timeout,
                  const std::function<bool()>& stop) {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait_for(lock, timeout, [&] { return pending_ != seq || stop(); });
    }

private:
    std::mutex m_;
    std::condition_variable cv_;
    std::uint64_t pending_ = 0;
};

// Type-erased handle the dispatch loop uses to drain one message from an input.
struct InputPort {
    virtual ~InputPort() = default;
    virtual bool drain_one() = 0;
    virtual Dispatch make_dispatch() = 0;
};

template <class T>
class InputChannel : public InputPort {
public:
    InputChannel(std::string topic, DecodeFn<T> decode, HandlerFn<T> handler, Notifier* notifier)
        : topic_(std::move(topic)),
          decode_(std::move(decode)),
          handler_(std::move(handler)),
          notifier_(notifier) {}

    // Runs on the transport receive thread: decode, then enqueue for the handler.
    Dispatch make_dispatch() override {
        return [this](const uint8_t* data, std::size_t len) {
            T msg;
            try {
                msg = decode_(data, len);
            } catch (const std::exception& e) {
                DIMOS_ERROR_THROTTLED(log::from_secs(1), "decode error",
                                      log::Field("topic", topic_),
                                      log::Field("error", std::string(e.what())));
                return;
            }
            push(std::move(msg));
        };
    }

    bool drain_one() override {
        T msg;
        {
            std::lock_guard<std::mutex> lock(m_);
            if (queue_.empty()) {
                return false;
            }
            msg = std::move(queue_.front());
            queue_.pop_front();
        }
        // Isolate a throwing handler like make_dispatch isolates a bad decode:
        // log, drop this message, keep the dispatch loop alive. Mirrors the
        // Rust runtime, which catch_unwinds each handler.
        try {
            handler_(std::move(msg));
        } catch (const std::exception& e) {
            DIMOS_ERROR_THROTTLED(log::from_secs(1), "handler error",
                                  log::Field("topic", topic_),
                                  log::Field("error", std::string(e.what())));
        } catch (...) {
            DIMOS_ERROR_THROTTLED(log::from_secs(1), "handler error",
                                  log::Field("topic", topic_),
                                  log::Field("error", std::string("non-standard exception")));
        }
        return true;
    }

private:
    void push(T msg) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (queue_.size() >= kInputQueueCapacity) {
                std::uint64_t n = dropped_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (log::check_and_record(last_warn_ns_, log::from_secs(1))) {
                    log::warn("handler full, dropping message",
                              {log::Field("topic", topic_),
                               log::Field("dropped", static_cast<std::int64_t>(n)),
                               log::Field("capacity",
                                          static_cast<std::int64_t>(kInputQueueCapacity))});
                }
                return;
            }
            queue_.push_back(std::move(msg));
        }
        notifier_->notify();
    }

    std::string topic_;
    DecodeFn<T> decode_;
    HandlerFn<T> handler_;
    Notifier* notifier_;
    std::mutex m_;
    std::deque<T> queue_;
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<std::uint64_t> last_warn_ns_{0};
};

// Bounded outbound queue for one channel, drained by a dedicated worker thread.
class PublishQueue {
public:
    explicit PublishQueue(std::string channel) : channel_(std::move(channel)) {}

    void push(std::vector<uint8_t> data) {
        {
            std::lock_guard<std::mutex> lock(m_);
            if (stopped_) {
                return;
            }
            if (queue_.size() >= kPublishQueueCapacity) {
                std::uint64_t n = dropped_.fetch_add(1, std::memory_order_relaxed) + 1;
                if (log::check_and_record(last_warn_ns_, log::from_secs(1))) {
                    log::warn("publish queue full, dropping message",
                              {log::Field("channel", channel_),
                               log::Field("dropped", static_cast<std::int64_t>(n)),
                               log::Field("capacity",
                                          static_cast<std::int64_t>(kPublishQueueCapacity))});
                }
                return;
            }
            queue_.push_back(std::move(data));
        }
        cv_.notify_one();
    }

    // Worker blocks here until a message is available or the queue is stopped
    // and drained. Returns false only when there is nothing left to publish.
    bool pop(std::vector<uint8_t>& out) {
        std::unique_lock<std::mutex> lock(m_);
        cv_.wait(lock, [this] { return stopped_ || !queue_.empty(); });
        if (!queue_.empty()) {
            out = std::move(queue_.front());
            queue_.pop_front();
            return true;
        }
        return false;
    }

    void stop() {
        {
            std::lock_guard<std::mutex> lock(m_);
            stopped_ = true;
        }
        cv_.notify_all();
    }

    const std::string& channel() const { return channel_; }

private:
    std::string channel_;
    std::mutex m_;
    std::condition_variable cv_;
    std::deque<std::vector<uint8_t>> queue_;
    bool stopped_ = false;
    std::atomic<std::uint64_t> dropped_{0};
    std::atomic<std::uint64_t> last_warn_ns_{0};
};

template <class T>
class Output {
public:
    Output() = default;
    Output(EncodeFn<T> encode, std::shared_ptr<PublishQueue> queue)
        : encode_(std::move(encode)), queue_(std::move(queue)) {}

    void publish(const T& msg) const {
        if (!queue_) {
            throw std::runtime_error("Output published before build() wired it");
        }
        queue_->push(encode_(msg));
    }

private:
    EncodeFn<T> encode_;
    std::shared_ptr<PublishQueue> queue_;
};

class Builder {
public:
    Builder(std::unordered_map<std::string, std::string> topics, Notifier* notifier)
        : topics_(std::move(topics)), notifier_(notifier) {}

    // Explicit decoder + handler form. Use for a lambda or a non-lcm decoder.
    template <class T>
    void input(const std::string& port, DecodeFn<T> decode, HandlerFn<T> handler) {
        std::string topic = topic_for(port);
        auto channel = std::make_unique<InputChannel<T>>(topic, std::move(decode),
                                                         std::move(handler), notifier_);
        routes_.emplace_back(topic, channel->make_dispatch());
        input_ports_.push_back(channel.get());
        owned_inputs_.push_back(std::move(channel));
    }

    // Route each `T` on `port` to a method on `self`, decoded by the lcm codec:
    // `builder.input<Twist>("data", &Pong::on_data, this)`. Handlers run
    // serialized on the dispatch thread, so they touch module state without locks.
    template <class T, class Self>
    void input(const std::string& port, void (Self::*handler)(const T&), Self* self,
               DecodeFn<T> decode = lcm_decode<T>) {
        input<T>(port, std::move(decode),
                 [self, handler](T msg) { (self->*handler)(msg); });
    }

    // Declare an output and return its publish handle, encoded by the lcm codec:
    // `out_ = builder.output<Twist>("confirm")` then `out_.publish(msg)`. publish()
    // hands off to a per-channel worker, so it never blocks the caller.
    template <class T>
    Output<T> output(const std::string& port, EncodeFn<T> encode = lcm_encode<T>) {
        std::string topic = topic_for(port);
        auto queue = std::make_shared<PublishQueue>(topic);
        publish_queues_.push_back(queue);
        return Output<T>(std::move(encode), queue);
    }

    std::string topic_for(const std::string& port) const {
        auto it = topics_.find(port);
        return it != topics_.end() ? it->second : "/" + port;
    }

    const std::vector<std::pair<std::string, Dispatch>>& routes() const { return routes_; }
    const std::vector<InputPort*>& input_ports() const { return input_ports_; }
    const std::vector<std::shared_ptr<PublishQueue>>& publish_queues() const {
        return publish_queues_;
    }

private:
    std::unordered_map<std::string, std::string> topics_;
    Notifier* notifier_;
    std::vector<std::pair<std::string, Dispatch>> routes_;
    std::vector<std::unique_ptr<InputPort>> owned_inputs_;
    std::vector<InputPort*> input_ports_;
    std::vector<std::shared_ptr<PublishQueue>> publish_queues_;
};

inline void publish_worker_loop(PublishQueue* queue, Transport* transport) {
    std::vector<uint8_t> data;
    while (queue->pop(data)) {
        transport->publish(queue->channel(), std::move(data));
    }
}

class Module {
public:
    virtual ~Module() = default;

    virtual void build(Builder& builder, Config& config) = 0;
    virtual void setup() {}
    virtual void handle() { default_handle(); }
    virtual void teardown() {}

    // Framework use only: wires the dispatch loop to the built inputs.
    void bind_runtime(const std::vector<InputPort*>* ports, Notifier* notifier) {
        ports_ = ports;
        notifier_ = notifier;
    }

protected:
    bool shutdown_requested() const {
        return shutdown_flag().load(std::memory_order_relaxed);
    }

    // Default main body: round-robin drain inputs (fair, one per input per round)
    // until shutdown. With no inputs, just wait for shutdown like Rust's pending.
    void default_handle() {
        constexpr auto kPoll = std::chrono::milliseconds(100);
        while (!shutdown_requested()) {
            // Snapshot before draining so a message that lands mid-round blocks
            // the wait below instead of sleeping until the poll timeout.
            std::uint64_t seq = notifier_ != nullptr ? notifier_->seq() : 0;
            bool progressed = false;
            if (ports_ != nullptr) {
                for (InputPort* port : *ports_) {
                    if (port->drain_one()) {
                        progressed = true;
                    }
                }
            }
            if (!progressed) {
                if (notifier_ != nullptr) {
                    notifier_->wait_for(seq, kPoll, [this] { return shutdown_requested(); });
                } else {
                    std::this_thread::sleep_for(kPoll);
                }
            }
        }
    }

private:
    const std::vector<InputPort*>* ports_ = nullptr;
    Notifier* notifier_ = nullptr;
};

// Parse the coordinator's stdin line into topics / config / qos.
struct StdinConfig {
    std::unordered_map<std::string, std::string> topics;
    nlohmann::json config;
    std::string qos;
};

inline StdinConfig parse_stdin_config(const std::string& line) {
    StdinConfig out;
    nlohmann::json blob =
        line.empty() ? nlohmann::json::object() : nlohmann::json::parse(line);
    if (!blob.is_object()) {
        throw std::runtime_error("stdin config must be a JSON object");
    }
    if (blob.contains("topics") && blob["topics"].is_object()) {
        for (auto it = blob["topics"].begin(); it != blob["topics"].end(); ++it) {
            if (it.value().is_string()) {
                out.topics[it.key()] = it.value().get<std::string>();
            }
        }
    }
    out.config = blob.contains("config") ? blob["config"] : nlohmann::json();
    if (blob.contains("qos")) {
        out.qos = blob["qos"].dump();
    }
    return out;
}

template <class M>
void run_fallible(std::unique_ptr<Transport> transport) {
    std::string line;
    std::getline(std::cin, line);
    StdinConfig parsed = parse_stdin_config(line);

    transport->set_publisher_qos(parsed.qos);

    Notifier notifier;
    Builder builder(std::move(parsed.topics), &notifier);
    M module;
    Config config(std::move(parsed.config));
    module.build(builder, config);
    config.enforce_all_consumed();

    for (const auto& route : builder.routes()) {
        transport->subscribe(route.first, route.second);
    }

    std::vector<std::thread> workers;
    workers.reserve(builder.publish_queues().size());
    for (const auto& queue : builder.publish_queues()) {
        workers.emplace_back(publish_worker_loop, queue.get(), transport.get());
    }

    install_signal_handlers();
    module.bind_runtime(&builder.input_ports(), &notifier);

    // Stop the workers and transport before builder and notifier are destroyed,
    // on every exit path. Declared last, destroyed first.
    struct Shutdown {
        Builder& builder;
        std::vector<std::thread>& workers;
        std::unique_ptr<Transport>& transport;
        ~Shutdown() {
            for (const auto& queue : builder.publish_queues()) {
                queue->stop();
            }
            for (std::thread& worker : workers) {
                if (worker.joinable()) {
                    worker.join();
                }
            }
            transport.reset();
        }
    } shutdown{builder, workers, transport};

    // Run teardown on the throw path too, so the module's threads join before it
    // is destroyed. A leftover joinable std::thread would call std::terminate.
    try {
        module.setup();
        module.handle();
    } catch (...) {
        try {
            module.teardown();
        } catch (...) {
        }
        throw;
    }
    module.teardown();
}

/// Run module `M` over `transport`, reading config from stdin and blocking until
/// shutdown. Any startup error is logged and the process exits non-zero, matching
/// the Rust runtime.
template <class M>
void run(std::unique_ptr<Transport> transport) {
    try {
        run_fallible<M>(std::move(transport));
    } catch (const std::exception& e) {
        log::error(e.what());
        std::exit(1);
    }
}

}  // namespace dimos::native
