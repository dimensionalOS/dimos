// drdds_probe.cpp - Minimal libdrdds PointCloud2 receive probe.
//
// Built only on NOS via the local cmake build. This intentionally avoids the
// POSIX SHM bridge path so lidar transport/debug variants can be tested without
// touching production drdds_recv behavior.

#include "drdds/core/drdds_core.h"
#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static std::string arg_value(int argc, char** argv, const std::string& key, const std::string& fallback) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (argv[i] == key) {
            return argv[i + 1];
        }
    }
    return fallback;
}

static bool parse_bool(const std::string& value) {
    return value == "1" || value == "true" || value == "TRUE" || value == "yes";
}

static int parse_int(const std::string& value, int fallback) {
    char* end = nullptr;
    long parsed = std::strtol(value.c_str(), &end, 10);
    return (end && *end == '\0') ? static_cast<int>(parsed) : fallback;
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    const std::string mode = arg_value(argc, argv, "--mode", "channel");
    const std::string topic = arg_value(argc, argv, "--topic", "/LIDAR/POINTS");
    const std::string network = arg_value(argc, argv, "--network", "eth0/eth1");
    std::string prefix = arg_value(argc, argv, "--prefix", "rt");
    if (prefix == "__empty__") {
        prefix.clear();
    }
    const int domain = parse_int(arg_value(argc, argv, "--domain", "0"), 0);
    const int seconds = parse_int(arg_value(argc, argv, "--seconds", "60"), 60);
    const bool use_shm = parse_bool(arg_value(argc, argv, "--use-shm", "0"));
    const bool pub_use_shm = parse_bool(arg_value(argc, argv, "--pub-use-shm", "0"));

    std::atomic<uint64_t> count{0};
    auto callback = [&](const sensor_msgs::msg::PointCloud2* msg) {
        const uint64_t c = count.fetch_add(1) + 1;
        if (c <= 5 || c % 10 == 0) {
            std::cout << "[drdds_probe] msg #" << c
                      << " stamp=" << msg->header().stamp().sec()
                      << "." << msg->header().stamp().nanosec()
                      << " width=" << msg->width()
                      << " height=" << msg->height()
                      << " point_step=" << msg->point_step()
                      << " bytes=" << msg->data().size()
                      << std::endl;
        }
    };

    std::cout << "[drdds_probe] Init domain=" << domain
              << " network=" << network
              << " mode=" << mode
              << " topic=" << topic
              << " use_shm=" << (use_shm ? "true" : "false")
              << " pub_use_shm=" << (pub_use_shm ? "true" : "false")
              << " prefix=" << (prefix.empty() ? "<empty>" : prefix)
              << " seconds=" << seconds
              << std::endl;

    DrDDSManager::Init(domain, network);

    using PointCloud2PubSub = sensor_msgs::msg::PointCloud2PubSubType;
    std::unique_ptr<DrDDSChannel<PointCloud2PubSub>> channel;
    std::unique_ptr<DrDDSChannel<PointCloud2PubSub>> publisher_channel;
    std::unique_ptr<DrDDSSubscriber<PointCloud2PubSub>> subscriber;

    if (mode == "channel") {
        channel = std::make_unique<DrDDSChannel<PointCloud2PubSub>>(
            callback, topic, domain, use_shm, prefix);
    } else if (mode == "subscriber") {
        subscriber = std::make_unique<DrDDSSubscriber<PointCloud2PubSub>>(
            callback, topic, domain, prefix, use_shm);
    } else if (mode == "hybrid") {
        // Matches the symbol pattern seen in localization/charge_manager:
        // a publisher-only channel plus an explicit subscriber. The publisher
        // side stays on the default transport while the reader can opt into
        // SHM/DataSharing for PointCloud2.
        publisher_channel = std::make_unique<DrDDSChannel<PointCloud2PubSub>>(
            topic, domain, pub_use_shm, prefix);
        subscriber = std::make_unique<DrDDSSubscriber<PointCloud2PubSub>>(
            callback, topic, domain, prefix, use_shm);
    } else if (mode == "double") {
        // Reproduces the only observed sample burst so far: a normal callback
        // channel plus a second explicit SHM subscriber on the same topic.
        channel = std::make_unique<DrDDSChannel<PointCloud2PubSub>>(
            callback, topic, domain, pub_use_shm, prefix);
        subscriber = std::make_unique<DrDDSSubscriber<PointCloud2PubSub>>(
            callback, topic, domain, prefix, use_shm);
    } else {
        std::cerr << "[drdds_probe] unknown --mode " << mode
                  << " (expected channel, subscriber, hybrid, or double)" << std::endl;
        DrDDSManager::Delete();
        return 2;
    }

    const auto start = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        const int matched = channel ? channel->GetMatchedCount() : subscriber->GetMatchedCount();
        const int pub_matched = publisher_channel ? publisher_channel->GetMatchedCount() : -1;
        const uint64_t msgs = count.load();
        std::cout << "[drdds_probe] status matched=" << matched
                  << " pub_matched=" << pub_matched
                  << " msgs=" << msgs << std::endl;

        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start);
        if (seconds > 0 && elapsed.count() >= seconds) {
            break;
        }
    }

    channel.reset();
    publisher_channel.reset();
    subscriber.reset();
    DrDDSManager::Delete();
    std::cout << "[drdds_probe] done msgs=" << count.load() << std::endl;
    return count.load() > 0 ? 0 : 1;
}
