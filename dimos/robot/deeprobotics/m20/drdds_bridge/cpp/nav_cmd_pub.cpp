// nav_cmd_pub.cpp — Raw FastDDS publisher for /NAV_CMD (no ROS, no drdds).
//
// Publishes NavCmd messages to DDS topic "rt/NAV_CMD" using FastDDS 2.14
// directly. Uses the ROS2 topic naming convention (rt/ prefix) so that
// basic_server's rclpy subscriber on AOS can discover and receive commands.
//
// Subscribes to cmd_vel on LCM from dimos and forwards as NavCmd at 10Hz.
//
// Build: included in DrddsLidarBridge nix flake or via cmake.
// Run: launched as part of dimos NativeModule framework.

#include <lcm/lcm-cpp.hpp>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>

#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/publisher/Publisher.hpp>
#include <fastdds/dds/publisher/DataWriter.hpp>
#include <fastdds/dds/publisher/DataWriterListener.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastrtps/attributes/ParticipantAttributes.h>
#include <fastrtps/transport/UDPv4TransportDescriptor.h>

#include "dimos_native_module.hpp"
#include "dridl/dr_msgs/msg/NavCmd.h"
#include "dridl/dr_msgs/msg/NavCmdPubSubTypes.h"
#include "geometry_msgs/Twist.hpp"

// ---------------------------------------------------------------------------
// Globals
// ---------------------------------------------------------------------------

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

// Latest velocity from LCM
static std::mutex g_vel_mutex;
static float g_vx = 0.0f, g_vy = 0.0f, g_vyaw = 0.0f;
static std::atomic<uint64_t> g_vel_seq{0};

// ---------------------------------------------------------------------------
// DataWriter listener for match counting
// ---------------------------------------------------------------------------

class NavCmdWriterListener : public eprosima::fastdds::dds::DataWriterListener {
public:
    void on_publication_matched(
        eprosima::fastdds::dds::DataWriter*,
        const eprosima::fastdds::dds::PublicationMatchedStatus& info) override
    {
        matched_ = info.current_count;
        fprintf(stderr, "[nav_cmd_pub] matched_count=%d\n", matched_.load());
    }
    std::atomic<int> matched_{0};
};

// ---------------------------------------------------------------------------
// LCM handler class for cmd_vel
// ---------------------------------------------------------------------------

class CmdVelHandler {
public:
    void handle(const lcm::ReceiveBuffer*, const std::string&,
                const geometry_msgs::Twist* msg) {
        std::lock_guard<std::mutex> lock(g_vel_mutex);
        g_vx = static_cast<float>(msg->linear.x);
        g_vy = static_cast<float>(msg->linear.y);
        g_vyaw = static_cast<float>(msg->angular.z);
        g_vel_seq++;
    }
};

// ---------------------------------------------------------------------------
// Main
// ---------------------------------------------------------------------------

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    dimos::NativeModule mod(argc, argv);
    std::string cmd_vel_topic = mod.has("cmd_vel") ? mod.topic("cmd_vel") : "";

    if (cmd_vel_topic.empty()) {
        fprintf(stderr, "Usage: %s --cmd_vel <lcm_topic>\n", argv[0]);
        return 1;
    }

    // --- Create FastDDS participant with UDP peer to AOS 10.21.33.103 ---
    using namespace eprosima::fastdds::dds;
    using namespace eprosima::fastrtps::rtps;

    DomainParticipantQos pqos;
    pqos.name("nav_cmd_pub");

    // Add AOS as initial peer for discovery
    Locator_t peer;
    peer.kind = LOCATOR_KIND_UDPv4;
    eprosima::fastrtps::rtps::IPLocator::setIPv4(peer, "10.21.33.103");
    pqos.wire_protocol().builtin.initialPeersList.push_back(peer);

    // Use builtin transports (UDP + SHM)
    pqos.transport().use_builtin_transports = true;

    auto* participant = DomainParticipantFactory::get_instance()
        ->create_participant(0, pqos);
    if (!participant) {
        fprintf(stderr, "[nav_cmd_pub] Failed to create participant\n");
        return 1;
    }

    // --- Register NavCmd type with ROS2-compatible type name ---
    TypeSupport type(new drdds::msg::NavCmdPubSubType());
    // Override type name to match ROS2 rmw_fastrtps convention
    type->setName("drdds::msg::dds_::NavCmd_");
    type.register_type(participant);

    // --- Create topic with rt/ prefix (ROS2 convention) ---
    auto* topic = participant->create_topic(
        "rt/NAV_CMD", type->getName(), TOPIC_QOS_DEFAULT);
    if (!topic) {
        fprintf(stderr, "[nav_cmd_pub] Failed to create topic rt/NAV_CMD\n");
        return 1;
    }

    // --- Create publisher + datawriter with ROS2-matching QoS ---
    auto* publisher = participant->create_publisher(PUBLISHER_QOS_DEFAULT);

    DataWriterQos wqos;
    wqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
    wqos.durability().kind = VOLATILE_DURABILITY_QOS;
    wqos.history().kind = KEEP_LAST_HISTORY_QOS;
    wqos.history().depth = 10;

    NavCmdWriterListener listener;
    auto* writer = publisher->create_datawriter(topic, wqos, &listener);
    if (!writer) {
        fprintf(stderr, "[nav_cmd_pub] Failed to create datawriter\n");
        return 1;
    }

    fprintf(stderr, "[nav_cmd_pub] Publishing on rt/NAV_CMD (type=%s)\n",
            type->getName());
    fprintf(stderr, "[nav_cmd_pub] Subscribing to cmd_vel on LCM: %s\n",
            cmd_vel_topic.c_str());

    // --- Subscribe to cmd_vel on LCM ---
    lcm::LCM lcm;
    if (!lcm.good()) {
        fprintf(stderr, "[nav_cmd_pub] LCM init failed\n");
        return 1;
    }
    CmdVelHandler handler;
    lcm.subscribe(cmd_vel_topic, &CmdVelHandler::handle, &handler);

    // LCM handler thread
    std::thread lcm_thread([&]() {
        while (g_running) {
            lcm.handleTimeout(100);
        }
    });

    // --- Publish loop at 10Hz ---
    drdds::msg::NavCmd nav_cmd;
    uint64_t count = 0;

    while (g_running) {
        {
            std::lock_guard<std::mutex> lock(g_vel_mutex);
            nav_cmd.data().x_vel(g_vx);
            nav_cmd.data().y_vel(g_vy);
            nav_cmd.data().yaw_vel(g_vyaw);
        }

        auto now = std::chrono::system_clock::now();
        auto epoch = now.time_since_epoch();
        auto sec = std::chrono::duration_cast<std::chrono::seconds>(epoch);
        auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(epoch) -
                    std::chrono::duration_cast<std::chrono::nanoseconds>(sec);
        nav_cmd.header().frame_id(0);
        nav_cmd.header().timestamp().sec(static_cast<int32_t>(sec.count()));
        nav_cmd.header().timestamp().nsec(static_cast<uint32_t>(nsec.count()));

        writer->write(&nav_cmd);
        count++;

        if (count % 50 == 1) {
            fprintf(stderr, "[nav_cmd_pub] #%lu x=%.3f y=%.3f yaw=%.3f matched=%d\n",
                    count, g_vx, g_vy, g_vyaw, listener.matched_.load());
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(100));  // 10Hz
    }

    // Zero velocity on shutdown
    nav_cmd.data().x_vel(0.0f);
    nav_cmd.data().y_vel(0.0f);
    nav_cmd.data().yaw_vel(0.0f);
    writer->write(&nav_cmd);

    lcm_thread.join();

    publisher->delete_datawriter(writer);
    participant->delete_publisher(publisher);
    participant->delete_topic(topic);
    DomainParticipantFactory::get_instance()->delete_participant(participant);

    fprintf(stderr, "[nav_cmd_pub] Shutdown (%lu msgs)\n", count);
    return 0;
}
