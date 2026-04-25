// fastdds_pc2_probe.cpp - Raw FastDDS PointCloud2 receive probe.
//
// This bypasses libdrdds while using the same generated DR IDL type. It is a
// diagnostic target for isolating libdrdds wrapper behavior from FastDDS
// discovery/transport/type compatibility.

#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"

#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/rtps/transport/shared_mem/SharedMemTransportDescriptor.h>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/types/TypesBase.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdlib>
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

class Pc2Listener : public eprosima::fastdds::dds::DataReaderListener {
public:
    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader*,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override
    {
        matched_ = info.current_count;
        std::cout << "[fastdds_pc2_probe] matched=" << matched_.load() << std::endl;
    }

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override
    {
        sensor_msgs::msg::PointCloud2 msg;
        eprosima::fastdds::dds::SampleInfo info;
        while (reader->take_next_sample(&msg, &info) ==
               eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
            if (!info.valid_data) {
                continue;
            }
            const uint64_t c = count_.fetch_add(1) + 1;
            if (c <= 5 || c % 10 == 0) {
                std::cout << "[fastdds_pc2_probe] msg #" << c
                          << " stamp=" << msg.header().stamp().sec()
                          << "." << msg.header().stamp().nanosec()
                          << " width=" << msg.width()
                          << " height=" << msg.height()
                          << " point_step=" << msg.point_step()
                          << " bytes=" << msg.data().size()
                          << std::endl;
            }
        }
    }

    std::atomic<int> matched_{0};
    std::atomic<uint64_t> count_{0};
};

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    using namespace eprosima::fastdds::dds;
    using eprosima::fastdds::rtps::SharedMemTransportDescriptor;
    using eprosima::fastdds::rtps::UDPv4TransportDescriptor;

    const int domain = parse_int(arg_value(argc, argv, "--domain", "0"), 0);
    const int seconds = parse_int(arg_value(argc, argv, "--seconds", "60"), 60);
    const bool reliable = parse_bool(arg_value(argc, argv, "--reliable", "0"));
    const bool custom_shm = parse_bool(arg_value(argc, argv, "--custom-shm", "0"));
    const bool custom_udp = parse_bool(arg_value(argc, argv, "--custom-udp", "0"));
    const bool keep_builtin = parse_bool(arg_value(argc, argv, "--keep-builtin", "0"));
    const bool data_sharing = parse_bool(arg_value(argc, argv, "--data-sharing", "0"));
    const int recv_buffer = parse_int(arg_value(argc, argv, "--recv-buffer", "8388608"), 8388608);
    const std::string interface = arg_value(argc, argv, "--interface", "");
    const std::string topic_name = arg_value(argc, argv, "--topic", "rt/LIDAR/POINTS");
    const std::string type_name = arg_value(argc, argv, "--type", "sensor_msgs::msg::dds_::PointCloud2_");

    DomainParticipantQos pqos;
    pqos.name("fastdds_pc2_probe");
    pqos.transport().use_builtin_transports = true;
    if ((custom_shm || custom_udp) && !keep_builtin) {
        pqos.transport().use_builtin_transports = false;
    }
    if (custom_udp) {
        auto udp = std::make_shared<UDPv4TransportDescriptor>();
        udp->maxMessageSize = 65500;
        udp->receiveBufferSize = recv_buffer;
        udp->sendBufferSize = recv_buffer;
        udp->non_blocking_send = false;
        if (!interface.empty()) {
            udp->interfaceWhiteList.push_back(interface);
        }
        pqos.transport().user_transports.push_back(udp);
    }
    if (custom_shm) {
        auto shm = std::make_shared<SharedMemTransportDescriptor>();
        shm->segment_size(64 * 1024 * 1024);
        shm->max_message_size(4 * 1024 * 1024);
        shm->port_queue_capacity(512);
        pqos.transport().user_transports.push_back(shm);
    }

    auto* participant = DomainParticipantFactory::get_instance()->create_participant(domain, pqos);
    if (!participant) {
        std::cerr << "[fastdds_pc2_probe] failed to create participant" << std::endl;
        return 2;
    }

    TypeSupport type(new sensor_msgs::msg::PointCloud2PubSubType());
    type->setName(type_name.c_str());
    type.register_type(participant);

    auto* topic = participant->create_topic(topic_name, type->getName(), TOPIC_QOS_DEFAULT);
    if (!topic) {
        std::cerr << "[fastdds_pc2_probe] failed to create topic " << topic_name << std::endl;
        DomainParticipantFactory::get_instance()->delete_participant(participant);
        return 2;
    }

    auto* subscriber = participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
    if (!subscriber) {
        std::cerr << "[fastdds_pc2_probe] failed to create subscriber" << std::endl;
        participant->delete_topic(topic);
        DomainParticipantFactory::get_instance()->delete_participant(participant);
        return 2;
    }

    DataReaderQos rqos = DATAREADER_QOS_DEFAULT;
    rqos.reliability().kind = reliable ? RELIABLE_RELIABILITY_QOS : BEST_EFFORT_RELIABILITY_QOS;
    rqos.durability().kind = VOLATILE_DURABILITY_QOS;
    rqos.history().kind = KEEP_LAST_HISTORY_QOS;
    rqos.history().depth = 5;
    if (data_sharing) {
        rqos.data_sharing().on("/dev/shm");
    }

    Pc2Listener listener;
    auto* reader = subscriber->create_datareader(topic, rqos, &listener);
    if (!reader) {
        std::cerr << "[fastdds_pc2_probe] failed to create reader" << std::endl;
        participant->delete_subscriber(subscriber);
        participant->delete_topic(topic);
        DomainParticipantFactory::get_instance()->delete_participant(participant);
        return 2;
    }

    std::cout << "[fastdds_pc2_probe] domain=" << domain
              << " topic=" << topic_name
              << " type=" << type->getName()
              << " reliability=" << (reliable ? "RELIABLE" : "BEST_EFFORT")
              << " custom_udp=" << (custom_udp ? "true" : "false")
              << " keep_builtin=" << (keep_builtin ? "true" : "false")
              << " interface=" << (interface.empty() ? "<all>" : interface)
              << " recv_buffer=" << recv_buffer
              << " custom_shm=" << (custom_shm ? "true" : "false")
              << " data_sharing=" << (data_sharing ? "on" : "auto")
              << " seconds=" << seconds
              << std::endl;

    const auto start = std::chrono::steady_clock::now();
    while (g_running) {
        std::this_thread::sleep_for(std::chrono::seconds(5));
        std::cout << "[fastdds_pc2_probe] status matched=" << listener.matched_.load()
                  << " msgs=" << listener.count_.load() << std::endl;
        const auto elapsed = std::chrono::duration_cast<std::chrono::seconds>(
            std::chrono::steady_clock::now() - start);
        if (seconds > 0 && elapsed.count() >= seconds) {
            break;
        }
    }

    subscriber->delete_datareader(reader);
    participant->delete_subscriber(subscriber);
    participant->delete_topic(topic);
    DomainParticipantFactory::get_instance()->delete_participant(participant);
    std::cout << "[fastdds_pc2_probe] done msgs=" << listener.count_.load() << std::endl;
    return listener.count_.load() > 0 ? 0 : 1;
}
