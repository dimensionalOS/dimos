// drdds_recv.cpp - Subscribes to /LIDAR/POINTS and /IMU via raw FastDDS,
// writes to POSIX shared memory for DrddsLidarBridge to read.
//
// Links against: libdrdds.so, libfastrtps.so.2.14, libfastcdr.so.2
// Does NOT link against ROS2 (avoids FastDDS version conflict).

#include "shm_transport.h"
#include "dridl/sensor_msgs/msg/PointCloud2.h"
#include "dridl/sensor_msgs/msg/PointCloud2PubSubTypes.h"
#include "dridl/sensor_msgs/msg/Imu.h"
#include "dridl/sensor_msgs/msg/ImuPubSubTypes.h"
#include <fastdds/dds/core/status/SubscriptionMatchedStatus.hpp>
#include <fastdds/dds/domain/DomainParticipant.hpp>
#include <fastdds/dds/domain/DomainParticipantFactory.hpp>
#include <fastdds/dds/subscriber/DataReader.hpp>
#include <fastdds/dds/subscriber/DataReaderListener.hpp>
#include <fastdds/dds/subscriber/Subscriber.hpp>
#include <fastdds/dds/topic/Topic.hpp>
#include <fastdds/dds/topic/TypeSupport.hpp>
#include <fastdds/rtps/transport/UDPv4TransportDescriptor.h>
#include <fastrtps/types/TypesBase.h>
#include <csignal>
#include <iostream>
#include <chrono>
#include <thread>
#include <atomic>
#include <functional>
#include <memory>
#include <semaphore.h>
#include <sys/wait.h>
#include <unistd.h>

static std::atomic<bool> g_running{true};
static void signal_handler(int) { g_running = false; }

static std::atomic<uint64_t> lidar_count{0};
static std::atomic<uint64_t> lidar2_count{0};
static std::atomic<uint64_t> imu_count{0};
static std::atomic<uint64_t> imu_airy_front_count{0};
static std::atomic<uint64_t> imu_airy_rear_count{0};

class RawPointCloud2Listener : public eprosima::fastdds::dds::DataReaderListener {
public:
    using Message = sensor_msgs::msg::PointCloud2;
    using Callback = std::function<void(const Message*)>;

    RawPointCloud2Listener(const char* label, Callback callback)
        : label_(label), callback_(std::move(callback)) {}

    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader*,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override
    {
        matched_.store(info.current_count);
        std::cout << "[drdds_recv] " << label_
                  << " raw_pc2_matched=" << matched_.load() << std::endl;
    }

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override
    {
        Message msg;
        eprosima::fastdds::dds::SampleInfo info;
        while (reader->take_next_sample(&msg, &info) ==
               eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
            if (info.valid_data) {
                callback_(&msg);
            }
        }
    }

    int matched() const { return matched_.load(); }

private:
    const char* label_;
    Callback callback_;
    std::atomic<int> matched_{0};
};

class RawImuListener : public eprosima::fastdds::dds::DataReaderListener {
public:
    using Message = sensor_msgs::msg::Imu;
    using Callback = std::function<void(const Message*)>;

    RawImuListener(const char* label, Callback callback)
        : label_(label), callback_(std::move(callback)) {}

    void on_subscription_matched(
        eprosima::fastdds::dds::DataReader*,
        const eprosima::fastdds::dds::SubscriptionMatchedStatus& info) override
    {
        matched_.store(info.current_count);
        std::cout << "[drdds_recv] " << label_
                  << " raw_imu_matched=" << matched_.load() << std::endl;
    }

    void on_data_available(eprosima::fastdds::dds::DataReader* reader) override
    {
        Message msg;
        eprosima::fastdds::dds::SampleInfo info;
        while (reader->take_next_sample(&msg, &info) ==
               eprosima::fastrtps::types::ReturnCode_t::RETCODE_OK) {
            if (info.valid_data) {
                callback_(&msg);
            }
        }
    }

    int matched() const { return matched_.load(); }

private:
    const char* label_;
    Callback callback_;
    std::atomic<int> matched_{0};
};

int main(int argc, char** argv) {
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Sensor readers run in the forked raw FastDDS child below. The parent
    // creates the POSIX SHM files first so downstream readers can attach early.

    // Create shared memory writers
    drdds_bridge::ShmWriter lidar_shm(
        drdds_bridge::SHM_LIDAR_NAME,
        drdds_bridge::LIDAR_NUM_SLOTS,
        drdds_bridge::LIDAR_SLOT_SIZE);

    // Semaphore for notifying ros2_pub of new data (replaces 1ms sleep polling)
    sem_unlink(drdds_bridge::SHM_NOTIFY_NAME); // clear stale count from previous run
    sem_t* notify_sem = sem_open(drdds_bridge::SHM_NOTIFY_NAME, O_CREAT, 0666, 0);
    if (notify_sem == SEM_FAILED) {
        std::cerr << "[drdds_recv] WARN: sem_open failed, ros2_pub will fall back to polling" << std::endl;
        notify_sem = nullptr;
    }

    drdds_bridge::ShmWriter imu_shm(
        drdds_bridge::SHM_IMU_NAME,
        drdds_bridge::IMU_NUM_SLOTS,
        drdds_bridge::IMU_SLOT_SIZE);

    // Separate-mode extras: rsdriver's send_separately=true publishes
    //   /LIDAR/POINTS2   — rear Airy cloud
    //   /LIDAR/IMU201    — front Airy integrated IMU
    //   /LIDAR/IMU202    — rear Airy integrated IMU
    // In merged mode these publishers simply aren't created — matched stays 0.
    drdds_bridge::ShmWriter lidar2_shm(
        drdds_bridge::SHM_LIDAR2_NAME,
        drdds_bridge::LIDAR_NUM_SLOTS,
        drdds_bridge::LIDAR_SLOT_SIZE);

    drdds_bridge::ShmWriter imu_airy_front_shm(
        drdds_bridge::SHM_IMU_AIRY_FRONT_NAME,
        drdds_bridge::IMU_NUM_SLOTS,
        drdds_bridge::IMU_SLOT_SIZE);

    drdds_bridge::ShmWriter imu_airy_rear_shm(
        drdds_bridge::SHM_IMU_AIRY_REAR_NAME,
        drdds_bridge::IMU_NUM_SLOTS,
        drdds_bridge::IMU_SLOT_SIZE);

    std::cout << "[drdds_recv] SHM initialized. Subscribing to /LIDAR/POINTS, "
              << "/LIDAR/POINTS2, /IMU, /LIDAR/IMU201, /LIDAR/IMU202..." << std::endl;

    // Lidar callback — only passes raw point data + header metadata.
    // PointField descriptors are hardcoded in ros2_pub (RSAIRY fields never change).
    auto on_lidar = [&](const sensor_msgs::msg::PointCloud2* msg) {
        auto* slot = lidar_shm.acquire();
        uint8_t* data = lidar_shm.slot_data(slot);

        uint32_t data_size = msg->data().size();
        if (data_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE) {
            std::cerr << "[drdds_recv] WARN: lidar msg too large: " << data_size << std::endl;
            return;
        }

        slot->data_size = data_size;
        slot->msg_type = 0; // PointCloud2
        slot->stamp_sec = msg->header().stamp().sec();
        slot->stamp_nsec = msg->header().stamp().nanosec();
        slot->height = msg->height();
        slot->width = msg->width();
        slot->point_step = msg->point_step();
        slot->row_step = msg->row_step();
        slot->is_dense = msg->is_dense() ? 1 : 0;
        slot->is_bigendian = msg->is_bigendian() ? 1 : 0;
        slot->fields_size = 0; // fields hardcoded in ros2_pub

        std::memcpy(data, msg->data().data(), data_size);
        lidar_shm.commit();
        if (notify_sem) sem_post(notify_sem);

        uint64_t c = lidar_count.fetch_add(1) + 1;
        if (c % 10 == 1) {
            std::cout << "[drdds_recv] lidar #" << c
                      << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                      << " pts=" << msg->width() * msg->height()
                      << " bytes=" << data_size << std::endl;
        }
    };

    // IMU callback — serializes IMU data into a flat buffer for SHM transport.
    // Layout: [stamp_sec(4) stamp_nsec(4) orientation(4×f64=32) angular_vel(3×f64=24) linear_acc(3×f64=24)]
    // Total: 88 bytes (fits easily in IMU_SLOT_SIZE=256)
    auto on_imu = [&](const sensor_msgs::msg::Imu* msg) {
        auto* slot = imu_shm.acquire();
        uint8_t* data = imu_shm.slot_data(slot);

        slot->data_size = 88;
        slot->msg_type = 1; // Imu
        slot->stamp_sec = msg->header().stamp().sec();
        slot->stamp_nsec = msg->header().stamp().nanosec();

        // Pack IMU data as doubles into flat buffer
        size_t off = 0;
        auto write_d = [&](double v) { std::memcpy(data + off, &v, 8); off += 8; };
        // Orientation quaternion (x, y, z, w)
        write_d(msg->orientation().x());
        write_d(msg->orientation().y());
        write_d(msg->orientation().z());
        write_d(msg->orientation().w());
        // Angular velocity (x, y, z)
        write_d(msg->angular_velocity().x());
        write_d(msg->angular_velocity().y());
        write_d(msg->angular_velocity().z());
        // Linear acceleration (x, y, z)
        write_d(msg->linear_acceleration().x());
        write_d(msg->linear_acceleration().y());
        write_d(msg->linear_acceleration().z());

        imu_shm.commit();
        if (notify_sem) sem_post(notify_sem);

        uint64_t c = imu_count.fetch_add(1) + 1;
        if (c % 200 == 1) {
            std::cout << "[drdds_recv] imu #" << c
                      << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                      << " acc=(" << msg->linear_acceleration().x()
                      << "," << msg->linear_acceleration().y()
                      << "," << msg->linear_acceleration().z() << ")" << std::endl;
        }
    };

    // Lidar2 callback (rear lidar in separate mode, writes to SHM_LIDAR2).
    auto on_lidar2 = [&](const sensor_msgs::msg::PointCloud2* msg) {
        auto* slot = lidar2_shm.acquire();
        uint8_t* data = lidar2_shm.slot_data(slot);
        uint32_t data_size = msg->data().size();
        if (data_size + sizeof(drdds_bridge::SlotHeader) > drdds_bridge::LIDAR_SLOT_SIZE) {
            std::cerr << "[drdds_recv] WARN: lidar2 msg too large: " << data_size << std::endl;
            return;
        }
        slot->data_size = data_size;
        slot->msg_type = 0;
        slot->stamp_sec = msg->header().stamp().sec();
        slot->stamp_nsec = msg->header().stamp().nanosec();
        slot->height = msg->height();
        slot->width = msg->width();
        slot->point_step = msg->point_step();
        slot->row_step = msg->row_step();
        slot->is_dense = msg->is_dense() ? 1 : 0;
        slot->is_bigendian = msg->is_bigendian() ? 1 : 0;
        slot->fields_size = 0;
        std::memcpy(data, msg->data().data(), data_size);
        lidar2_shm.commit();
        if (notify_sem) sem_post(notify_sem);
        uint64_t c = lidar2_count.fetch_add(1) + 1;
        if (c % 10 == 1) {
            std::cout << "[drdds_recv] lidar2 #" << c
                      << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                      << " pts=" << msg->width() * msg->height()
                      << " bytes=" << data_size << std::endl;
        }
    };

    // Airy IMU callback factory — serializes to the given SHM writer and
    // bumps the named counter. Separate SHM per IMU so downstream consumers
    // can pick front-Airy vs rear-Airy vs yesense independently.
    auto make_imu_cb = [](drdds_bridge::ShmWriter* shm, std::atomic<uint64_t>* count, const char* label) {
        return [shm, count, label](const sensor_msgs::msg::Imu* msg) {
            auto* slot = shm->acquire();
            uint8_t* data = shm->slot_data(slot);
            slot->data_size = 88;
            slot->msg_type = 1;
            slot->stamp_sec = msg->header().stamp().sec();
            slot->stamp_nsec = msg->header().stamp().nanosec();
            size_t off = 0;
            auto write_d = [&](double v) { std::memcpy(data + off, &v, 8); off += 8; };
            write_d(msg->orientation().x()); write_d(msg->orientation().y());
            write_d(msg->orientation().z()); write_d(msg->orientation().w());
            write_d(msg->angular_velocity().x()); write_d(msg->angular_velocity().y()); write_d(msg->angular_velocity().z());
            write_d(msg->linear_acceleration().x()); write_d(msg->linear_acceleration().y()); write_d(msg->linear_acceleration().z());
            shm->commit();
            uint64_t c = count->fetch_add(1) + 1;
            if (c % 200 == 1) {
                std::cout << "[drdds_recv] " << label << " #" << c
                          << " stamp=" << msg->header().stamp().sec() << "." << msg->header().stamp().nanosec()
                          << " acc=(" << msg->linear_acceleration().x()
                          << "," << msg->linear_acceleration().y()
                          << "," << msg->linear_acceleration().z() << ")" << std::endl;
            }
        };
    };

    auto on_imu_airy_front = make_imu_cb(&imu_airy_front_shm, &imu_airy_front_count, "imu_airy_front");
    auto on_imu_airy_rear = make_imu_cb(&imu_airy_rear_shm, &imu_airy_rear_count, "imu_airy_rear");

    // Run raw FastDDS sensor readers in a child process. This avoids the
    // post-OTA libdrdds participant state that can leave DrDDSChannel readers
    // matched=0, while preserving the same POSIX SHM output buffers.
    pid_t sensor_pid = fork();
    if (sensor_pid < 0) {
        std::cerr << "[drdds_recv] ERROR: fork for raw sensor child failed" << std::endl;
        return 1;
    }
    if (sensor_pid == 0) {
        using namespace eprosima::fastdds::dds;
        using eprosima::fastdds::rtps::UDPv4TransportDescriptor;

        DomainParticipantQos sensor_pqos;
        sensor_pqos.name("drdds_recv_sensor_raw");
        sensor_pqos.transport().use_builtin_transports = false;
        auto sensor_udp = std::make_shared<UDPv4TransportDescriptor>();
        sensor_udp->maxMessageSize = 65500;
        sensor_udp->receiveBufferSize = 256 * 1024 * 1024;
        sensor_udp->sendBufferSize = 256 * 1024 * 1024;
        sensor_udp->non_blocking_send = false;
        sensor_pqos.transport().user_transports.push_back(sensor_udp);

        auto* sensor_participant = DomainParticipantFactory::get_instance()
            ->create_participant(0, sensor_pqos);
        if (!sensor_participant) {
            std::cerr << "[drdds_recv] ERROR: failed to create raw sensor participant" << std::endl;
            return 1;
        }

        TypeSupport pc2_type(new sensor_msgs::msg::PointCloud2PubSubType());
        pc2_type->setName("sensor_msgs::msg::dds_::PointCloud2_");
        pc2_type.register_type(sensor_participant);
        TypeSupport imu_type(new sensor_msgs::msg::ImuPubSubType());
        imu_type->setName("sensor_msgs::msg::dds_::Imu_");
        imu_type.register_type(sensor_participant);

        auto* pc2_topic_front = sensor_participant->create_topic(
            "rt/LIDAR/POINTS", pc2_type->getName(), TOPIC_QOS_DEFAULT);
        auto* pc2_topic_rear = sensor_participant->create_topic(
            "rt/LIDAR/POINTS2", pc2_type->getName(), TOPIC_QOS_DEFAULT);
        auto* imu_topic_yesense = sensor_participant->create_topic(
            "rt/IMU", imu_type->getName(), TOPIC_QOS_DEFAULT);
        auto* imu_topic_airy_front = sensor_participant->create_topic(
            "rt/LIDAR/IMU201", imu_type->getName(), TOPIC_QOS_DEFAULT);
        auto* imu_topic_airy_rear = sensor_participant->create_topic(
            "rt/LIDAR/IMU202", imu_type->getName(), TOPIC_QOS_DEFAULT);
        auto* sensor_subscriber = sensor_participant->create_subscriber(SUBSCRIBER_QOS_DEFAULT);
        if (!pc2_topic_front || !pc2_topic_rear || !imu_topic_yesense ||
            !imu_topic_airy_front || !imu_topic_airy_rear || !sensor_subscriber) {
            std::cerr << "[drdds_recv] ERROR: failed to create raw sensor topics/subscriber" << std::endl;
            DomainParticipantFactory::get_instance()->delete_participant(sensor_participant);
            return 1;
        }

        DataReaderQos pc2_rqos = DATAREADER_QOS_DEFAULT;
        pc2_rqos.reliability().kind = BEST_EFFORT_RELIABILITY_QOS;
        pc2_rqos.durability().kind = VOLATILE_DURABILITY_QOS;
        pc2_rqos.history().kind = KEEP_LAST_HISTORY_QOS;
        pc2_rqos.history().depth = 8;
        DataReaderQos imu_rqos = DATAREADER_QOS_DEFAULT;
        imu_rqos.reliability().kind = RELIABLE_RELIABILITY_QOS;
        imu_rqos.durability().kind = VOLATILE_DURABILITY_QOS;
        imu_rqos.history().kind = KEEP_LAST_HISTORY_QOS;
        imu_rqos.history().depth = 32;

        RawPointCloud2Listener lidar_listener("lidar_front", on_lidar);
        RawPointCloud2Listener lidar2_listener("lidar_rear", on_lidar2);
        RawImuListener imu_listener("imu_yesense", on_imu);
        RawImuListener imu_airy_front_listener("imu_airy_front", on_imu_airy_front);
        RawImuListener imu_airy_rear_listener("imu_airy_rear", on_imu_airy_rear);
        auto* pc2_reader_front = sensor_subscriber->create_datareader(
            pc2_topic_front, pc2_rqos, &lidar_listener);
        auto* pc2_reader_rear = sensor_subscriber->create_datareader(
            pc2_topic_rear, pc2_rqos, &lidar2_listener);
        auto* imu_reader_yesense = sensor_subscriber->create_datareader(
            imu_topic_yesense, imu_rqos, &imu_listener);
        auto* imu_reader_airy_front = sensor_subscriber->create_datareader(
            imu_topic_airy_front, imu_rqos, &imu_airy_front_listener);
        auto* imu_reader_airy_rear = sensor_subscriber->create_datareader(
            imu_topic_airy_rear, imu_rqos, &imu_airy_rear_listener);
        if (!pc2_reader_front || !pc2_reader_rear || !imu_reader_yesense ||
            !imu_reader_airy_front || !imu_reader_airy_rear) {
            std::cerr << "[drdds_recv] ERROR: failed to create raw sensor readers" << std::endl;
            DomainParticipantFactory::get_instance()->delete_participant(sensor_participant);
            return 1;
        }

        std::cout << "[drdds_recv] raw sensor child created. Waiting for lidar/IMU data..." << std::endl;
        uint64_t last_lidar = 0;
        int no_data_cycles = 0;
        while (g_running) {
            std::this_thread::sleep_for(std::chrono::seconds(5));
            uint64_t cur = lidar_count.load();
            std::cout << "[drdds_recv] status:"
                      << " lidar_front_matched=" << lidar_listener.matched()
                      << " lidar_front_msgs=" << cur
                      << " lidar_rear_matched=" << lidar2_listener.matched()
                      << " lidar_rear_msgs=" << lidar2_count.load()
                      << std::endl;
            std::cout << "[drdds_recv] imu_status:"
                      << " imu_yesense_matched=" << imu_listener.matched()
                      << " imu_yesense_msgs=" << imu_count.load()
                      << " imu_airy_front_matched=" << imu_airy_front_listener.matched()
                      << " imu_airy_front_msgs=" << imu_airy_front_count.load()
                      << " imu_airy_rear_matched=" << imu_airy_rear_listener.matched()
                      << " imu_airy_rear_msgs=" << imu_airy_rear_count.load()
                      << std::endl;

            if (cur == last_lidar) {
                no_data_cycles++;
                if (no_data_cycles >= 6) {
                    std::cerr << "[drdds_recv] WARNING: no lidar data for 30s." << std::endl;
                    no_data_cycles = 0;
                }
            } else {
                no_data_cycles = 0;
            }
            last_lidar = cur;
        }

        std::cout << "[drdds_recv] raw sensor child shutting down." << std::endl;
        sensor_subscriber->delete_datareader(pc2_reader_front);
        sensor_subscriber->delete_datareader(pc2_reader_rear);
        sensor_subscriber->delete_datareader(imu_reader_yesense);
        sensor_subscriber->delete_datareader(imu_reader_airy_front);
        sensor_subscriber->delete_datareader(imu_reader_airy_rear);
        sensor_participant->delete_subscriber(sensor_subscriber);
        sensor_participant->delete_topic(pc2_topic_front);
        sensor_participant->delete_topic(pc2_topic_rear);
        sensor_participant->delete_topic(imu_topic_yesense);
        sensor_participant->delete_topic(imu_topic_airy_front);
        sensor_participant->delete_topic(imu_topic_airy_rear);
        DomainParticipantFactory::get_instance()->delete_participant(sensor_participant);
        return 0;
    }

    std::cout << "[drdds_recv] raw sensor child pid=" << sensor_pid
              << " created. Parent supervising only." << std::endl;

    while (g_running) {
        int status = 0;
        pid_t done = waitpid(sensor_pid, &status, WNOHANG);
        if (done == sensor_pid) {
            std::cerr << "[drdds_recv] ERROR: raw sensor child exited status="
                      << status << std::endl;
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    std::cout << "[drdds_recv] Shutting down." << std::endl;
    kill(sensor_pid, SIGTERM);
    waitpid(sensor_pid, nullptr, 0);
    return 0;
}
