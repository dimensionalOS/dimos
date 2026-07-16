// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cctype>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <booster/idl/sensor_msgs/CameraInfo.h>
#include <booster/idl/sensor_msgs/CompressedImage.h>
#include <booster/idl/sensor_msgs/Image.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster_fastdds/fastdds/dds/log/Log.hpp>
#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "sensor_msgs/CameraInfo.hpp"
#include "sensor_msgs/Image.hpp"

namespace
{

    constexpr auto kMainLoopInterval = std::chrono::milliseconds(50);
    constexpr double kDefaultDepthScale = 0.001;
    constexpr size_t kImageQueueCapacity = 1;

    volatile std::sig_atomic_t keep_running = 1;

    void handle_signal(int)
    {
        keep_running = 0;
    }

    std::string lowercase(std::string value)
    {
        std::transform(
            value.begin(), value.end(), value.begin(),
            [](unsigned char character)
            { return static_cast<char>(std::tolower(character)); });
        return value;
    }

    std_msgs::Header convert_header(const std_msgs::msg::Header &source)
    {
        static std::atomic<int32_t> sequence{0};
        std_msgs::Header result;
        result.seq = sequence.fetch_add(1, std::memory_order_relaxed);
        result.stamp.sec = source.stamp().sec();
        result.stamp.nsec = static_cast<int32_t>(source.stamp().nanosec());
        result.frame_id = source.frame_id();
        return result;
    }

    std::string canonical_encoding(const std::string &source)
    {
        const std::string normalized = lowercase(source);
        if (normalized == "rgb8" || normalized == "bgr8" || normalized == "rgba8" ||
            normalized == "bgra8" || normalized == "mono8" || normalized == "mono16")
        {
            return normalized;
        }
        if (normalized == "16uc1")
        {
            return "16UC1";
        }
        if (normalized == "16sc1")
        {
            return "16SC1";
        }
        if (normalized == "32fc1")
        {
            return "32FC1";
        }
        throw std::runtime_error("unsupported Booster image encoding '" + source + "'");
    }

    uint16_t read_uint16(const uint8_t *source, bool big_endian)
    {
        if (big_endian)
        {
            return static_cast<uint16_t>(
                (static_cast<uint16_t>(source[0]) << 8U) | source[1]);
        }
        return static_cast<uint16_t>(
            source[0] | (static_cast<uint16_t>(source[1]) << 8U));
    }

    uint32_t read_uint32(const uint8_t *source, bool big_endian)
    {
        if (big_endian)
        {
            return (static_cast<uint32_t>(source[0]) << 24U) |
                   (static_cast<uint32_t>(source[1]) << 16U) |
                   (static_cast<uint32_t>(source[2]) << 8U) |
                   static_cast<uint32_t>(source[3]);
        }
        return static_cast<uint32_t>(source[0]) |
               (static_cast<uint32_t>(source[1]) << 8U) |
               (static_cast<uint32_t>(source[2]) << 16U) |
               (static_cast<uint32_t>(source[3]) << 24U);
    }

    class FrameAgeMetrics
    {
    public:
        struct Snapshot
        {
            double window_seconds{0.0};
            size_t frames{0};
            size_t bytes{0};
            size_t invalid_timestamps{0};
            size_t negative_ages{0};
            std::vector<double> frame_age_ms;
            std::vector<double> bridge_processing_ms;
        };

        void record(
            const std_msgs::msg::Header &header,
            std::chrono::steady_clock::time_point processing_started,
            size_t bytes)
        {
            const auto published_at = std::chrono::system_clock::now();
            const auto processing_finished = std::chrono::steady_clock::now();
            const double processing_ms = std::chrono::duration<double, std::milli>(
                                             processing_finished - processing_started)
                                             .count();

            bool valid_timestamp = false;
            double frame_age_ms = 0.0;
            const auto seconds = header.stamp().sec();
            const auto nanoseconds = header.stamp().nanosec();
            if (seconds > 0 && nanoseconds < 1000000000U)
            {
                const auto captured_at = std::chrono::system_clock::time_point(
                    std::chrono::seconds(seconds) + std::chrono::nanoseconds(nanoseconds));
                frame_age_ms = std::chrono::duration<double, std::milli>(
                                   published_at - captured_at)
                                   .count();
                valid_timestamp = true;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            ++frames_;
            bytes_ += bytes;
            bridge_processing_ms_.push_back(processing_ms);
            if (valid_timestamp)
            {
                frame_age_ms_.push_back(frame_age_ms);
                if (frame_age_ms < 0.0)
                {
                    ++negative_ages_;
                }
            }
            else
            {
                ++invalid_timestamps_;
            }
        }

        Snapshot snapshot_and_reset()
        {
            const auto now = std::chrono::steady_clock::now();
            std::lock_guard<std::mutex> lock(mutex_);

            Snapshot snapshot;
            snapshot.window_seconds =
                std::chrono::duration<double>(now - window_started_).count();
            snapshot.frames = frames_;
            snapshot.bytes = bytes_;
            snapshot.invalid_timestamps = invalid_timestamps_;
            snapshot.negative_ages = negative_ages_;
            snapshot.frame_age_ms = std::move(frame_age_ms_);
            snapshot.bridge_processing_ms = std::move(bridge_processing_ms_);

            window_started_ = now;
            frames_ = 0;
            bytes_ = 0;
            invalid_timestamps_ = 0;
            negative_ages_ = 0;
            return snapshot;
        }

    private:
        std::mutex mutex_;
        std::chrono::steady_clock::time_point window_started_{
            std::chrono::steady_clock::now()};
        size_t frames_{0};
        size_t bytes_{0};
        size_t invalid_timestamps_{0};
        size_t negative_ages_{0};
        std::vector<double> frame_age_ms_;
        std::vector<double> bridge_processing_ms_;
    };

    class BoosterCameraBridge
    {
    public:
        BoosterCameraBridge(
            lcm::LCM &lcm,
            std::string color_output,
            std::string depth_output,
            std::string color_info_output,
            std::string depth_info_output) : lcm_(lcm),
                                             color_output_(std::move(color_output)),
                                             depth_output_(std::move(depth_output)),
                                             color_info_output_(std::move(color_info_output)),
                                             depth_info_output_(std::move(depth_info_output)) {}

        BoosterCameraBridge(const BoosterCameraBridge &) = delete;
        BoosterCameraBridge &operator=(const BoosterCameraBridge &) = delete;

        ~BoosterCameraBridge()
        {
            shutdown();
        }

        void initialize(
            const std::string &network_interface,
            double configured_depth_scale,
            bool image_reliable,
            bool configured_color_compressed,
            const std::string &configured_color_topic,
            const std::string &configured_depth_topic,
            const std::string &configured_color_info_topic,
            const std::string &configured_depth_info_topic)
        {
            booster_eprosima::fastdds::dds::Log::SetVerbosity(
                booster_eprosima::fastdds::dds::Log::Kind::Info);
            booster_eprosima::fastdds::dds::Log::ReportFilenames(true);
            booster_eprosima::fastdds::dds::Log::ReportFunctions(true);
            if (network_interface.empty())
            {
                booster::robot::ChannelFactory::Instance()->InitDefault(0);
                std::cout << "Initialized Booster DDS with default transports"
                          << std::endl;
            }
            else
            {
                booster::robot::ChannelFactory::Instance()->Init(0, network_interface);
                std::cout << "Initialized Booster DDS on interface " << network_interface
                          << std::endl;
            }
            depth_scale_ = configured_depth_scale;
            require_topic(configured_color_topic, "color image");
            require_topic(configured_depth_topic, "depth image");
            require_topic(configured_color_info_topic, "color camera info");
            require_topic(configured_depth_info_topic, "depth camera info");

            booster::robot::ChannelSubscriberOptions image_options;
            image_options.reliable = image_reliable;
            image_options.executor_options.queue_capacity = kImageQueueCapacity;
            image_options.executor_options.overflow_policy =
                booster::robot::ChannelSubscriberOverflowPolicy::kDropOldest;
            image_options.executor_options.dispatch_mode =
                booster::common::DdsExecutorDispatchMode::kDedicated;

            if (configured_color_compressed)
            {
                compressed_color_subscriber_ =
                    std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::CompressedImage>>(
                        configured_color_topic, image_options);
                compressed_color_subscriber_->InitChannel(
                    [this](const void *message)
                    { on_compressed_color(message); });
            }
            else
            {
                color_subscriber_ =
                    std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::Image>>(
                        configured_color_topic, image_options);
                color_subscriber_->InitChannel(
                    [this](const void *message)
                    { on_color(message); });
            }

            depth_subscriber_ =
                std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::Image>>(
                    configured_depth_topic, image_options);
            depth_subscriber_->InitChannel([this](const void *message)
                                           { on_depth(message); });
            color_info_subscriber_ =
                std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>(
                    configured_color_info_topic, true);
            depth_info_subscriber_ =
                std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>(
                    configured_depth_info_topic, true);
            color_info_subscriber_->InitChannel(
                [this](const void *message)
                { on_camera_info(message, color_info_output_); });
            depth_info_subscriber_->InitChannel(
                [this](const void *message)
                { on_camera_info(message, depth_info_output_); });

            std::cout << "Streaming Booster camera (color=" << configured_color_topic
                      << ", depth=" << configured_depth_topic << ")" << std::endl;
            std::cout << "Frame age uses the source header clock; synchronize robot and host "
                         "with PTP or chrony before interpreting it"
                      << std::endl;
        }

        void log_dds_status()
        {
            const size_t color_matches =
                color_subscriber_
                    ? color_subscriber_->GetMatchedPublicationsCount()
                    : compressed_color_subscriber_->GetMatchedPublicationsCount();
            std::cout << "DDS matches: color=" << color_matches
                      << ", depth=" << depth_subscriber_->GetMatchedPublicationsCount()
                      << ", color_info="
                      << color_info_subscriber_->GetMatchedPublicationsCount()
                      << ", depth_info="
                      << depth_info_subscriber_->GetMatchedPublicationsCount()
                      << std::endl;
            color_metrics_.snapshot_and_reset();
            depth_metrics_.snapshot_and_reset();
        }

        void shutdown() noexcept
        {
            if (shutdown_started_.exchange(true))
            {
                return;
            }
            close_subscriber(depth_info_subscriber_);
            close_subscriber(color_info_subscriber_);
            close_subscriber(depth_subscriber_);
            close_subscriber(compressed_color_subscriber_);
            close_subscriber(color_subscriber_);
        }

    private:
        template <typename Message>
        static void close_subscriber(
            std::unique_ptr<booster::robot::ChannelSubscriber<Message>> &subscriber) noexcept
        {
            if (subscriber)
            {
                subscriber->CloseChannel();
                subscriber.reset();
            }
        }

        static void require_topic(const std::string &topic, const std::string &description)
        {
            if (topic.empty())
            {
                throw std::runtime_error("missing configured " + description + " topic");
            }
        }

        void on_color(const void *raw_message) noexcept
        {
            try
            {
                const auto processing_started = std::chrono::steady_clock::now();
                const auto &source = *static_cast<const sensor_msgs::msg::Image *>(raw_message);

                sensor_msgs::Image result;
                result.header = convert_header(source.header());
                result.height = static_cast<int32_t>(source.height());
                result.width = static_cast<int32_t>(source.width());
                result.encoding = canonical_encoding(source.encoding());
                result.is_bigendian = source.is_bigendian();
                result.step = static_cast<int32_t>(source.step());
                result.data = source.data();
                result.data_length = static_cast<int32_t>(result.data.size());
                publish(color_output_, result);
                color_metrics_.record(source.header(), processing_started, source.data().size());
            }
            catch (const std::exception &error)
            {
                std::cerr << "Failed to bridge Booster color frame: " << error.what() << std::endl;
            }
        }

        void on_compressed_color(const void *raw_message) noexcept
        {
            try
            {
                const auto processing_started = std::chrono::steady_clock::now();
                const auto &source =
                    *static_cast<const sensor_msgs::msg::CompressedImage *>(raw_message);

                sensor_msgs::Image result;
                result.header = convert_header(source.header());
                result.height = 0;
                result.width = 0;
                result.encoding = "jpeg";
                result.is_bigendian = 0;
                result.step = 0;
                result.data = source.data();
                result.data_length = static_cast<int32_t>(result.data.size());
                publish(color_output_, result);
                color_metrics_.record(source.header(), processing_started, source.data().size());
            }
            catch (const std::exception &error)
            {
                std::cerr << "Failed to bridge Booster JPEG frame: " << error.what() << std::endl;
            }
        }

        void on_depth(const void *raw_message) noexcept
        {
            try
            {
                const auto processing_started = std::chrono::steady_clock::now();
                const auto &source = *static_cast<const sensor_msgs::msg::Image *>(raw_message);

                const size_t published_bytes = publish_depth(
                    source.header(), source.width(), source.height(), source.encoding(),
                    source.is_bigendian() != 0, source.step(), source.data());
                depth_metrics_.record(source.header(), processing_started, published_bytes);
            }
            catch (const std::exception &error)
            {
                std::cerr << "Failed to bridge Booster depth frame: " << error.what() << std::endl;
            }
        }

        size_t publish_depth(
            const std_msgs::msg::Header &header,
            uint32_t width,
            uint32_t height,
            const std::string &source_encoding,
            bool source_big_endian,
            uint32_t source_step_value,
            const std::vector<uint8_t> &source_data)
        {
            const std::string encoding = canonical_encoding(source_encoding);
            if (encoding != "16UC1" && encoding != "32FC1")
            {
                throw std::runtime_error("unsupported depth encoding '" + encoding + "'");
            }

            const size_t bytes_per_pixel = encoding == "16UC1" ? 2U : 4U;
            const size_t packed_step = static_cast<size_t>(width) * bytes_per_pixel;
            const size_t source_step =
                source_step_value == 0 ? packed_step : source_step_value;
            const size_t required_size = source_step * height;
            if (source_data.size() < required_size)
            {
                throw std::runtime_error("depth frame data is shorter than height * step");
            }

            std::vector<uint8_t> meters(
                static_cast<size_t>(width) * height * sizeof(float));
            for (uint32_t row = 0; row < height; ++row)
            {
                const uint8_t *source_row = source_data.data() + row * source_step;
                for (uint32_t column = 0; column < width; ++column)
                {
                    float value = 0.0F;
                    if (encoding == "16UC1")
                    {
                        value = static_cast<float>(
                            read_uint16(
                                source_row + column * 2U, source_big_endian) *
                            depth_scale_);
                    }
                    else
                    {
                        const uint32_t bits = read_uint32(
                            source_row + column * 4U, source_big_endian);
                        std::memcpy(&value, &bits, sizeof(value));
                    }
                    std::memcpy(
                        meters.data() +
                            (static_cast<size_t>(row) * width + column) * sizeof(value),
                        &value, sizeof(value));
                }
            }
            const size_t published_bytes = meters.size();
            publish_depth_meters(header, width, height, std::move(meters));
            return published_bytes;
        }

        void publish_depth_meters(
            const std_msgs::msg::Header &header,
            uint32_t width,
            uint32_t height,
            std::vector<uint8_t> meters)
        {
            sensor_msgs::Image result;
            result.header = convert_header(header);
            result.height = static_cast<int32_t>(height);
            result.width = static_cast<int32_t>(width);
            result.encoding = "32FC1";
            result.is_bigendian = 0;
            result.step = static_cast<int32_t>(width * sizeof(float));
            result.data = std::move(meters);
            result.data_length = static_cast<int32_t>(result.data.size());
            publish(depth_output_, result);
        }

        void on_camera_info(const void *raw_message, const std::string &output) noexcept
        {
            try
            {
                const auto &source =
                    *static_cast<const sensor_msgs::msg::CameraInfo *>(raw_message);
                sensor_msgs::CameraInfo result;
                result.header = convert_header(source.header());
                result.height = static_cast<int32_t>(source.height());
                result.width = static_cast<int32_t>(source.width());
                result.distortion_model = source.distortion_model();
                result.D = source.d();
                result.D_length = static_cast<int32_t>(result.D.size());
                std::copy(source.k().begin(), source.k().end(), result.K);
                std::copy(source.r().begin(), source.r().end(), result.R);
                std::copy(source.p().begin(), source.p().end(), result.P);
                result.binning_x = static_cast<int32_t>(source.binning_x());
                result.binning_y = static_cast<int32_t>(source.binning_y());
                result.roi.x_offset = static_cast<int32_t>(source.roi().x_offset());
                result.roi.y_offset = static_cast<int32_t>(source.roi().y_offset());
                result.roi.height = static_cast<int32_t>(source.roi().height());
                result.roi.width = static_cast<int32_t>(source.roi().width());
                result.roi.do_rectify = source.roi().do_rectify();
                publish(output, result);
            }
            catch (const std::exception &error)
            {
                std::cerr << "Failed to bridge Booster camera info: " << error.what() << std::endl;
            }
        }

        template <typename Message>
        void publish(const std::string &channel, const Message &message)
        {
            std::lock_guard<std::mutex> lock(lcm_mutex_);
            if (lcm_.publish(channel, &message) != 0)
            {
                throw std::runtime_error("LCM publish failed on " + channel);
            }
        }

        lcm::LCM &lcm_;
        std::string color_output_;
        std::string depth_output_;
        std::string color_info_output_;
        std::string depth_info_output_;
        double depth_scale_{kDefaultDepthScale};
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::Image>>
            color_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::CompressedImage>>
            compressed_color_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::Image>>
            depth_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>
            color_info_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>
            depth_info_subscriber_;
        FrameAgeMetrics color_metrics_;
        FrameAgeMetrics depth_metrics_;
        std::mutex lcm_mutex_;
        std::atomic_bool shutdown_started_{false};
    };

} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    try
    {
        dimos::NativeModule module(argc, argv);
        lcm::LCM lcm;
        if (!lcm.good())
        {
            throw std::runtime_error("failed to initialize LCM");
        }

        BoosterCameraBridge bridge(
            lcm,
            module.topic("color_image"),
            module.topic("depth_image"),
            module.topic("camera_info"),
            module.topic("depth_camera_info"));
        bridge.initialize(
            module.arg("network_interface"),
            module.arg_float("depth_scale", kDefaultDepthScale),
            module.arg_bool("image_reliable", false),
            module.arg_bool("color_compressed"),
            module.arg("color_topic"),
            module.arg("depth_topic"),
            module.arg("color_camera_info_topic"),
            module.arg("depth_camera_info_topic"));

        auto next_dds_status = std::chrono::steady_clock::now();
        while (keep_running != 0)
        {
            const auto now = std::chrono::steady_clock::now();
            if (now >= next_dds_status)
            {
                bridge.log_dds_status();
                next_dds_status = now + std::chrono::seconds(1);
            }
            std::this_thread::sleep_for(kMainLoopInterval);
        }
        bridge.shutdown();
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Booster camera bridge failed: " << error.what() << std::endl;
        return 1;
    }
}
