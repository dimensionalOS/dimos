// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <booster/idl/sensor_msgs/CameraInfo.h>
#include <booster/idl/sensor_msgs/CompressedImage.h>
#include <booster/idl/sensor_msgs/Image.h>
#include <booster/robot/channel/channel_factory.hpp>
#include <booster/robot/channel/channel_subscriber.hpp>
#include <booster_fastdds/fastdds/dds/log/Log.hpp>
#include <lcm/lcm-cpp.hpp>

#include "camera_conversion.hpp"
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

    class PublishRateLimiter
    {
    public:
        void configure(double rate_hz)
        {
            if (!std::isfinite(rate_hz) || rate_hz < 0.0)
            {
                throw std::runtime_error("publish_rate_hz must be finite and non-negative");
            }

            std::lock_guard<std::mutex> lock(mutex_);
            minimum_interval_ = rate_hz == 0.0
                                    ? std::chrono::steady_clock::duration::zero()
                                    : std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                          std::chrono::duration<double>(1.0 / rate_hz));
            next_allowed_at_ = std::chrono::steady_clock::time_point::min();
        }

        bool allow(std::chrono::steady_clock::time_point now)
        {
            if (minimum_interval_ == std::chrono::steady_clock::duration::zero())
            {
                return true;
            }

            std::lock_guard<std::mutex> lock(mutex_);
            if (now < next_allowed_at_)
            {
                return false;
            }
            next_allowed_at_ = now + minimum_interval_;
            return true;
        }

    private:
        std::mutex mutex_;
        std::chrono::steady_clock::duration minimum_interval_{};
        std::chrono::steady_clock::time_point next_allowed_at_{
            std::chrono::steady_clock::time_point::min()};
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
            double configured_publish_rate_hz,
            bool image_reliable,
            bool configured_color_compressed,
            bool configured_depth_enabled,
            bool configured_depth_compressed,
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
            color_rate_limiter_.configure(configured_publish_rate_hz);
            require_topic(configured_color_topic, "color image");
            require_topic(configured_color_info_topic, "color camera info");
            if (configured_depth_enabled)
            {
                depth_rate_limiter_.configure(configured_publish_rate_hz);
                require_topic(configured_depth_topic, "depth image");
                require_topic(configured_depth_info_topic, "depth camera info");
            }

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

            color_info_subscriber_ =
                std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>(
                    configured_color_info_topic, true);
            color_info_subscriber_->InitChannel(
                [this](const void *message)
                { on_camera_info(message, color_info_output_); });
            if (configured_depth_enabled)
            {
                if (configured_depth_compressed)
                {
                    compressed_depth_subscriber_ =
                        std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::CompressedImage>>(
                            configured_depth_topic, image_options);
                    compressed_depth_subscriber_->InitChannel(
                        [this](const void *message)
                        { on_compressed_depth(message); });
                }
                else
                {
                    depth_subscriber_ =
                        std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::Image>>(
                            configured_depth_topic, image_options);
                    depth_subscriber_->InitChannel([this](const void *message)
                                                   { on_depth(message); });
                }
                depth_info_subscriber_ =
                    std::make_unique<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>(
                        configured_depth_info_topic, true);
                depth_info_subscriber_->InitChannel(
                    [this](const void *message)
                    { on_camera_info(message, depth_info_output_); });
            }

            std::cout << "Streaming Booster camera (color=" << configured_color_topic
                      << ", depth="
                      << (configured_depth_enabled ? configured_depth_topic : "disabled")
                      << ")" << std::endl;
        }

        void shutdown() noexcept
        {
            if (shutdown_started_.exchange(true))
            {
                return;
            }
            close_subscriber(depth_info_subscriber_);
            close_subscriber(color_info_subscriber_);
            close_subscriber(compressed_depth_subscriber_);
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
                if (!color_rate_limiter_.allow(std::chrono::steady_clock::now()))
                {
                    return;
                }
                const auto &source = *static_cast<const sensor_msgs::msg::Image *>(raw_message);
                publish_converted_image(
                    source.header(),
                    color_output_,
                    dimos::booster::camera::convert_color_image(
                        source.width(),
                        source.height(),
                        source.encoding(),
                        source.is_bigendian() != 0,
                        source.step(),
                        source.data()));
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
                if (!color_rate_limiter_.allow(std::chrono::steady_clock::now()))
                {
                    return;
                }
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
                if (!depth_rate_limiter_.allow(std::chrono::steady_clock::now()))
                {
                    return;
                }
                const auto &source = *static_cast<const sensor_msgs::msg::Image *>(raw_message);
                publish_converted_image(
                    source.header(),
                    depth_output_,
                    dimos::booster::camera::convert_depth_image(
                        source.width(),
                        source.height(),
                        source.encoding(),
                        source.is_bigendian() != 0,
                        source.step(),
                        source.data(),
                        depth_scale_));
            }
            catch (const std::exception &error)
            {
                std::cerr << "Failed to bridge Booster depth frame: " << error.what() << std::endl;
            }
        }

        void on_compressed_depth(const void *raw_message) noexcept
        {
            try
            {
                if (!depth_rate_limiter_.allow(std::chrono::steady_clock::now()))
                {
                    return;
                }
                const auto &source =
                    *static_cast<const sensor_msgs::msg::CompressedImage *>(raw_message);
                publish_converted_image(
                    source.header(),
                    depth_output_,
                    dimos::booster::camera::convert_compressed_depth_image(
                        source.format(), source.data(), depth_scale_));
            }
            catch (const std::exception &error)
            {
                std::cerr << "Failed to bridge Booster compressed depth frame: "
                          << error.what() << std::endl;
            }
        }

        void publish_converted_image(
            const std_msgs::msg::Header &header,
            const std::string &output,
            dimos::booster::camera::ConvertedImage image)
        {
            sensor_msgs::Image result;
            result.header = convert_header(header);
            result.height = static_cast<int32_t>(image.height);
            result.width = static_cast<int32_t>(image.width);
            result.encoding = std::move(image.encoding);
            result.is_bigendian = 0;
            result.step = static_cast<int32_t>(image.step);
            result.data = std::move(image.data);
            result.data_length = static_cast<int32_t>(result.data.size());
            publish(output, result);
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
        PublishRateLimiter color_rate_limiter_;
        PublishRateLimiter depth_rate_limiter_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::Image>>
            color_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::CompressedImage>>
            compressed_color_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::Image>>
            depth_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::CompressedImage>>
            compressed_depth_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>
            color_info_subscriber_;
        std::unique_ptr<booster::robot::ChannelSubscriber<sensor_msgs::msg::CameraInfo>>
            depth_info_subscriber_;
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
            module.arg_float("publish_rate_hz", 0.0F),
            module.arg_bool("image_reliable", false),
            module.arg_bool("color_compressed"),
            module.arg_bool("depth_enabled", false),
            module.arg_bool("depth_compressed", true),
            module.arg("color_topic"),
            module.arg("depth_topic"),
            module.arg("color_camera_info_topic"),
            module.arg("depth_camera_info_topic"));

        while (keep_running != 0)
        {
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
