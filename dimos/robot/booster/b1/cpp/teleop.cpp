// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <iostream>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <booster/robot/b1/b1_loco_client.hpp>
#include <lcm/lcm-cpp.hpp>

#include "dimos_native_module.hpp"
#include "geometry_msgs/Twist.hpp"

namespace
{

    constexpr float kMaxLinearVelocity = 0.2F;
    constexpr float kMaxAngularVelocity = 1.0F;
    constexpr double kDefaultCommandTimeoutSec = 0.25;
    constexpr int kLcmPollTimeoutMs = 20;
    constexpr auto kWatchdogPollInterval = std::chrono::milliseconds(10);

    volatile std::sig_atomic_t keep_running = 1;

    void handle_signal(int)
    {
        keep_running = 0;
    }

    float clipped_velocity(double value, float limit)
    {
        if (!std::isfinite(value))
        {
            return 0.0F;
        }
        return std::clamp(static_cast<float>(value), -limit, limit);
    }

    class BoosterB1Driver
    {
    public:
        BoosterB1Driver(
            std::string network_interface,
            double command_timeout_sec) : network_interface_(std::move(network_interface)),
                                          command_timeout_(command_timeout_sec),
                                          last_command_(std::chrono::steady_clock::now())
        {
            if (network_interface_.empty())
            {
                throw std::invalid_argument("network_interface must not be empty");
            }
            if (!std::isfinite(command_timeout_sec) || command_timeout_sec <= 0.0)
            {
                throw std::invalid_argument("command_timeout_sec must be positive");
            }
        }

        BoosterB1Driver(const BoosterB1Driver &) = delete;
        BoosterB1Driver &operator=(const BoosterB1Driver &) = delete;

        ~BoosterB1Driver()
        {
            shutdown();
        }

        void initialize()
        {
            booster::robot::ChannelFactory::Instance()->Init(0, network_interface_);
            client_.Init();
            client_initialized_ = true;

            const int32_t result = client_.ChangeMode(booster::robot::RobotMode::kPrepare);
            if (result != 0)
            {
                throw std::runtime_error(
                    "failed to switch Booster B1 to prepare mode, error " +
                    std::to_string(result));
            }

            {
                std::lock_guard<std::mutex> lock(command_mutex_);
                last_command_ = std::chrono::steady_clock::now();
                timeout_stop_sent_ = false;
            }
            watchdog_running_.store(true);
            watchdog_ = std::thread(&BoosterB1Driver::watchdog_loop, this);
            std::cout << "Booster B1 prepare mode enabled" << std::endl;
        }

        void subscribe(lcm::LCM &lcm, const std::string &topic)
        {
            lcm.subscribe(topic, &BoosterB1Driver::on_cmd_vel, this);
        }

        void shutdown() noexcept
        {
            if (shutdown_started_.exchange(true))
            {
                return;
            }

            watchdog_running_.store(false);
            if (watchdog_.joinable())
            {
                watchdog_.join();
            }

            if (!client_initialized_)
            {
                return;
            }

            std::lock_guard<std::mutex> lock(command_mutex_);
            if (walking_mode_enabled_)
            {
                send_move_locked(0.0F, 0.0F, 0.0F, "shutdown");
            }
            switch_to_prepare_locked();
        }

    private:
        void on_cmd_vel(
            const lcm::ReceiveBuffer *,
            const std::string &,
            const geometry_msgs::Twist *command)
        {
            const float vx = clipped_velocity(command->linear.x, kMaxLinearVelocity);
            const float vy = clipped_velocity(command->linear.y, kMaxLinearVelocity);
            const float wz = clipped_velocity(command->angular.z, kMaxAngularVelocity);

            std::lock_guard<std::mutex> lock(command_mutex_);
            if (!walking_mode_enabled_)
            {
                if (vx > 0.0F)
                {
                    switch_to_walking_locked();
                }
                return;
            }

            last_command_ = std::chrono::steady_clock::now();
            timeout_stop_sent_ = false;
            send_move_locked(vx, vy, wz, "cmd_vel");
        }

        void watchdog_loop()
        {
            while (watchdog_running_.load())
            {
                std::this_thread::sleep_for(kWatchdogPollInterval);

                std::lock_guard<std::mutex> lock(command_mutex_);
                if (!walking_mode_enabled_)
                {
                    continue;
                }

                const auto command_age = std::chrono::steady_clock::now() - last_command_;
                if (!timeout_stop_sent_ && command_age >= command_timeout_)
                {
                    timeout_stop_sent_ =
                        send_move_locked(0.0F, 0.0F, 0.0F, "command watchdog");
                    if (timeout_stop_sent_)
                    {
                        std::cout << "Booster B1 command watchdog stopped locomotion"
                                  << std::endl;
                    }
                }
            }
        }

        bool send_move_locked(float vx, float vy, float wz, const char *source) noexcept
        {
            try
            {
                const int32_t result = client_.Move(vx, vy, wz);
                if (result != 0)
                {
                    std::cerr << "Booster B1 Move failed from " << source << ", error "
                              << result << std::endl;
                    return false;
                }
                return true;
            }
            catch (const std::exception &error)
            {
                std::cerr << "Booster B1 Move threw from " << source << ": "
                          << error.what() << std::endl;
            }
            catch (...)
            {
                std::cerr << "Booster B1 Move threw an unknown exception from " << source
                          << std::endl;
            }
            return false;
        }

        bool switch_to_walking_locked() noexcept
        {
            try
            {
                const int32_t result =
                    client_.ChangeMode(booster::robot::RobotMode::kWalking);
                if (result != 0)
                {
                    std::cerr << "Failed to switch Booster B1 to walking mode, error "
                              << result << std::endl;
                    return false;
                }

                walking_mode_enabled_ = true;
                last_command_ = std::chrono::steady_clock::now();
                timeout_stop_sent_ = false;
                std::cout << "Booster B1 walking mode enabled" << std::endl;
                return true;
            }
            catch (const std::exception &error)
            {
                std::cerr << "Switching Booster B1 to walking mode threw: " << error.what()
                          << std::endl;
            }
            catch (...)
            {
                std::cerr << "Switching Booster B1 to walking mode threw an unknown exception"
                          << std::endl;
            }
            return false;
        }

        void switch_to_prepare_locked() noexcept
        {
            walking_mode_enabled_ = false;
            try
            {
                const int32_t result =
                    client_.ChangeMode(booster::robot::RobotMode::kPrepare);
                if (result != 0)
                {
                    std::cerr << "Failed to switch Booster B1 to prepare mode, error "
                              << result << std::endl;
                }
            }
            catch (const std::exception &error)
            {
                std::cerr << "Switching Booster B1 to prepare mode threw: " << error.what()
                          << std::endl;
            }
            catch (...)
            {
                std::cerr << "Switching Booster B1 to prepare mode threw an unknown exception"
                          << std::endl;
            }
        }

        std::string network_interface_;
        std::chrono::duration<double> command_timeout_;
        std::chrono::steady_clock::time_point last_command_;
        booster::robot::b1::B1LocoClient client_;
        std::mutex command_mutex_;
        std::thread watchdog_;
        std::atomic_bool watchdog_running_{false};
        std::atomic_bool shutdown_started_{false};
        bool client_initialized_{false};
        bool timeout_stop_sent_{false};
        bool walking_mode_enabled_{false};
    };

} // namespace

int main(int argc, char **argv)
{
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    try
    {
        dimos::NativeModule module(argc, argv);
        const std::string network_interface = module.arg_required("network_interface");
        const double command_timeout_sec =
            module.arg_float("command_timeout_sec", kDefaultCommandTimeoutSec);

        lcm::LCM lcm;
        if (!lcm.good())
        {
            throw std::runtime_error("failed to initialize LCM");
        }

        BoosterB1Driver driver(network_interface, command_timeout_sec);
        driver.initialize();
        driver.subscribe(lcm, module.topic("cmd_vel"));

        while (keep_running != 0)
        {
            const int result = lcm.handleTimeout(kLcmPollTimeoutMs);
            if (result < 0 && keep_running != 0)
            {
                throw std::runtime_error("LCM receive loop failed");
            }
        }

        driver.shutdown();
        return 0;
    }
    catch (const std::exception &error)
    {
        std::cerr << "Booster B1 native driver failed: " << error.what() << std::endl;
        return 1;
    }
}
