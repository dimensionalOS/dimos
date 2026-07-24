// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <chrono>
#include <cstdint>
#include <thread>

#include "dimos/native.hpp"
#include "geometry_msgs/Twist.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
namespace logging = dimos::native::log;
using geometry_msgs::Twist;

class Ping : public Module {
public:
    void build(Builder& builder, Config& /*config*/) override {
        // publish data topic
        data_ = builder.output<Twist>("data");

        // input confirm topic
        builder.input<Twist>("confirm", &Ping::on_confirm, this);
    }

    void setup() override {
        producer_ = std::thread([this] {
            std::uint64_t seq = 0;
            while (!shutdown_requested()) {
                Twist msg;
                msg.linear.x = static_cast<double>(seq);
                msg.linear.y = 0.0;
                msg.linear.z = 0.0;
                msg.angular.x = 0.0;
                msg.angular.y = 0.0;
                msg.angular.z = 0.0;
                data_.publish(msg);
                ++seq;
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
            }
        });
    }

    void teardown() override {
        if (producer_.joinable()) {
            producer_.join();
        }
    }

private:
    void on_confirm(const Twist& echo) {
        logging::info("echo received (cpp)",
                      {logging::Field("seq", static_cast<std::int64_t>(echo.linear.x)),
                       logging::Field("sample_config",
                                      static_cast<std::int64_t>(echo.angular.z))});
    }

    Output<Twist> data_;
    std::thread producer_;
};

int main() {
    dimos::native::run_with_transport<Ping>();
    return 0;
}
