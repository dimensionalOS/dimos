// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <cstdint>
#include <stdexcept>

#include "dimos/native.hpp"
#include "geometry_msgs/Twist.hpp"

using dimos::native::Builder;
using dimos::native::Config;
using dimos::native::Module;
using dimos::native::Output;
using geometry_msgs::Twist;

struct PongConfig {
    std::int64_t sample_config;

    void validate() const {
        if (sample_config < 0 || sample_config > 1000) {
            throw std::runtime_error("sample_config must be in [0, 1000]");
        }
    }
};

class Pong : public Module {
public:
    void build(Builder& builder, Config& config) override {
        // read the config from stdin
        config_ = config.parse<PongConfig>();

        // publish confirm topic
        confirm_ = builder.output<Twist>("confirm");

        // input data topic
        builder.input<Twist>("data", &Pong::on_data, this);
    }

private:
    void on_data(const Twist& msg) {
        Twist reply = msg;
        reply.angular.z = static_cast<double>(config_.sample_config);
        confirm_.publish(reply);
    }

    Output<Twist> confirm_;
    PongConfig config_;
};

int main() {
    dimos::native::run_with_transport<Pong>();
    return 0;
}
