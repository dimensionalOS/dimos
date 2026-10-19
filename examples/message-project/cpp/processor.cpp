// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#include <dimos/native.hpp>
#include <story_msgs/msg/device_reading.hpp>

using namespace dimos::native;
using Reading = story_msgs::msg::DeviceReading;

class Processor : public Module {
    Output<Reading> processed_;
public:
    void build(Builder& builder, Config&) override {
        processed_ = builder.output<Reading>("processed");
        builder.input<Reading>("reading", &Processor::process, this);
    }
    void process(const Reading& input) {
        auto output = input;
        output.value += 1;
        processed_.publish(output);
    }
};

int main() { run_with_transport<Processor>(); }
