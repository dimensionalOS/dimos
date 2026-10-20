// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
#include "dimos/native.hpp"
#include "dimos/native/cdr_codec.hpp"
#include <telemetry_msgs/msg/device_reading.hpp>

using Reading = telemetry_msgs::msg::DeviceReading;
class Relay : public dimos::native::Module {
    dimos::native::Output<Reading> output_;
public:
    void build(dimos::native::Builder& builder, dimos::native::Config&) override {
        output_ = builder.output<Reading>("cpp", [](const Reading& value) { return dimos::native::cdr_encode(value); });
        builder.input<Reading>("raw", &Relay::receive, this, [](const uint8_t* data, size_t size) { return dimos::native::cdr_decode<Reading>(data, size); });
    }
    void receive(const Reading& input) {
        Reading value = input;
        value.value += 1.0;
        value.label += "/cpp";
        output_.publish(value);
    }
};
int main() { dimos::native::run_with_transport<Relay>(); }
