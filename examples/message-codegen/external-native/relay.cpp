// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <demo_msgs/msg/telemetry.hpp>
#include "dimos/native.hpp"


using demo_msgs::msg::Telemetry;
using namespace dimos::native;

class Relay : public Module {
public:
    void build(Builder& builder, Config&) override {
        telemetry_out_ = builder.output<Telemetry>("telemetry_out");
        builder.input<Telemetry>("telemetry_in", &Relay::receive, this);
    }
private:
    void receive(const Telemetry& message) {
        auto changed = message;
        changed.application_note += "/cpp-native";
        telemetry_out_.publish(changed);
    }
    Output<Telemetry> telemetry_out_;
};

int main() { dimos::native::run_with_transport<Relay>(); }
