// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include "dimos/native.hpp"
#include <dimos_generated/messages.hpp>

using namespace dimos::native;
using dimos_msgs::msg::LineSegments3D;
using sensor_msgs::msg::Image;

class CdrRelay : public Module {
public:
    void build(Builder& builder, Config&) override {
        lines_ = builder.output<LineSegments3D>("lines_out");
        image_ = builder.output<Image>("image_out");
        builder.input<LineSegments3D>("lines_in", &CdrRelay::on_lines, this);
        builder.input<Image>("image_in", &CdrRelay::on_image, this);
    }

private:
    void on_lines(const LineSegments3D& message) {
        auto reply = message;
        for (auto& segment : reply.segments) segment.weight += 1;
        lines_.publish(reply);
    }
    void on_image(const Image& message) { image_.publish(message); }
    Output<LineSegments3D> lines_;
    Output<Image> image_;
};

int main() { dimos::native::run_with_transport<CdrRelay>(); }
