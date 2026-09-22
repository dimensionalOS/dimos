// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0

#include <cassert>
#include <iostream>
#include <limits>
#include <stdexcept>

#include "../../dimos/hardware/sensors/lidar/common/point_cloud_utils.hpp"

int main() {
    auto cloud = dimos::make_xyzi_cloud("lidar", 1.25, 2);
    for (int i = 0; i < 2; ++i) {
        float* point = dimos::xyzi_point(cloud, i);
        point[0] = static_cast<float>(i + 1);
        point[1] = 2;
        point[2] = 3;
        point[3] = 0.5;
    }
    for (bool little : {true, false}) {
        auto bytes = dimos::cdr::encode(cloud, little);
        auto decoded = dimos::cdr::decode<sensor_msgs::msg::PointCloud2>(bytes);
        assert(decoded == cloud);
        assert(decoded.header.stamp.sec == 1 && decoded.header.stamp.nanosec == 250000000);
        assert(decoded.width == 2 && decoded.fields.size() == 4 && decoded.data.size() == 32);
        std::cout << (little ? "little" : "big") << "-endian CDR: "
                  << decoded.width << " XYZI points, frame=" << decoded.header.frame_id
                  << ", stamp=" << decoded.header.stamp.sec << "s + "
                  << decoded.header.stamp.nanosec << "ns, intensity="
                  << dimos::xyzi_point(decoded, 0)[3] << '\n';
    }
    auto negative = dimos::time_from_seconds(-0.25);
    assert(negative.sec == -1 && negative.nanosec == 750000000);
    bool rejected = false;
    try { dimos::time_from_seconds(std::numeric_limits<double>::infinity()); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    rejected = false;
    try { dimos::make_xyzi_cloud("lidar", 0, -1); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
    std::cout << "Negative timestamps normalize; invalid timestamps and negative point counts fail.\n";
}
