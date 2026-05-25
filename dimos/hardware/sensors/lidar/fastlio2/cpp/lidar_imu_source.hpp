// Copyright 2026 Dimensional Inc.
// SPDX-License-Identifier: Apache-2.0
//
// Polymorphic base for things that feed FAST-LIO with LiDAR + IMU.
// Concrete implementations: LivoxDriverSource (owns the SDK) and
// LcmStreamSource (subscribes to LCM topics). main builds one of them
// based on --input_mode and hands it to run_loop.

#ifndef LIDAR_IMU_SOURCE_HPP_
#define LIDAR_IMU_SOURCE_HPP_

#include <chrono>

class LidarImuSource {
public:
    virtual ~LidarImuSource() = default;

    virtual bool start() = 0;

    virtual void stop() = 0;

    virtual void tick(std::chrono::steady_clock::time_point /*now*/) {}
};

#endif  // LIDAR_IMU_SOURCE_HPP_
