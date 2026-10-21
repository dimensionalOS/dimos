# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Continuous-time IMU noise model. Extrinsics are supplied separately through TF.
std_msgs/Header header
# rad/s/sqrt(Hz)
float64 gyro_noise_density
# rad/s^2/sqrt(Hz)
float64 gyro_random_walk
# m/s^2/sqrt(Hz)
float64 accel_noise_density
# m/s^3/sqrt(Hz)
float64 accel_random_walk
# Delivered sample rate in Hz.
float64 frequency
