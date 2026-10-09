# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Per-joint hybrid commands. All arrays must have the same length.
# q: rad; dq: rad/s; tau: N*m. Gains follow the actuator controller's units.
std_msgs/Header header
float64[] q
float64[] dq
float64[] kp
float64[] kd
float64[] tau
