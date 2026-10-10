# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Cylinder of one map update; seed cloud/bounds pair by region_id.
std_msgs/Header header
int32 region_id
geometry_msgs/Point center
float64 radius
float64 z_min
float64 z_max
