# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# A keyed region of a point-cloud map. Header matches cloud.header.
std_msgs/Header header
int32 region_id
sensor_msgs/PointCloud2 cloud
