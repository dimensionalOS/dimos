# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

std_msgs/Header header
int32[3] key
# Row-major Nx3 vertex coordinates and per-vertex normals.
float32[] vertices
float32[] normals
# Row-major Mx3 triangle vertex indices.
uint32[] faces
