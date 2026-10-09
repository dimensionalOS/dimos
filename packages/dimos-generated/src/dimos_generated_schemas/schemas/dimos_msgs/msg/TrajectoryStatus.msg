# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

std_msgs/Header header
uint8 IDLE=0
uint8 EXECUTING=1
uint8 COMPLETED=2
uint8 ABORTED=3
uint8 FAULT=4
uint8 state
# Fraction completed, in [0, 1].
float64 progress
builtin_interfaces/Duration time_elapsed
builtin_interfaces/Duration time_remaining
string error
