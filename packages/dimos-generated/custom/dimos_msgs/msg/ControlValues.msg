# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# One publisher's sparse interface values; lifecycle transitions use RPC.
string source
# Publisher Unix timestamp in seconds, not receiver-side freshness.
float64 source_ts
# State generation or command session; sequence ordering applies within an epoch.
int64 epoch
int64 sequence
# Parallel arrays; the control contract validates shape and finite values.
string[] interface_names
float64[] values
