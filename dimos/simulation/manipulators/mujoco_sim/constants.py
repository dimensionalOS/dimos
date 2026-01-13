# Copyright 2025-2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Constants for MuJoCo manipulator simulation."""

# Default control frequency (Hz)
DEFAULT_CONTROL_FREQUENCY = 100.0

# Minimum control frequency to prevent division by zero
MIN_CONTROL_FREQUENCY = 0.01

# Thread join timeout (seconds)
THREAD_JOIN_TIMEOUT = 2.0

# Velocity control threshold (rad/s) - below this, treat as stopped
VELOCITY_STOP_THRESHOLD = 1e-6

# Position target zero threshold (rad) - below this, treat as zero
POSITION_ZERO_THRESHOLD = 1e-6
