# Copyright 2026 Dimensional Inc.
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

"""DIM-893: Confirm Linear webhook pipeline connectivity.

This is a no-op test file that proves the Linear → OpenClaw pipeline
successfully received and processed the test issue. Safe to delete
after confirmation.
"""


def test_webhook_pipeline_connected():
    """Confirm the webhook pipeline delivered DIM-893 successfully."""
    assert True, "Linear webhook pipeline is connected"
