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

from queue import Queue

from dimos.agents.skills.browser_notification_skill import BrowserNotificationSkill


def test_notify_user_stores_and_broadcasts_alert() -> None:
    notifier = BrowserNotificationSkill()
    client_queue = Queue()
    with notifier._lock:
        notifier._client_queues.add(client_queue)

    try:
        result = notifier.notify_user(
            title="Stop",
            message="Red traffic light detected",
            urgency="high",
            sound=True,
            vibrate=True,
        )

        alert = client_queue.get_nowait()
        assert alert["id"] == 1
        assert alert["title"] == "Stop"
        assert alert["message"] == "Red traffic light detected"
        assert alert["urgency"] == "high"
        assert alert["sound"] is True
        assert alert["vibrate"] is True
        assert notifier._latest_alert == alert
        assert "1 connected client" in result
    finally:
        notifier.stop()


def test_notify_user_normalizes_unknown_urgency() -> None:
    notifier = BrowserNotificationSkill()
    try:
        notifier.notify_user(
            title="Go",
            message="Green traffic light detected",
            urgency="unexpected",
        )
        assert notifier._latest_alert is not None
        assert notifier._latest_alert["urgency"] == "normal"
    finally:
        notifier.stop()
