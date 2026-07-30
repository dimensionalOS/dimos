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

"""Private DimSim control publisher for terminating an active eval turn."""

from __future__ import annotations

import argparse
import sys
from typing import Any, Protocol

from dimos.core.transport_factory import apply_transport_arg, make_transport

EVAL_TURN_CONTROL_TOPIC = "/dimsim_eval_turn_control"
CANCEL_ACTIVE_TURN = "cancel_active_turn"


class ControlTransport(Protocol):
    """Transport behavior needed by the one-shot control publisher."""

    def start(self) -> None: ...

    def publish(self, message: Any) -> None: ...

    def stop(self) -> None: ...


def publish_turn_cancellation(
    transport: ControlTransport,
    run_id: str,
) -> None:
    """Publish one run-correlated request and own the transport lifecycle."""
    if not run_id:
        raise ValueError("run_id must not be empty")

    transport.start()
    try:
        transport.publish(
            {
                "type": CANCEL_ACTIVE_TURN,
                "runId": run_id,
            }
        )
    finally:
        transport.stop()


def main() -> None:
    apply_transport_arg(sys.argv)
    parser = argparse.ArgumentParser(
        description="Terminate the currently active DimSim evaluation turn.",
    )
    parser.add_argument("run_id")
    parser.add_argument(
        "--transport",
        choices=("lcm", "zenoh"),
        help=argparse.SUPPRESS,
    )
    args = parser.parse_args()
    publish_turn_cancellation(
        make_transport(EVAL_TURN_CONTROL_TOPIC),
        args.run_id,
    )


if __name__ == "__main__":
    main()
