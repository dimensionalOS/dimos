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

"""Unit tests for receipt-bound manipulation plan execution."""

import threading
from unittest.mock import MagicMock

from dimos.manipulation._test_manipulation_helpers import make_module
from dimos.manipulation.manipulation_module import ManipulationState
from dimos.manipulation.planning.spec.enums import PlanningStatus
from dimos.manipulation.planning.spec.models import GeneratedPlan, PlanningResult
from dimos.msgs.sensor_msgs.JointState import JointState


class TestPlanReceipts:
    """Test receipt-bound execution cannot race with plan replacement."""

    def test_replaced_plan_receipt_cannot_dispatch_replacement(self):
        module = make_module()
        module._world_monitor = MagicMock()
        plan_a = GeneratedPlan(group_ids=("arm/manipulator",), path=[JointState(), JointState()])
        plan_b = GeneratedPlan(group_ids=("arm/manipulator",), path=[JointState(), JointState()])
        module._last_plan = plan_a
        module._last_plan_receipt = "receipt-a"
        module._state = ManipulationState.COMPLETED

        # Beginning plan B invalidates plan A's receipt before B is stored.
        assert module._begin_group_planning() is not None
        assert module._last_plan_receipt is None
        module._last_plan = plan_b
        module._last_plan_receipt = "receipt-b"
        module._state = ManipulationState.COMPLETED
        module._execute_plan = MagicMock(return_value=True)

        assert module.execute_plan_receipt("receipt-a") is False
        module._execute_plan.assert_not_called()

    def test_current_receipt_dispatches_its_exact_plan(self):
        module = make_module()
        plan = GeneratedPlan(group_ids=("arm/manipulator",), path=[JointState(), JointState()])
        module._last_plan = plan
        module._last_plan_receipt = "current-receipt"
        module._state = ManipulationState.COMPLETED
        module._execute_plan = MagicMock(return_value=True)

        assert module.execute_plan_receipt("current-receipt") is True
        module._execute_plan.assert_called_once_with(plan, execution_reserved=True)
        assert module._last_plan_receipt is None

    def test_concurrent_receipt_execution_reserves_once(self):
        module = make_module()
        plan = GeneratedPlan(group_ids=("arm/manipulator",), path=[JointState(), JointState()])
        module._last_plan = plan
        module._last_plan_receipt = "once"
        module._state = ManipulationState.COMPLETED
        dispatch_started = threading.Event()
        allow_dispatch_return = threading.Event()

        def dispatch(*_args, **_kwargs):
            dispatch_started.set()
            assert allow_dispatch_return.wait(timeout=1.0)
            return True

        module._execute_plan = MagicMock(side_effect=dispatch)
        first = threading.Thread(target=module.execute_plan_receipt, args=("once",))
        first.start()
        assert dispatch_started.wait(timeout=1.0)

        assert module.execute_plan_receipt("once") is False
        allow_dispatch_return.set()
        first.join(timeout=1.0)

        module._execute_plan.assert_called_once_with(plan, execution_reserved=True)

    def test_legacy_execute_consumes_stored_plan_receipt(self):
        module = make_module()
        plan = GeneratedPlan(group_ids=("arm/manipulator",), path=[JointState(), JointState()])
        module._last_plan = plan
        module._last_plan_receipt = "receipt-a"
        module._state = ManipulationState.COMPLETED
        module._execute_plan = MagicMock(return_value=True)

        assert module.execute() is True
        assert module._last_plan_receipt is None
        assert module.execute_plan_receipt("receipt-a") is False
        module._execute_plan.assert_called_once_with(plan, robot_name=None, execution_reserved=True)

    def test_receipt_execution_prevents_legacy_redispatch(self):
        module = make_module()
        plan = GeneratedPlan(group_ids=("arm/manipulator",), path=[JointState(), JointState()])
        module._last_plan = plan
        module._last_plan_receipt = "receipt-a"
        module._state = ManipulationState.COMPLETED
        module._execute_plan = MagicMock(return_value=True)

        assert module.execute_plan_receipt("receipt-a") is True
        # A completed execution leaves the plan available for status and preview,
        # but it must never be dispatched through the legacy API again.
        module._state = ManipulationState.COMPLETED
        assert module.execute() is False
        module._execute_plan.assert_called_once_with(plan, execution_reserved=True)

    def test_cancel_invalidates_receipt(self):
        module = make_module()
        module._state = ManipulationState.EXECUTING
        module._last_plan = GeneratedPlan(
            group_ids=("arm/manipulator",), path=[JointState(), JointState()]
        )
        module._last_plan_receipt = "receipt"

        assert module.cancel() is True
        assert module._last_plan_receipt is None

    def test_stale_projection_failure_does_not_clear_newer_plan(self):
        module = make_module()
        module._world_monitor = MagicMock()
        module._world_monitor.world = MagicMock()
        module._planner = MagicMock()
        module._planner.plan_selected_joint_path.return_value = PlanningResult(
            status=PlanningStatus.SUCCESS,
            path=[JointState(), JointState()],
        )
        projection_started = threading.Event()
        release_projection = threading.Event()

        def stale_projection(_plan):
            projection_started.set()
            assert release_projection.wait(timeout=1.0)
            return None

        module._project_plan_to_robot_paths = MagicMock(side_effect=stale_projection)
        module._state = ManipulationState.PLANNING
        module._planning_epoch = 1
        stale = threading.Thread(
            target=module._plan_selected_path,
            args=(("arm/manipulator",), JointState(), JointState(), 1),
        )
        stale.start()
        assert projection_started.wait(timeout=1.0)

        assert module.cancel() is True
        newer_plan = GeneratedPlan(
            group_ids=("arm/manipulator",), path=[JointState(), JointState()]
        )
        with module._lock:
            module._planning_epoch += 1
            module._state = ManipulationState.COMPLETED
            module._last_plan = newer_plan
            module._last_plan_receipt = "receipt-b"
        release_projection.set()
        stale.join(timeout=1.0)

        assert not stale.is_alive()
        assert module._state == ManipulationState.COMPLETED
        assert module._last_plan is newer_plan
        assert module._last_plan_receipt == "receipt-b"
