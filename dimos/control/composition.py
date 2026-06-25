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

"""Internal composition helpers for coordinator-facing control tasks."""

from __future__ import annotations

from collections.abc import Sequence
from typing import TYPE_CHECKING

from dimos.control.task import (
    BaseControlTask,
    ControlTask,
    CoordinatorState,
    JointCommandOutput,
    ReferenceTransformTask,
    ResourceClaim,
)

if TYPE_CHECKING:
    from dimos.control.components import JointName


class ComposedControlTask(BaseControlTask):
    """A single coordinator-facing task backed by a linear internal pipeline.

    The coordinator still sees one task with one claim and one final
    ``JointCommandOutput``. Internally, ``source.compute()`` produces the
    nominal reference and each transform consumes the previous output.
    """

    def __init__(
        self,
        name: str,
        source: ControlTask,
        transforms: Sequence[ReferenceTransformTask],
    ) -> None:
        if not name:
            raise ValueError("ComposedControlTask requires a non-empty name")
        if not transforms:
            raise ValueError(f"ComposedControlTask '{name}' requires at least one transform")

        self._name = name
        self._source = source
        self._transforms = list(transforms)
        self._claim = self._validate_pipeline(name, source, self._transforms)

    @property
    def name(self) -> str:
        """Unique task identifier exposed to the coordinator."""
        return self._name

    @property
    def source(self) -> ControlTask:
        """Source task used by the internal pipeline."""
        return self._source

    @property
    def transforms(self) -> tuple[ReferenceTransformTask, ...]:
        """Ordered transforms used by the internal pipeline."""
        return tuple(self._transforms)

    def claim(self) -> ResourceClaim:
        """Declare the single external claim owned by this composed task.

        v1 composition assumes source/transform claims are static after
        construction; the validated claim is cached to keep arbitration cheap.
        """
        return self._claim

    def is_active(self) -> bool:
        """A composed pipeline is active when its source is active."""
        return self._source.is_active()

    def compute(self, state: CoordinatorState) -> JointCommandOutput | None:
        """Run source then transforms in order."""
        output = self._source.compute(state)
        if output is None:
            return None

        for transform in self._transforms:
            output = transform.compute_from_reference(state, output)
            if output is None:
                return None
        return output

    def on_preempted(self, by_task: str, joints: frozenset[JointName]) -> None:
        """Forward preemption and reset transform state when available."""
        self._source.on_preempted(by_task, joints)
        for transform in self._transforms:
            transform.on_preempted(by_task, joints)
            reset = getattr(transform, "reset", None)
            if callable(reset):
                reset()

    @staticmethod
    def _validate_pipeline(
        name: str,
        source: ControlTask,
        transforms: Sequence[ReferenceTransformTask],
    ) -> ResourceClaim:
        source_claim = source.claim()
        if not source_claim.joints:
            raise ValueError(f"ComposedControlTask '{name}' source must claim at least one joint")

        for transform in transforms:
            transform_claim = transform.claim()
            if transform_claim.joints != source_claim.joints:
                raise ValueError(
                    f"ComposedControlTask '{name}' transform '{transform.name}' joints "
                    f"{sorted(transform_claim.joints)} do not match source joints "
                    f"{sorted(source_claim.joints)}"
                )
            if transform_claim.mode != source_claim.mode:
                raise ValueError(
                    f"ComposedControlTask '{name}' transform '{transform.name}' mode "
                    f"{transform_claim.mode} does not match source mode {source_claim.mode}"
                )

        return source_claim
