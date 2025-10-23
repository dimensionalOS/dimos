# Copyright 2025 Dimensional Inc.
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

# Copyright 2025 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     [http://www.apache.org/licenses/LICENSE-2.0](http://www.apache.org/licenses/LICENSE-2.0)
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
import tempfile
import time

import pytest

from dimos.manipulation.manipulation_history import ManipulationHistory, ManipulationHistoryEntry
from dimos.types.manipulation import (
    ForceConstraint,
    ManipulationTask,
    RotationConstraint,
    TranslationConstraint,
)
from dimos.types.vector import Vector


@pytest.fixture
def sample_task():
    """Create a sample manipulation task for testing."""
    return ManipulationTask(
        description="Pick up the cup",
        target_object="cup",
        target_point=(100, 200),
        task_id="task1",
        metadata={
            "timestamp": time.time(),
            "objects": {
                "cup1": {
                    "object_id": 1,
                    "label": "cup",
                    "confidence": 0.95,
                    "position": {"x": 1.5, "y": 2.0, "z": 0.5},
                },
                "table1": {
                    "object_id": 2,
                    "label": "table",
                    "confidence": 0.98,
                    "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                },
            },
        },
    )


@pytest.fixture
def sample_task_with_constraints():
    """Create a sample manipulation task with constraints for testing."""
    task = ManipulationTask(
        description="Rotate the bottle",
        target_object="bottle",
        target_point=(150, 250),
        task_id="task2",
        metadata={
            "timestamp": time.time(),
            "objects": {
                "bottle1": {
                    "object_id": 3,
                    "label": "bottle",
                    "confidence": 0.92,
                    "position": {"x": 2.5, "y": 1.0, "z": 0.3},
                }
            },
        },
    )

    # Add rich translation constraint
    translation_constraint = TranslationConstraint(
        translation_axis="y",
        reference_point=Vector(2.5, 1.0, 0.3),
        bounds_min=Vector(2.0, 0.5, 0.3),
        bounds_max=Vector(3.0, 1.5, 0.3),
        target_point=Vector(2.7, 1.2, 0.3),
        description="Constrained translation along Y-axis only",
    )
    task.add_constraint(translation_constraint)

    # Add rich rotation constraint
    rotation_constraint = RotationConstraint(
        rotation_axis="roll",
        start_angle=Vector(0, 0, 0),
        end_angle=Vector(90, 0, 0),
        pivot_point=Vector(2.5, 1.0, 0.3),
        secondary_pivot_point=Vector(2.5, 1.0, 0.5),
        description="Constrained rotation around X-axis (roll only)",
    )
    task.add_constraint(rotation_constraint)

    # Add force constraint
    force_constraint = ForceConstraint(
        min_force=2.0,
        max_force=5.0,
        force_direction=Vector(0, 0, -1),
        description="Apply moderate downward force during manipulation",
    )
    task.add_constraint(force_constraint)

    return task


@pytest.fixture
def temp_output_dir():
    """Create a temporary directory for testing history saving/loading."""
    with tempfile.TemporaryDirectory() as temp_dir:
        yield temp_dir


@pytest.fixture
def populated_history(sample_task, sample_task_with_constraints):
    """Create a populated history with multiple entries for testing."""
    history = ManipulationHistory()

    # Add first entry
    entry1 = ManipulationHistoryEntry(
        task=sample_task,
        result={"status": "success", "execution_time": 2.5},
        manipulation_response="Successfully picked up the cup",
    )
    history.add_entry(entry1)

    # Add second entry
    entry2 = ManipulationHistoryEntry(
        task=sample_task_with_constraints,
        result={"status": "failure", "error": "Collision detected"},
        manipulation_response="Failed to rotate the bottle due to collision",
    )
    history.add_entry(entry2)

    return history


def test_manipulation_history_init() -> None:
    """Test initialization of ManipulationHistory."""
    # Default initialization
    history = ManipulationHistory()
    assert len(history) == 0
    assert str(history) == "ManipulationHistory(empty)"

    # With output directory
    with tempfile.TemporaryDirectory() as temp_dir:
        history = ManipulationHistory(output_dir=temp_dir, new_memory=True)
        assert len(history) == 0
        assert os.path.exists(temp_dir)


def test_manipulation_history_add_entry(sample_task) -> None:
    """Test adding entries to ManipulationHistory."""
    history = ManipulationHistory()

    # Create and add entry
    entry = ManipulationHistoryEntry(
        task=sample_task, result={"status": "success"}, manipulation_response="Task completed"
    )
    history.add_entry(entry)

    assert len(history) == 1
    assert history.get_entry_by_index(0) == entry


def test_manipulation_history_create_task_entry(sample_task) -> None:
    """Test creating a task entry directly."""
    history = ManipulationHistory()

    entry = history.create_task_entry(
        task=sample_task, result={"status": "success"}, agent_response="Task completed"
    )

    assert len(history) == 1
    assert entry.task == sample_task
    assert entry.result["status"] == "success"
    assert entry.manipulation_response == "Task completed"


def test_manipulation_history_save_load(temp_output_dir, sample_task) -> None:
    """Test saving and loading history from disk."""
    # Create history and add entry
    history = ManipulationHistory(output_dir=temp_output_dir)
    history.create_task_entry(
        task=sample_task, result={"status": "success"}, agent_response="Task completed"
    )

    # Check that files were created
    pickle_path = os.path.join(temp_output_dir, "manipulation_history.pickle")
    json_path = os.path.join(temp_output_dir, "manipulation_history.json")
    assert os.path.exists(pickle_path)
    assert os.path.exists(json_path)

    # Create new history that loads from the saved files
    loaded_history = ManipulationHistory(output_dir=temp_output_dir)
    assert len(loaded_history) == 1
    assert loaded_history.get_entry_by_index(0).task.description == sample_task.description


def test_manipulation_history_clear(populated_history) -> None:
    """Test clearing the history."""
    assert len(populated_history) > 0

    populated_history.clear()
    assert len(populated_history) == 0
    assert str(populated_history) == "ManipulationHistory(empty)"


def test_manipulation_history_get_methods(populated_history) -> None:
    """Test various getter methods of ManipulationHistory."""
    # get_all_entries
    entries = populated_history.get_all_entries()
    assert len(entries) == 2

    # get_entry_by_index
    entry = populated_history.get_entry_by_index(0)
    assert entry.task.task_id == "task1"

    # Out of bounds index
    assert populated_history.get_entry_by_index(100) is None

    # get_entries_by_timerange
    start_time = time.time() - 3600  # 1 hour ago
    end_time = time.time() + 3600  # 1 hour from now
    entries = populated_history.get_entries_by_timerange(start_time, end_time)
    assert len(entries) == 2

    # get_entries_by_object
    cup_entries = populated_history.get_entries_by_object("cup")
    assert len(cup_entries) == 1
    assert cup_entries[0].task.task_id == "task1"

    bottle_entries = populated_history.get_entries_by_object("bottle")
    assert len(bottle_entries) == 1
    assert bottle_entries[0].task.task_id == "task2"


def test_manipulation_history_search_basic(populated_history) -> None:
    """Test basic search functionality."""
    # Search by exact match on top-level fields
    results = populated_history.search(timestamp=populated_history.get_entry_by_index(0).timestamp)
    assert len(results) == 1

    # Search by task fields
    results = populated_history.search(**{"task.task_id": "task1"})
    assert len(results) == 1
    assert results[0].task.target_object == "cup"

    # Search by result fields
    results = populated_history.search(**{"result.status": "success"})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Search by manipulation_response (substring match for strings)
    results = populated_history.search(manipulation_response="picked up")
    assert len(results) == 1
    assert results[0].task.task_id == "task1"


def test_manipulation_history_search_nested(populated_history) -> None:
    """Test search with nested field paths."""
    # Search by nested metadata fields
    results = populated_history.search(
        **{
            "task.metadata.timestamp": populated_history.get_entry_by_index(0).task.metadata[
                "timestamp"
            ]
        }
    )
    assert len(results) == 1

    # Search by nested object fields
    results = populated_history.search(**{"task.metadata.objects.cup1.label": "cup"})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Search by position values
    results = populated_history.search(**{"task.metadata.objects.cup1.position.x": 1.5})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"


def test_manipulation_history_search_wildcards(populated_history) -> None:
    """Test search with wildcard patterns."""
    # Search for any object with label "cup"
    results = populated_history.search(**{"task.metadata.objects.*.label": "cup"})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Search for any object with confidence > 0.95
    results = populated_history.search(**{"task.metadata.objects.*.confidence": 0.98})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Search for any object position with x=2.5
    results = populated_history.search(**{"task.metadata.objects.*.position.x": 2.5})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"


def test_manipulation_history_search_constraints(populated_history) -> None:
    """Test search by constraint properties."""
    # Find entries with any TranslationConstraint with y-axis
    results = populated_history.search(**{"task.constraints.*.translation_axis": "y"})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"

    # Find entries with any RotationConstraint with roll axis
    results = populated_history.search(**{"task.constraints.*.rotation_axis": "roll"})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"


def test_manipulation_history_search_string_contains(populated_history) -> None:
    """Test string contains searching."""
    # Basic string contains
    results = populated_history.search(**{"task.description": "Pick"})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Nested string contains
    results = populated_history.search(manipulation_response="collision")
    assert len(results) == 1
    assert results[0].task.task_id == "task2"


def test_manipulation_history_search_multiple_criteria(populated_history) -> None:
    """Test search with multiple criteria."""
    # Multiple criteria - all must match
    results = populated_history.search(**{"task.target_object": "cup", "result.status": "success"})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Multiple criteria with no matches
    results = populated_history.search(**{"task.target_object": "cup", "result.status": "failure"})
    assert len(results) == 0

    # Combination of direct and wildcard paths
    results = populated_history.search(
        **{"task.target_object": "bottle", "task.metadata.objects.*.position.z": 0.3}
    )
    assert len(results) == 1
    assert results[0].task.task_id == "task2"


def test_manipulation_history_search_nonexistent_fields(populated_history) -> None:
    """Test search with fields that don't exist."""
    # Search by nonexistent field
    results = populated_history.search(nonexistent_field="value")
    assert len(results) == 0

    # Search by nonexistent nested field
    results = populated_history.search(**{"task.nonexistent_field": "value"})
    assert len(results) == 0

    # Search by nonexistent object
    results = populated_history.search(**{"task.metadata.objects.nonexistent_object": "value"})
    assert len(results) == 0


def test_manipulation_history_search_timestamp_ranges(populated_history) -> None:
    """Test searching by timestamp ranges."""
    # Get reference timestamps
    entry1_time = populated_history.get_entry_by_index(0).task.metadata["timestamp"]
    entry2_time = populated_history.get_entry_by_index(1).task.metadata["timestamp"]
    mid_time = (entry1_time + entry2_time) / 2

    # Search for timestamps before second entry
    results = populated_history.search(**{"task.metadata.timestamp": ("<", entry2_time)})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Search for timestamps after first entry
    results = populated_history.search(**{"task.metadata.timestamp": (">", entry1_time)})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"

    # Search within a time window using >= and <=
    results = populated_history.search(**{"task.metadata.timestamp": (">=", mid_time - 1800)})
    assert len(results) == 2
    assert results[0].task.task_id == "task1"
    assert results[1].task.task_id == "task2"


def test_manipulation_history_search_vector_fields(populated_history) -> None:
    """Test searching by vector components in constraints."""
    # Search by reference point components
    results = populated_history.search(**{"task.constraints.*.reference_point.x": 2.5})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"

    # Search by target point components
    results = populated_history.search(**{"task.constraints.*.target_point.z": 0.3})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"

    # Search by rotation angles
    results = populated_history.search(**{"task.constraints.*.end_angle.x": 90})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"


def test_manipulation_history_search_execution_details(populated_history) -> None:
    """Test searching by execution time and error patterns."""
    # Search by execution time
    results = populated_history.search(**{"result.execution_time": 2.5})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Search by error message pattern
    results = populated_history.search(**{"result.error": "Collision"})
    assert len(results) == 1
    assert results[0].task.task_id == "task2"

    # Search by status
    results = populated_history.search(**{"result.status": "success"})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"


def test_manipulation_history_search_multiple_criteria(populated_history) -> None:
    """Test search with multiple criteria."""
    # Multiple criteria - all must match
    results = populated_history.search(**{"task.target_object": "cup", "result.status": "success"})
    assert len(results) == 1
    assert results[0].task.task_id == "task1"

    # Multiple criteria with no matches
    results = populated_history.search(**{"task.target_object": "cup", "result.status": "failure"})
    assert len(results) == 0

    # Combination of direct and wildcard paths
    results = populated_history.search(
        **{"task.target_object": "bottle", "task.metadata.objects.*.position.z": 0.3}
    )
    assert len(results) == 1
    assert results[0].task.task_id == "task2"
