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

"""Module for manipulation history tracking and search."""

from dataclasses import dataclass, field
from datetime import datetime
import json
import os
import pickle
import time
from typing import Any

from dimos.types.manipulation import (
    ManipulationTask,
)
from dimos.utils.logging_config import setup_logger

logger = setup_logger()


@dataclass
class ManipulationHistoryEntry:
    """An entry in the manipulation history.

    Attributes:
        task: The manipulation task executed
        timestamp: When the manipulation was performed
        result: Result of the manipulation (success/failure)
        manipulation_response: Response from the motion planner/manipulation executor
    """

    task: ManipulationTask
    timestamp: float = field(default_factory=time.time)
    result: dict[str, Any] = field(default_factory=dict)
    manipulation_response: str | None = (
        None  # Any elaborative response from the motion planner / manipulation executor
    )

    def __str__(self) -> str:
        status = self.result.get("status", "unknown")
        return f"ManipulationHistoryEntry(task='{self.task.description}', status={status}, time={datetime.fromtimestamp(self.timestamp).strftime('%H:%M:%S')})"


class ManipulationHistory:
    """A simplified, dictionary-based storage for manipulation history.

    This class provides an efficient way to store and query manipulation tasks,
    focusing on quick lookups and flexible search capabilities.
    """

    def __init__(self, output_dir: str | None = None, new_memory: bool = False) -> None:
        """Initialize a new manipulation history.

        Args:
            output_dir: Directory to save history to
            new_memory: If True, creates a new memory instead of loading existing one
        """
        self._history: list[ManipulationHistoryEntry] = []
        self._output_dir = output_dir

        if output_dir and not new_memory:
            self.load_from_dir(output_dir)
        elif output_dir:
            os.makedirs(output_dir, exist_ok=True)
            logger.info(f"Created new manipulation history at {output_dir}")

    def __len__(self) -> int:
        """Return the number of entries in the history."""
        return len(self._history)

    def __str__(self) -> str:
        """Return a string representation of the history."""
        if not self._history:
            return "ManipulationHistory(empty)"

        return (
            f"ManipulationHistory(entries={len(self._history)}, "
            f"time_range={datetime.fromtimestamp(self._history[0].timestamp).strftime('%Y-%m-%d %H:%M:%S')} to "
            f"{datetime.fromtimestamp(self._history[-1].timestamp).strftime('%Y-%m-%d %H:%M:%S')})"
        )

    def clear(self) -> None:
        """Clear all entries from the history."""
        self._history.clear()
        logger.info("Cleared manipulation history")

        if self._output_dir:
            self.save_history()

    def add_entry(self, entry: ManipulationHistoryEntry) -> None:
        """Add an entry to the history.

        Args:
            entry: The entry to add
        """
        self._history.append(entry)
        self._history.sort(key=lambda e: e.timestamp)

        if self._output_dir:
            self.save_history()

    def save_history(self) -> None:
        """Save the history to the output directory."""
        if not self._output_dir:
            logger.warning("Cannot save history: no output directory specified")
            return

        os.makedirs(self._output_dir, exist_ok=True)
        history_path = os.path.join(self._output_dir, "manipulation_history.pickle")

        with open(history_path, "wb") as f:
            pickle.dump(self._history, f)

        logger.info(f"Saved manipulation history to {history_path}")

        # Also save a JSON representation for easier inspection
        json_path = os.path.join(self._output_dir, "manipulation_history.json")
        try:
            history_data = [
                {
                    "task": {
                        "description": entry.task.description,
                        "target_object": entry.task.target_object,
                        "target_point": entry.task.target_point,
                        "timestamp": entry.task.timestamp,
                        "task_id": entry.task.task_id,
                        "metadata": entry.task.metadata,
                    },
                    "result": entry.result,
                    "timestamp": entry.timestamp,
                    "manipulation_response": entry.manipulation_response,
                }
                for entry in self._history
            ]

            with open(json_path, "w") as f:
                json.dump(history_data, f, indent=2)

            logger.info(f"Saved JSON representation to {json_path}")
        except Exception as e:
            logger.error(f"Failed to save JSON representation: {e}")

    def load_from_dir(self, directory: str) -> None:
        """Load history from the specified directory.

        Args:
            directory: Directory to load history from
        """
        history_path = os.path.join(directory, "manipulation_history.pickle")

        if not os.path.exists(history_path):
            logger.warning(f"No history found at {history_path}")
            return

        try:
            with open(history_path, "rb") as f:
                self._history = pickle.load(f)

            logger.info(
                f"Loaded manipulation history from {history_path} with {len(self._history)} entries"
            )
        except Exception as e:
            logger.error(f"Failed to load history: {e}")

    def get_all_entries(self) -> list[ManipulationHistoryEntry]:
        """Get all entries in chronological order.

        Returns:
            List of all manipulation history entries
        """
        return self._history.copy()

    def get_entry_by_index(self, index: int) -> ManipulationHistoryEntry | None:
        """Get an entry by its index.

        Args:
            index: Index of the entry to retrieve

        Returns:
            The entry at the specified index or None if index is out of bounds
        """
        if 0 <= index < len(self._history):
            return self._history[index]
        return None

    def get_entries_by_timerange(
        self, start_time: float, end_time: float
    ) -> list[ManipulationHistoryEntry]:
        """Get entries within a specific time range.

        Args:
            start_time: Start time (UNIX timestamp)
            end_time: End time (UNIX timestamp)

        Returns:
            List of entries within the specified time range
        """
        return [entry for entry in self._history if start_time <= entry.timestamp <= end_time]

    def get_entries_by_object(self, object_name: str) -> list[ManipulationHistoryEntry]:
        """Get entries related to a specific object.

        Args:
            object_name: Name of the object to search for

        Returns:
            List of entries related to the specified object
        """
        return [entry for entry in self._history if entry.task.target_object == object_name]

    def create_task_entry(
        self,
        task: ManipulationTask,
        result: dict[str, Any] | None = None,
        agent_response: str | None = None,
    ) -> ManipulationHistoryEntry:
        """Create a new manipulation history entry.

        Args:
            task: The manipulation task
            result: Result of the manipulation
            agent_response: Response from the agent about this manipulation

        Returns:
            The created history entry
        """
        entry = ManipulationHistoryEntry(
            task=task, result=result or {}, manipulation_response=agent_response
        )
        self.add_entry(entry)
        return entry

    def search(self, **kwargs) -> list[ManipulationHistoryEntry]:  # type: ignore[no-untyped-def]
        """Flexible search method that can search by any field in ManipulationHistoryEntry using dot notation.

        This method supports dot notation to access nested fields. String values automatically use
        substring matching (contains), while all other types use exact matching.

        Examples:
        # Time-based searches:
        - search(**{"task.metadata.timestamp": ('>', start_time)}) - entries after start_time
        - search(**{"task.metadata.timestamp": ('>=', time - 1800)}) - entries in last 30 mins

        # Constraint searches:
        - search(**{"task.constraints.*.reference_point.x": 2.5}) - tasks with x=2.5 reference point
        - search(**{"task.constraints.*.end_angle.x": 90}) - tasks with 90-degree x rotation
        - search(**{"task.constraints.*.lock_x": True}) - tasks with x-axis translation locked

        # Object and result searches:
        - search(**{"task.metadata.objects.*.label": "cup"}) - tasks involving cups
        - search(**{"result.status": "success"}) - successful tasks
        - search(**{"result.error": "Collision"}) - tasks that had collisions

        Args:
            **kwargs: Key-value pairs for searching using dot notation for field paths.

        Returns:
            List of matching entries
        """
        if not kwargs:
            return self._history.copy()

        results = self._history.copy()

        for key, value in kwargs.items():
            # For all searches, automatically determine if we should use contains for strings
            results = [e for e in results if self._check_field_match(e, key, value)]

        return results

    def _check_field_match(self, entry, field_path, value) -> bool:  # type: ignore[no-untyped-def]
        """Check if a field matches the value, with special handling for strings, collections and comparisons.

        For string values, we automatically use substring matching (contains).
        For collections (returned by * path), we check if any element matches.
        For numeric values (like timestamps), supports >, <, >= and <= comparisons.
        For all other types, we use exact matching.

        Args:
            entry: The entry to check
            field_path: Dot-separated path to the field
            value: Value to match against. For comparisons, use tuples like:
                  ('>',  timestamp) - greater than
                  ('<',  timestamp) - less than
                  ('>=', timestamp) - greater or equal
                  ('<=', timestamp) - less or equal

        Returns:
            True if the field matches the value, False otherwise
        """
        try:
            field_value = self._get_value_by_path(entry, field_path)  # type: ignore[no-untyped-call]

            # Handle comparison operators for timestamps and numbers
            if isinstance(value, tuple) and len(value) == 2:
                op, compare_value = value
                if op == ">":
                    return field_value > compare_value  # type: ignore[no-any-return]
                elif op == "<":
                    return field_value < compare_value  # type: ignore[no-any-return]
                elif op == ">=":
                    return field_value >= compare_value  # type: ignore[no-any-return]
                elif op == "<=":
                    return field_value <= compare_value  # type: ignore[no-any-return]

            # Handle lists (from collection searches)
            if isinstance(field_value, list):
                for item in field_value:
                    # String values use contains matching
                    if isinstance(item, str) and isinstance(value, str):
                        if value in item:
                            return True
                    # All other types use exact matching
                    elif item == value:
                        return True
                return False

            # String values use contains matching
            elif isinstance(field_value, str) and isinstance(value, str):
                return value in field_value
            # All other types use exact matching
            else:
                return field_value == value  # type: ignore[no-any-return]

        except (AttributeError, KeyError):
            return False

    def _get_value_by_path(self, obj, path):  # type: ignore[no-untyped-def]
        """Get a value from an object using a dot-separated path.

        This method handles three special cases:
        1. Regular attribute access (obj.attr)
        2. Dictionary key access (dict[key])
        3. Collection search (dict.*.attr) - when * is used, it searches all values in the collection

        Args:
            obj: Object to get value from
            path: Dot-separated path to the field (e.g., "task.metadata.robot")

        Returns:
            Value at the specified path or list of values for collection searches

        Raises:
            AttributeError: If an attribute in the path doesn't exist
            KeyError: If a dictionary key in the path doesn't exist
        """
        current = obj
        parts = path.split(".")

        for i, part in enumerate(parts):
            # Collection search (*.attr) - search across all items in a collection
            if part == "*":
                # Get remaining path parts
                remaining_path = ".".join(parts[i + 1 :])

                # Handle different collection types
                if isinstance(current, dict):
                    items = current.values()
                    if not remaining_path:  # If * is the last part, return all values
                        return list(items)
                elif isinstance(current, list):
                    items = current  # type: ignore[assignment]
                    if not remaining_path:  # If * is the last part, return all items
                        return items
                else:  # Not a collection
                    raise AttributeError(
                        f"Cannot use wildcard on non-collection type: {type(current)}"
                    )

                # Apply remaining path to each item in the collection
                results = []
                for item in items:
                    try:
                        # Recursively get values from each item
                        value = self._get_value_by_path(item, remaining_path)  # type: ignore[no-untyped-call]
                        if isinstance(value, list):  # Flatten nested lists
                            results.extend(value)
                        else:
                            results.append(value)
                    except (AttributeError, KeyError):
                        # Skip items that don't have the attribute
                        pass
                return results

            # Regular attribute/key access
            elif isinstance(current, dict):
                current = current[part]
            else:
                current = getattr(current, part)

        return current
