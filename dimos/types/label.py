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

from typing import Dict, Any

class LabelType:
    def __init__(self, labels: Dict[str, Any], metadata: Any = None):
        """
        Initializes a standardized label type.

        Args:
            labels (Dict[str, Any]): A dictionary of labels with descriptions.
            metadata (Any, optional): Additional metadata related to the labels.
        """
        self.labels = labels
        self.metadata = metadata 

    def get_label_descriptions(self):
        """Return a list of label descriptions."""
        return [desc['description'] for desc in self.labels.values()]

    def save_to_json(self, filepath: str):
        """Save the labels to a JSON file."""
        import json
        with open(filepath, 'w') as f:
            json.dump(self.labels, f, indent=4) 