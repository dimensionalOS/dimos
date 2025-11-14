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