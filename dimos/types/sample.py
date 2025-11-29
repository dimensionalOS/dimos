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

import builtins
from collections import OrderedDict
from collections.abc import Sequence
from enum import Enum
import json
import logging
from pathlib import Path
from typing import Annotated, Any, Literal, Union, get_origin

from datasets import Dataset  # type: ignore[import-not-found]
from gymnasium import spaces  # type: ignore[import-not-found]
from jsonref import replace_refs  # type: ignore[import-not-found]
from mbodied.data.utils import to_features  # type: ignore[import-not-found]
from mbodied.utils.import_utils import smart_import  # type: ignore[import-not-found]
import numpy as np
from pydantic import BaseModel, ConfigDict, ValidationError
from pydantic.fields import FieldInfo
from pydantic_core import from_json
import torch

Flattenable = Annotated[Literal["dict", "np", "pt", "list"], "Numpy, PyTorch, list, or dict"]


class Sample(BaseModel):
    """A base model class for serializing, recording, and manipulating arbitray data.

    It was designed to be extensible, flexible, yet strongly typed. In addition to
    supporting any json API out of the box, it can be used to represent
    arbitrary action and observation spaces in robotics and integrates seemlessly with H5, Gym, Arrow,
    PyTorch, DSPY, numpy, and HuggingFace.

    Methods:
        schema: Get a simplified json schema of your data.
        to: Convert the Sample instance to a different container type:
            -
        default_value: Get the default value for the Sample instance.
        unflatten: Unflatten a one-dimensional array or dictionary into a Sample instance.
        flatten: Flatten the Sample instance into a one-dimensional array or dictionary.
        space_for: Default Gym space generation for a given value.
        init_from: Initialize a Sample instance from a given value.
        from_space: Generate a Sample instance from a Gym space.
        pack_from: Pack a list of samples into a single sample with lists for attributes.
        unpack: Unpack the packed Sample object into a list of Sample objects or dictionaries.
        dict: Return the Sample object as a dictionary with None values excluded.
        model_field_info: Get the FieldInfo for a given attribute key.
        space: Return the corresponding Gym space for the Sample instance based on its instance attributes.
        random_sample: Generate a random Sample instance based on its instance attributes.

    Examples:
        >>> sample = Sample(x=1, y=2, z={"a": 3, "b": 4}, extra_field=5)
        >>> flat_list = sample.flatten()
        >>> print(flat_list)
        [1, 2, 3, 4, 5]
        >>> schema = sample.schema()
        {'type': 'object', 'properties': {'x': {'type': 'number'}, 'y': {'type': 'number'}, 'z': {'type': 'object', 'properties': {'a': {'type': 'number'}, 'b': {'type': 'number'}}}, 'extra_field': {'type': 'number'}}}
        >>> unflattened_sample = Sample.unflatten(flat_list, schema)
        >>> print(unflattened_sample)
        Sample(x=1, y=2, z={'a': 3, 'b': 4}, extra_field=5)
    """

    __doc__ = "A base model class for serializing, recording, and manipulating arbitray data."

    model_config: ConfigDict = ConfigDict(  # type: ignore[misc]
        use_enum_values=False,
        from_attributes=True,
        validate_assignment=False,
        extra="allow",
        arbitrary_types_allowed=True,
    )

    def __init__(self, datum=None, **data) -> None:  # type: ignore[no-untyped-def]
        """Accepts an arbitrary datum as well as keyword arguments."""
        if datum is not None:
            if isinstance(datum, Sample):
                data.update(datum.dict())
            elif isinstance(datum, dict):
                data.update(datum)
            else:
                data["datum"] = datum
        super().__init__(**data)

    def __hash__(self) -> int:
        """Return a hash of the Sample instance."""
        return hash(tuple(self.dict().values()))

    def __str__(self) -> str:
        """Return a string representation of the Sample instance."""
        return f"{self.__class__.__name__}({', '.join([f'{k}={v}' for k, v in self.dict().items() if v is not None])})"

    def dict(self, exclude_none: bool = True, exclude: set[str] | None = None) -> dict[str, Any]:  # type: ignore[override]
        """Return the Sample object as a dictionary with None values excluded.

        Args:
            exclude_none (bool, optional): Whether to exclude None values. Defaults to True.
            exclude (set[str], optional): Set of attribute names to exclude. Defaults to None.

        Returns:
            Dict[str, Any]: Dictionary representation of the Sample object.
        """
        return self.model_dump(exclude_none=exclude_none, exclude=exclude)

    @classmethod
    def unflatten(cls, one_d_array_or_dict, schema=None) -> "Sample":  # type: ignore[no-untyped-def]
        """Unflatten a one-dimensional array or dictionary into a Sample instance.

        If a dictionary is provided, its keys are ignored.

        Args:
            one_d_array_or_dict: A one-dimensional array or dictionary to unflatten.
            schema: A dictionary representing the JSON schema. Defaults to using the class's schema.

        Returns:
            Sample: The unflattened Sample instance.

        Examples:
            >>> sample = Sample(x=1, y=2, z={"a": 3, "b": 4}, extra_field=5)
            >>> flat_list = sample.flatten()
            >>> print(flat_list)
            [1, 2, 3, 4, 5]
            >>> Sample.unflatten(flat_list, sample.schema())
            Sample(x=1, y=2, z={'a': 3, 'b': 4}, extra_field=5)
        """
        if schema is None:
            schema = cls().schema()

        # Convert input to list if it's not already
        if isinstance(one_d_array_or_dict, dict):
            flat_data = list(one_d_array_or_dict.values())
        else:
            flat_data = list(one_d_array_or_dict)

        def unflatten_recursive(schema_part, index: int = 0):  # type: ignore[no-untyped-def]
            if schema_part["type"] == "object":
                result = {}
                for prop, prop_schema in schema_part["properties"].items():
                    value, index = unflatten_recursive(prop_schema, index)
                    result[prop] = value
                return result, index
            elif schema_part["type"] == "array":
                items = []
                for _ in range(schema_part.get("maxItems", len(flat_data) - index)):
                    value, index = unflatten_recursive(schema_part["items"], index)
                    items.append(value)
                return items, index
            else:  # Assuming it's a primitive type
                return flat_data[index], index + 1

        unflattened_dict, _ = unflatten_recursive(schema)
        return cls(**unflattened_dict)

    def flatten(
        self,
        output_type: Flattenable = "dict",
        non_numerical: Literal["ignore", "forbid", "allow"] = "allow",
    ) -> builtins.dict[str, Any] | np.ndarray | torch.Tensor | list:  # type: ignore[type-arg]
        accumulator = {} if output_type == "dict" else []  # type: ignore[var-annotated]

        def flatten_recursive(obj, path: str = "") -> None:  # type: ignore[no-untyped-def]
            if isinstance(obj, Sample):
                for k, v in obj.dict().items():
                    flatten_recursive(v, path + k + "/")
            elif isinstance(obj, dict):
                for k, v in obj.items():
                    flatten_recursive(v, path + k + "/")
            elif isinstance(obj, list | tuple):
                for i, item in enumerate(obj):
                    flatten_recursive(item, path + str(i) + "/")
            elif hasattr(obj, "__len__") and not isinstance(obj, str):
                flat_list = obj.flatten().tolist()
                if output_type == "dict":
                    # Convert to list for dict storage
                    accumulator[path[:-1]] = flat_list  # type: ignore[index]
                else:
                    accumulator.extend(flat_list)  # type: ignore[attr-defined]
            else:
                if non_numerical == "ignore" and not isinstance(obj, int | float | bool):
                    return
                final_key = path[:-1]  # Remove trailing slash
                if output_type == "dict":
                    accumulator[final_key] = obj  # type: ignore[index]
                else:
                    accumulator.append(obj)  # type: ignore[attr-defined]

        flatten_recursive(self)
        accumulator = accumulator.values() if output_type == "dict" else accumulator  # type: ignore[attr-defined]
        if non_numerical == "forbid" and any(
            not isinstance(v, int | float | bool) for v in accumulator
        ):
            raise ValueError("Non-numerical values found in flattened data.")
        if output_type == "np":
            return np.array(accumulator)
        if output_type == "pt":
            torch = smart_import("torch")
            return torch.tensor(accumulator)  # type: ignore[no-any-return]
        return accumulator  # type: ignore[return-value]

    @staticmethod
    def obj_to_schema(value: Any) -> builtins.dict:  # type: ignore[type-arg]
        """Generates a simplified JSON schema from a dictionary.

        Args:
            value (Any): An object to generate a schema for.

        Returns:
            dict: A simplified JSON schema representing the structure of the dictionary.
        """
        if isinstance(value, dict):
            return {
                "type": "object",
                "properties": {k: Sample.obj_to_schema(v) for k, v in value.items()},
            }
        if isinstance(value, list | tuple | np.ndarray):
            if len(value) > 0:
                return {"type": "array", "items": Sample.obj_to_schema(value[0])}
            return {"type": "array", "items": {}}
        if isinstance(value, str):
            return {"type": "string"}
        if isinstance(value, int | np.integer):
            return {"type": "integer"}
        if isinstance(value, float | np.floating):
            return {"type": "number"}
        if isinstance(value, bool):
            return {"type": "boolean"}
        return {}

    def schema(
        self,
        resolve_refs: bool = True,
        include_descriptions: bool = False,  # type: ignore[override]
    ) -> builtins.dict:  # type: ignore[type-arg]
        """Returns a simplified json schema.

        Removing additionalProperties,
        selecting the first type in anyOf, and converting numpy schema to the desired type.
        Optionally resolves references.

        Args:
            resolve_refs (bool): Whether to resolve references in the schema. Defaults to True.
            include_descriptions (bool): Whether to include descriptions in the schema. Defaults to False.

        Returns:
            dict: A simplified JSON schema.
        """
        schema = self.model_json_schema()
        if "additionalProperties" in schema:
            del schema["additionalProperties"]

        if resolve_refs:
            schema = replace_refs(schema)

        if not include_descriptions and "description" in schema:
            del schema["description"]

        properties = schema.get("properties", {})
        for key, value in self.dict().items():
            if key not in properties:
                properties[key] = Sample.obj_to_schema(value)
            if isinstance(value, Sample):
                properties[key] = value.schema(
                    resolve_refs=resolve_refs, include_descriptions=include_descriptions
                )
            else:
                properties[key] = Sample.obj_to_schema(value)
        return schema

    @classmethod
    def read(cls, data: Any) -> "Sample":
        """Read a Sample instance from a JSON string or dictionary or path.

        Args:
            data (Any): The JSON string or dictionary to read.

        Returns:
            Sample: The read Sample instance.
        """
        if isinstance(data, str):
            try:
                data = cls.model_validate(from_json(data))
            except Exception as e:
                logging.info(f"Error reading data: {e}. Attempting to read as JSON.")
                if isinstance(data, str):
                    if Path(data).exists():
                        if hasattr(cls, "open"):
                            data = cls.open(data)
                        else:
                            data = Path(data).read_text()
                            data = json.loads(data)
                else:
                    data = json.load(data)

        if isinstance(data, dict):
            return cls(**data)
        return cls(data)

    def to(self, container: Any) -> Any:
        """Convert the Sample instance to a different container type.

        Args:
            container (Any): The container type to convert to. Supported types are
                'dict', 'list', 'np', 'pt' (pytorch), 'space' (gym.space),
                'schema', 'json', 'hf' (datasets.Dataset) and any subtype of Sample.

        Returns:
            Any: The converted container.
        """
        if isinstance(container, Sample) and not issubclass(container, Sample):  # type: ignore[arg-type]
            return container(**self.dict())  # type: ignore[operator]
        if isinstance(container, type) and issubclass(container, Sample):
            return container.unflatten(self.flatten())

        if container == "dict":
            return self.dict()
        if container == "list":
            return self.flatten(output_type="list")
        if container == "np":
            return self.flatten(output_type="np")
        if container == "pt":
            return self.flatten(output_type="pt")
        if container == "space":
            return self.space()
        if container == "schema":
            return self.schema()
        if container == "json":
            return self.model_dump_json()
        if container == "hf":
            return Dataset.from_dict(self.dict())
        if container == "features":
            return to_features(self.dict())
        raise ValueError(f"Unsupported container type: {container}")

    @classmethod
    def default_value(cls) -> "Sample":
        """Get the default value for the Sample instance.

        Returns:
            Sample: The default value for the Sample instance.
        """
        return cls()

    @classmethod
    def space_for(
        cls,
        value: Any,
        max_text_length: int = 1000,
        info: Annotated = None,  # type: ignore[valid-type]
    ) -> spaces.Space:
        """Default Gym space generation for a given value.

        Only used for subclasses that do not override the space method.
        """
        if isinstance(value, Enum) or get_origin(value) == Literal:
            return spaces.Discrete(len(value.__args__))
        if isinstance(value, bool):
            return spaces.Discrete(2)
        if isinstance(value, dict | Sample):
            if isinstance(value, Sample):
                value = value.dict()
            return spaces.Dict(
                {k: Sample.space_for(v, max_text_length, info) for k, v in value.items()},
            )
        if isinstance(value, str):
            return spaces.Text(max_length=max_text_length)
        if isinstance(value, int | float | list | tuple | np.ndarray):
            shape = None
            le = None
            ge = None
            dtype = None
            if info is not None:
                shape = info.metadata_lookup.get("shape")
                le = info.metadata_lookup.get("le")
                ge = info.metadata_lookup.get("ge")
                dtype = info.metadata_lookup.get("dtype")
            logging.debug(
                "Generating space for value: %s, shape: %s, le: %s, ge: %s, dtype: %s",
                value,
                shape,
                le,
                ge,
                dtype,
            )
            try:
                value = np.asfarray(value)
                shape = shape or value.shape
                dtype = dtype or value.dtype
                le = le or -np.inf
                ge = ge or np.inf
                return spaces.Box(low=le, high=ge, shape=shape, dtype=dtype)
            except Exception as e:
                logging.info(f"Could not convert value {value} to numpy array: {e}")
                if len(value) > 0 and isinstance(value[0], dict | Sample):
                    return spaces.Tuple(
                        [spaces.Dict(cls.space_for(v, max_text_length, info)) for v in value],
                    )
                return spaces.Tuple(
                    [cls.space_for(value[0], max_text_length, info) for value in value[:1]],
                )
        raise ValueError(f"Unsupported object {value} of type: {type(value)} for space generation")

    @classmethod
    def init_from(cls, d: Any, pack: bool = False) -> "Sample":
        if isinstance(d, spaces.Space):
            return cls.from_space(d)
        if isinstance(d, Union[Sequence, np.ndarray]):  # type: ignore[arg-type]
            if pack:
                return cls.pack_from(d)
            return cls.unflatten(d)
        if isinstance(d, dict):
            try:
                return cls.model_validate(d)
            except ValidationError as e:
                logging.info(f" Unable to validate {d} as {cls} {e}. Attempting to unflatten.")

                try:
                    return cls.unflatten(d)
                except Exception as e:
                    logging.info(f" Unable to unflatten {d} as {cls} {e}. Attempting to read.")
                    return cls.read(d)
        return cls(d)

    @classmethod
    def from_flat_dict(
        cls,
        flat_dict: builtins.dict[str, Any],
        schema: builtins.dict | None = None,  # type: ignore[type-arg]
    ) -> "Sample":
        """Initialize a Sample instance from a flattened dictionary."""
        """
        Reconstructs the original JSON object from a flattened dictionary using the provided schema.

        Args:
            flat_dict (dict): A flattened dictionary with keys like "key1.nestedkey1".
            schema (dict): A dictionary representing the JSON schema.

        Returns:
            dict: The reconstructed JSON object.
        """
        schema = schema or replace_refs(cls.model_json_schema())
        reconstructed = {}  # type: ignore[var-annotated]

        for flat_key, value in flat_dict.items():
            keys = flat_key.split(".")
            current = reconstructed
            for key in keys[:-1]:
                if key not in current:
                    current[key] = {}
                current = current[key]
            current[keys[-1]] = value

        return reconstructed  # type: ignore[return-value]

    @classmethod
    def from_space(cls, space: spaces.Space) -> "Sample":
        """Generate a Sample instance from a Gym space."""
        sampled = space.sample()
        if isinstance(sampled, dict | OrderedDict):
            return cls(**sampled)
        if hasattr(sampled, "__len__") and not isinstance(sampled, str):
            sampled = np.asarray(sampled)
            if len(sampled.shape) > 0 and isinstance(sampled[0], dict | Sample):
                return cls.pack_from(sampled)  # type: ignore[arg-type]
        return cls(sampled)

    @classmethod
    def pack_from(cls, samples: list[Union["Sample", builtins.dict]]) -> "Sample":  # type: ignore[type-arg]
        """Pack a list of samples into a single sample with lists for attributes.

        Args:
            samples (List[Union[Sample, Dict]]): List of samples or dictionaries.

        Returns:
            Sample: Packed sample with lists for attributes.
        """
        if samples is None or len(samples) == 0:
            return cls()

        first_sample = samples[0]
        if isinstance(first_sample, dict):
            attributes = list(first_sample.keys())
        elif hasattr(first_sample, "__dict__"):
            attributes = list(first_sample.__dict__.keys())
        else:
            attributes = ["item" + str(i) for i in range(len(samples))]

        aggregated = {attr: [] for attr in attributes}  # type: ignore[var-annotated]
        for sample in samples:
            for attr in attributes:
                # Handle both Sample instances and dictionaries
                if isinstance(sample, dict):
                    aggregated[attr].append(sample.get(attr, None))
                else:
                    aggregated[attr].append(getattr(sample, attr, None))
        return cls(**aggregated)

    def unpack(self, to_dicts: bool = False) -> list[Union["Sample", builtins.dict]]:  # type: ignore[type-arg]
        """Unpack the packed Sample object into a list of Sample objects or dictionaries."""
        attributes = list(self.model_extra.keys()) + list(self.model_fields.keys())  # type: ignore[union-attr]
        attributes = [attr for attr in attributes if getattr(self, attr) is not None]
        if not attributes or getattr(self, attributes[0]) is None:
            return []

        # Ensure all attributes are lists and have the same length
        list_sizes = {
            len(getattr(self, attr)) for attr in attributes if isinstance(getattr(self, attr), list)
        }
        if len(list_sizes) != 1:
            raise ValueError("Not all attribute lists have the same length.")
        list_size = list_sizes.pop()

        if to_dicts:
            return [{key: getattr(self, key)[i] for key in attributes} for i in range(list_size)]

        return [
            self.__class__(**{key: getattr(self, key)[i] for key in attributes})
            for i in range(list_size)
        ]

    @classmethod
    def default_space(cls) -> spaces.Dict:
        """Return the Gym space for the Sample class based on its class attributes."""
        return cls().space()

    @classmethod
    def default_sample(
        cls, output_type: str = "Sample"
    ) -> Union["Sample", builtins.dict[str, Any]]:
        """Generate a default Sample instance from its class attributes. Useful for padding.

        This is the "no-op" instance and should be overriden as needed.
        """
        if output_type == "Sample":
            return cls()
        return cls().dict()

    def model_field_info(self, key: str) -> FieldInfo:
        """Get the FieldInfo for a given attribute key."""
        if self.model_extra and self.model_extra.get(key) is not None:
            info = FieldInfo(metadata=self.model_extra[key])  # type: ignore[call-arg]
        if self.model_fields.get(key) is not None:
            info = FieldInfo(metadata=self.model_fields[key])  # type: ignore[call-arg]

        if info and hasattr(info, "annotation"):
            return info.annotation  # type: ignore[return-value]
        return None  # type: ignore[return-value]

    def space(self) -> spaces.Dict:
        """Return the corresponding Gym space for the Sample instance based on its instance attributes. Omits None values.

        Override this method in subclasses to customize the space generation.
        """
        space_dict = {}
        for key, value in self.dict().items():
            logging.debug("Generating space for key: '%s', value: %s", key, value)
            info = self.model_field_info(key)
            value = getattr(self, key) if hasattr(self, key) else value
            space_dict[key] = (
                value.space() if isinstance(value, Sample) else self.space_for(value, info=info)
            )
        return spaces.Dict(space_dict)

    def random_sample(self) -> "Sample":
        """Generate a random Sample instance based on its instance attributes. Omits None values.

        Override this method in subclasses to customize the sample generation.
        """
        return self.__class__.model_validate(self.space().sample())


if __name__ == "__main__":
    sample = Sample(x=1, y=2, z={"a": 3, "b": 4}, extra_field=5)
