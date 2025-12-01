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

"""Tests for RPC exception serialization utilities."""

from dimos.protocol.rpc.rpc_utils import (
    RemoteError,
    deserialize_exception,
    serialize_exception,
)


def test_exception_builtin_serialization():
    """Test serialization and deserialization of exceptions."""

    # Test with a builtin exception
    try:
        raise ValueError("test error", 42)
    except ValueError as e:
        serialized = serialize_exception(e)

        # Check serialized format
        assert serialized["type_name"] == "ValueError"
        assert serialized["type_module"] == "builtins"
        assert serialized["args"] == ("test error", 42)
        assert "Traceback" in serialized["traceback"]
        assert "test error" in serialized["traceback"]

        # Deserialize and check we get a real ValueError back
        deserialized = deserialize_exception(serialized)
        assert isinstance(deserialized, ValueError)
        assert deserialized.args == ("test error", 42)
        # Check that remote traceback is attached as cause
        assert isinstance(deserialized.__cause__, RemoteError)
        assert "test error" in deserialized.__cause__.remote_traceback


def test_exception_custom_serialization():
    # Test with a custom exception
    class CustomError(Exception):
        pass

    try:
        raise CustomError("custom message")
    except CustomError as e:
        serialized = serialize_exception(e)

        # Check serialized format
        assert serialized["type_name"] == "CustomError"
        # Module name varies when running under pytest vs directly
        assert serialized["type_module"] in ("__main__", "dimos.protocol.rpc.test_rpc_utils")
        assert serialized["args"] == ("custom message",)

        # Deserialize - should get RemoteError since it's not builtin
        deserialized = deserialize_exception(serialized)
        assert isinstance(deserialized, RemoteError)
        assert "CustomError" in deserialized.remote_type
        assert "custom message" in str(deserialized)
        assert "custom message" in deserialized.remote_traceback
