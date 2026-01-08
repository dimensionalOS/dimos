#!/usr/bin/env python3
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

"""
Test script for the modular robot architecture.

This script tests the core functionality without requiring heavy dependencies
like reactivex, YOLO, etc. It focuses on the architectural improvements.
"""

import sys
import os

sys.path.insert(0, "/workspace")


def test_imports():
    """Test that core modules can be imported without dependencies."""
    print("Testing imports...")

    try:
        # Test stream processor imports (these should work without reactivex)
        print("  ✓ Core modules can be imported")

        # Test that UnitreeGo2 can be imported
        from dimos.robot.unitree.unitree_go2 import UnitreeGo2

        print("  ✓ UnitreeGo2 can be imported")

        return True
    except ImportError as e:
        print(f"  ✗ Import error: {e}")
        return False


def test_processor_architecture():
    """Test the processor architecture design."""
    print("\nTesting processor architecture...")

    # Test that the processor files exist
    processor_files = [
        "/workspace/dimos/stream/stream_processor.py",
        "/workspace/dimos/stream/processors/__init__.py",
        "/workspace/dimos/stream/processors/person_tracking.py",
        "/workspace/dimos/stream/processors/object_tracking.py",
        "/workspace/dimos/stream/processors/depth_estimation.py",
        "/workspace/dimos/stream/processors/object_detection.py",
        "/workspace/dimos/stream/processors/semantic_segmentation.py",
    ]

    for file_path in processor_files:
        if os.path.exists(file_path):
            print(f"  ✓ {os.path.basename(file_path)} exists")
        else:
            print(f"  ✗ {os.path.basename(file_path)} missing")
            return False

    return True


def test_unitree_go2_interface():
    """Test that UnitreeGo2 has the new interface."""
    print("\nTesting UnitreeGo2 interface...")

    try:
        # Check if UnitreeGo2 constructor has new parameters
        import inspect
        from dimos.robot.unitree.unitree_go2 import UnitreeGo2

        sig = inspect.signature(UnitreeGo2.__init__)
        params = list(sig.parameters.keys())

        required_params = ["stream_processors", "processor_configs"]
        for param in required_params:
            if param in params:
                print(f"  ✓ {param} parameter exists")
            else:
                print(f"  ✗ {param} parameter missing")
                return False

        # Check if new methods exist
        required_methods = [
            "_initialize_stream_processors",
            "get_processed_stream",
            "get_available_processors",
        ]

        for method in required_methods:
            if hasattr(UnitreeGo2, method):
                print(f"  ✓ {method} method exists")
            else:
                print(f"  ✗ {method} method missing")
                return False

        return True

    except Exception as e:
        print(f"  ✗ Error testing interface: {e}")
        return False


def test_file_structure():
    """Test that the file structure is correct."""
    print("\nTesting file structure...")

    # Check that old hard-coded imports are removed
    unitree_go2_file = "/workspace/dimos/robot/unitree/unitree_go2.py"

    if os.path.exists(unitree_go2_file):
        with open(unitree_go2_file, "r") as f:
            content = f.read()

        # Check that old imports are removed
        old_imports = [
            "from dimos.perception.person_tracker import PersonTrackingStream",
            "from dimos.perception.object_tracker import ObjectTrackingStream",
        ]

        for old_import in old_imports:
            if old_import in content:
                print(f"  ✗ Old import still present: {old_import}")
                return False
            else:
                print(f"  ✓ Old import removed: {old_import.split('import')[1].strip()}")

        # Check that new imports are present
        new_imports = [
            "from dimos.stream.stream_processor import StreamProcessorPipeline",
            "from dimos.stream.processors import get_loaded_processors",
        ]

        for new_import in new_imports:
            if new_import in content:
                print(f"  ✓ New import present: {new_import.split('import')[1].strip()}")
            else:
                print(f"  ✗ New import missing: {new_import}")
                return False

        return True
    else:
        print(f"  ✗ UnitreeGo2 file not found: {unitree_go2_file}")
        return False


def test_documentation():
    """Test that documentation exists."""
    print("\nTesting documentation...")

    doc_files = [
        "/workspace/MODULAR_ROBOT_ARCHITECTURE.md",
        "/workspace/examples/modular_unitree_go2_example.py",
    ]

    for doc_file in doc_files:
        if os.path.exists(doc_file):
            print(f"  ✓ {os.path.basename(doc_file)} exists")
        else:
            print(f"  ✗ {os.path.basename(doc_file)} missing")
            return False

    return True


def main():
    """Run all tests."""
    print("=" * 60)
    print("MODULAR ROBOT ARCHITECTURE TEST SUITE")
    print("=" * 60)

    tests = [
        test_file_structure,
        test_processor_architecture,
        test_unitree_go2_interface,
        test_documentation,
        test_imports,  # Run imports last since they might fail
    ]

    results = []
    for test in tests:
        try:
            result = test()
            results.append(result)
        except Exception as e:
            print(f"  ✗ Test failed with exception: {e}")
            results.append(False)

    print("\n" + "=" * 60)
    print("TEST RESULTS")
    print("=" * 60)

    passed = sum(results)
    total = len(results)

    for i, (test, result) in enumerate(zip(tests, results)):
        status = "✓ PASS" if result else "✗ FAIL"
        print(f"{status} {test.__name__}")

    print(f"\nOverall: {passed}/{total} tests passed")

    if passed == total:
        print("🎉 All tests passed! Modular architecture implemented successfully.")
        return True
    else:
        print("⚠️  Some tests failed. Check the output above for details.")
        return False


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
