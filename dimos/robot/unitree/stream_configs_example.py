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

"""Example stream configurations for different deployment scenarios."""

from dimos.stream import StreamConfig


def get_gpu_full_config(camera_intrinsics, camera_pitch=0.0, camera_height=1.0):
    """Get full GPU configuration with all perception capabilities.

    Args:
        camera_intrinsics: List [fx, fy, cx, cy]
        camera_pitch: Camera pitch angle in radians
        camera_height: Camera height in meters

    Returns:
        List of StreamConfig objects
    """
    return [
        StreamConfig(
            name="person_tracking",
            enabled=True,
            device="cuda",
            parameters={
                "camera_intrinsics": camera_intrinsics,
                "camera_pitch": camera_pitch,
                "camera_height": camera_height,
                "model_path": "yolo11n.pt",  # Can use larger models like yolo11x.pt
            },
            priority=10,
        ),
        StreamConfig(
            name="object_tracking",
            enabled=True,
            device="cuda",
            parameters={
                "camera_intrinsics": camera_intrinsics,
                "camera_pitch": camera_pitch,
                "camera_height": camera_height,
                "use_depth_model": True,  # Enable Metric3D depth estimation
                "reid_threshold": 5,
                "reid_fail_tolerance": 10,
            },
            priority=5,
        ),
    ]


def get_cpu_minimal_config(camera_intrinsics, camera_pitch=0.0, camera_height=1.0):
    """Get minimal CPU configuration for resource-constrained environments.

    Args:
        camera_intrinsics: List [fx, fy, cx, cy]
        camera_pitch: Camera pitch angle in radians
        camera_height: Camera height in meters

    Returns:
        List of StreamConfig objects
    """
    return [
        # Disable person tracking on CPU as YOLO is compute-intensive
        StreamConfig(
            name="person_tracking",
            enabled=False,  # Disabled for CPU
            device="cpu",
            parameters={
                "camera_intrinsics": camera_intrinsics,
                "camera_pitch": camera_pitch,
                "camera_height": camera_height,
                "model_path": "yolo11n.pt",  # Smallest model if enabled
            },
            priority=10,
        ),
        StreamConfig(
            name="object_tracking",
            enabled=True,
            device="cpu",
            parameters={
                "camera_intrinsics": camera_intrinsics,
                "camera_pitch": camera_pitch,
                "camera_height": camera_height,
                "use_depth_model": False,  # Disable depth model for CPU
                "reid_threshold": 8,  # Higher threshold for more conservative tracking
                "reid_fail_tolerance": 5,  # Lower tolerance for faster failure detection
            },
            priority=5,
        ),
    ]


def get_hybrid_config(camera_intrinsics, camera_pitch=0.0, camera_height=1.0):
    """Get hybrid configuration with selective GPU usage.

    This configuration enables person tracking on GPU while keeping
    object tracking lightweight for CPU.

    Args:
        camera_intrinsics: List [fx, fy, cx, cy]
        camera_pitch: Camera pitch angle in radians
        camera_height: Camera height in meters

    Returns:
        List of StreamConfig objects
    """
    return [
        StreamConfig(
            name="person_tracking",
            enabled=True,
            device="cuda:0",  # Specify GPU device
            parameters={
                "camera_intrinsics": camera_intrinsics,
                "camera_pitch": camera_pitch,
                "camera_height": camera_height,
                "model_path": "yolo11s.pt",  # Small model for balance
            },
            priority=10,
        ),
        StreamConfig(
            name="object_tracking",
            enabled=True,
            device="cpu",  # Keep on CPU
            parameters={
                "camera_intrinsics": camera_intrinsics,
                "camera_pitch": camera_pitch,
                "camera_height": camera_height,
                "use_depth_model": False,  # No depth for CPU efficiency
                "reid_threshold": 5,
                "reid_fail_tolerance": 10,
            },
            priority=5,
        ),
    ]


def get_custom_config_example():
    """Example of how to create a custom configuration.

    Returns:
        List of StreamConfig objects
    """
    # Example camera parameters for Unitree Go2
    camera_intrinsics = [819.553492, 820.646595, 625.284099, 336.808987]
    camera_pitch = 0.0  # radians
    camera_height = 0.44  # meters

    # Create custom configuration
    configs = []

    # Add only person tracking for a specific use case
    person_config = StreamConfig(
        name="person_tracking",
        enabled=True,
        device="cuda" if check_cuda_available() else "cpu",
        parameters={
            "camera_intrinsics": camera_intrinsics,
            "camera_pitch": camera_pitch,
            "camera_height": camera_height,
            "model_path": "path/to/custom/yolo/model.pt",  # Custom model
        },
        priority=10,
    )
    configs.append(person_config)

    # You can add more custom streams here
    # For example, a future semantic segmentation stream:
    # seg_config = StreamConfig(
    #     name="semantic_segmentation",
    #     enabled=True,
    #     device="cuda",
    #     parameters={...},
    #     dependencies=["person_tracking"],  # Depends on person detection
    #     priority=1,
    # )
    # configs.append(seg_config)

    return configs


def check_cuda_available():
    """Check if CUDA is available on the system.

    Returns:
        bool: True if CUDA is available
    """
    try:
        import torch

        return torch.cuda.is_available()
    except ImportError:
        return False
