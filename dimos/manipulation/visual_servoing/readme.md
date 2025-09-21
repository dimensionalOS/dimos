# xArm Position-Based Visual Servoing (PBVS) Testing System

This document describes the xArm PBVS manipulation system with integrated error handling and recovery mechanisms.

## Hardware Requirements

### Robot Hardware
- **xArm7 Robotic Arm**: 7-DOF collaborative robot arm from UFactory
  - IP Address: `10.0.0.197` (default configuration)
  - Gripper: xArm gripper with force/current sensing
  - Collision sensitivity set to level 1 (most sensitive)

### Camera System
- **ZED Camera**: Stereo depth camera for RGB-D perception
  - Resolution: HD720 (1280x720)
  - Depth mode: NEURAL (AI-enhanced depth)
  - Frame rate: 30 FPS
  - Tracking: Disabled (using xArm transforms instead)
  Connect ZED_Camera to your CUDA compatible PC

### Network Setup
- Robot and control computer must be on same network
- xArm controller accessible at `10.0.0.197:18333`
- Ensure firewall allows communication on xArm ports

## Software Requirements

### Core Dependencies
- **Dimos Framework**: Distributed robotics framework
- **xArm Python SDK**: Official UFactory Python API
- **ZED SDK**: Stereo Labs camera drivers and SDK
- **OpenCV**: Computer vision processing
- **NumPy**: Numerical computing
- **ROS/TF**: Transform handling (via Dimos TF system)

### Key Components
1. **XArmModule** (`dimos/hardware/ufactory.py`): Hardware interface with error detection
2. **ManipulationModule** (`dimos/manipulation/visual_servoing/manipulation_module.py`): PBVS control logic
3. **XArmRobot** (`dimos/robot/ufactory/xarm_robot.py`): High-level robot system integration

## System Architecture

#### Error Detection Mechanisms
1. **Return Code Checking**: All `cmd_ee_pose()` calls return error codes (0 = success, non-zero = error)
2. **RPC Timeout Handling**: Gripper operations wrapped in try-catch blocks for timeout detection
3. **Movement Timeout with Retry**: `_wait_for_reach()` implements retry logic after timeouts

#### Recovery Flow
```
Error Detected → RECOVERY State → Clear Errors → Goto Observe → Retry Operation (max 1 retry) → Continue or Fail
```

#### State Machine Stages
- `IDLE`: Initial state, waiting for commands
- `PRE_GRASP`: Visual servoing to pre-grasp position
- `GRASP`: Move to final grasp position
- `CLOSE_AND_RETRACT`: Close gripper and retract
- `PLACE`: Move to place position (if specified)
- `RETRACT`: Final retraction to observe position
- `RECOVERY`: Error recovery state

## Configuration Parameters

### Robot Configuration (xarm_robot.py)
```python
reach_timeout=15.0          # Timeout for reaching poses (seconds)
pregrasp_distance=0.3       # Distance to hold from target during pre-grasp (meters)
grasp_distance_range=0.02   # Range for final grasp offset interpolation (meters)
grasp_width_offset=0.02     # Additional gripper opening (meters)
gripper_max_opening=0.1     # Maximum gripper opening (meters)
retract_distance=0.18       # Distance to retract after grasping (meters)
```

### Detection Parameters
```python
min_confidence=0.5          # Minimum detection confidence threshold
max_depth=1.0              # Maximum valid depth (meters)
max_object_size=0.15       # Maximum object size to consider valid (meters)
```

### Workspace Limits
```python
workspace_min_radius=0.2    # Minimum reach distance (meters)
workspace_max_radius=1.0    # Maximum reach distance (meters)
```

## Running the System

### Basic Test Script
Use the `test_pick_and_place_module.py` for testing:

python3 dimos/tests/test_pick_and_place_module.py --robot-type xarm

- specify robot-type "piper" or "xarm" depending on which one is used.

```bash
cd /home/mustafa/dimos
python -m dimos.tests.test_pick_and_place_module
```

### Test Parameters

#### Pick Target Options
1. **Pixel Coordinates**: `(x, y)` tuple for clicking on objects in camera view
   ```python
   pick_target = (640, 360)  # Center of 1280x720 image
   ```

2. **Detection3D Object**: Direct object detection (for automated selection)
   ```python
   pick_target = detection_3d_object  # From detection system
   ```

#### Place Target Options
1. **Pixel Coordinates**: `(x, y)` tuple for place location
   ```python
   place_target = (800, 400)  # Place location in image
   ```

2. **None**: Pick-only operation (no placing)
   ```python
   place_target = None
   ```

## Troubleshooting

### Common Issues

#### C31 Controller Errors
- **Symptom**: "ControllerError, code: 31" in logs
- **Cause**: Abnormal current detected (collision, obstruction, etc.)
- **Solution**: System automatically enters RECOVERY state, clears errors, and retries

#### RPC Timeouts
- **Symptom**: "RPC call timed out after 2.0 seconds"
- **Cause**: Hardware operation blocked by controller error
- **Solution**: Try-catch blocks transition to RECOVERY state

#### Movement Timeouts
- **Symptom**: "Failed to reach target pose within 15.0s"
- **Cause**: Robot cannot reach commanded pose
- **Solution**: Automatic retry after RECOVERY (max 1 retry)

### Recovery Process
1. **Clear Errors**: `arm.clear_errors()` clears xArm error states
2. **Reset Position**: `arm.goto_observe()` moves to safe position
3. **Retry**: Return to previous stage with fresh attempt
4. **Fail Safe**: After max retries, task fails gracefully

### Collision Sensitivity
The system uses collision sensitivity level 1 (most sensitive) for safety:
```python
self.arm.arm.set_collision_sensitivity(1)
```

## Key Features Implemented

1. **Robust Error Handling**: Comprehensive error detection and recovery
2. **Retry Logic**: Automatic retry after timeouts (max 1 retry per stage)
3. **RPC Timeout Protection**: All gripper operations protected from hanging
4. **State Machine Recovery**: Clean transitions to RECOVERY state
5. **Collision Detection**: Sensitive collision detection with automatic recovery
6. **Visual Servoing**: Real-time object tracking and pose updates
7. **Workspace Limits**: Safety boundaries for robot operation

## Testing Notes

- System handles C31 errors gracefully without hanging
- Retry mechanism provides second chance for movements
- All gripper operations protected from RPC timeouts
- Visual servoing maintains target tracking throughout operation
- Recovery state clears errors and provides clean restart