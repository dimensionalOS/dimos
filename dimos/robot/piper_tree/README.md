# PiperTree Mobile Manipulation System

Mobile pick-and-place system combining Piper Arm on Unitree Go2 with visual servoing.

## Launch Instructions

1. **Activate CAN connection**
   ```bash
   bash dimos/hardware/can_activate.sh
   ```

   **Reset the piper arm just for good practice**
   ```bash
   python /path/to/your/piper_sdk/piper_sdk/demo/V2/piper_ctrl_reset.py
   ```

2. **Launch all three components** (in separate terminals)
   ```bash
   python dimos/robot/piper_tree/test_piper_tree.py
   python dimos/robot/unitree_webrtc/unitree_go2.py
   python dimos/robot/agilex/piper_arm.py
   ```

## Controls

### Keyboard Commands
- `m` - Switch mode (cycles through pick_place, servoing, mobile_pick_place)
- `p` - Execute pick-only (after selecting pick location)
- `r` - Reset everything (arm, gripper, servoing)
- `s` - Stop servoing
- `g` - Open gripper
- `q` - Quit

### Operating Modes

**Pick and Place Mode:**
1. Click to select PICK location
2. Click to select PLACE location (auto-executes)
3. OR press `p` after first click for pick-only

**Visual Servoing Mode:**
- Click on object to servo to it

**Mobile Pick and Place Mode:**
- Click on object to servo to it and pick it up

## Troubleshooting

### Visualize Debug Topics in Foxglove

Monitor these LCM topics to debug the system:

- **Visual Servoing**: `/servoing/viz` - Shows object tracking and servoing status
- **Manipulation**: `/manipulation/viz` - Shows grasp detection and manipulation feedback
- **Camera Feed**: `/zed/color_image` - Raw camera input
- **Detections**: `/servoing/detection2d` and `/servoing/detection3d` - Object detections
- **Servoing State**: `/servoing/state` - Current servoing state (tracking/reached/idle)
- **Command Velocity**: `/cmd_vel` - Mobile base velocity commands

### Common Issues

1. **CAN connection failed**: Re-run `can_activate.sh` with sudo permissions
2. **Arm not responding**: Check CAN connection and power supply
3. **No object tracking**: Verify camera topics are publishing and object is in view
4. **Robot not moving**: Check `/cmd_vel` topic and Unitree connection status
5. **Piper arm is soft-stopped**: If the arm is not in observe position (arm raised, camera looking down), reset it:
   ```bash
   python /path/to/your/piper_sdk/piper_sdk/demo/V2/piper_ctrl_reset.py
   ```