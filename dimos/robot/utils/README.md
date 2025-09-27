# Robot Utils

## RobotDebugger

The `RobotDebugger` provides a way to debug a running robot through the python shell.

Requirements:

```bash
pip install rpyc
```

### Usage

1. **Add to your robot application:**
   ```python
   from dimos.robot.utils.robot_debugger import RobotDebugger

   # In your robot application's context manager or main loop:
   with RobotDebugger(robot):
       # Your robot code here
       pass

   # Or better, with an exit stack.
   exit_stack.enter_context(RobotDebugger(robot))
   ```

2. **Start your robot with debugging enabled:**
   ```bash
   ROBOT_DEBUGGER=true python your_robot_script.py
   ```

3. **Open the python shell:**
   ```bash
   ./bin/robot-debugger
    >>> robot.explore()
    True
   ```
