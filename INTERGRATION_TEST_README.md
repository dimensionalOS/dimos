# DiMOS Simulation Integration Tests

This document outlines the required integration tests that must PASS before merging any changes to the DiMOS codebase.

## Prerequisites

- DiMOS development environment set up
- Mujoco simulator installed
- Python environment with required dependencies

## Test Suite

### 1. Basic Robot Connection Test
- Launch Mujoco simulator with Go2 robot
- Verify robot connection establishes successfully
- Check camera feeds are streaming
- Validate robot state information is received

### 2. Perception Tests
- Test object detection and tracking
- Verify spatial memory module
- Check camera calibration

### 3. Skills Library Tests
- Test basic movement skills
- Verify complex behavior sequences
- Validate skill parameter configurations

## Running the Tests

1. Start Unitree Go2 Mujoco simulator,

First run the unitree_go2.py to check that it has no errors and input the sudo password:

```
CONNECTION_TYPE=mujoco python dimos/robot/unitree_webrtc/unitree_go2.py
```
Then, **kill** the window after putting in the password and seeing no errors

Next, run the runfile and it will start a mujoco window,
```
CONNECTION_TYPE=mujoco python dimos/robot/unitree_webrtc/run.py
```

Then, open ***Foxglove***, and click on *open connection*, click *open* on the default port 8765,

Import the default lcm_dashboard at ```assets/foxglove_unitree_lcm_dashboard.json``` by clicking on the top dashboard layout.

first navigate around the sim by clicking on start keyboard control, and move around with the **wasd** keys and look for any object or landmark in its fov. 

Turn slightly left and you will see a chair, that is the easiest object to track and navigate to.

Next, go to the port ```0.0.0.0:5555```

In the text box, put in ```"go to the chair and then turn 180"``` or click on the mic button if you want to talk to it. 

You should then see it successfully navigating to the chair and then turn 180 degrees. 

Next, walk around the office until you see the bookshelf, go towards the bookshelf. Then turn around and go anywhere else where the bookshelf is no longer in its vicinity. 

Now, in ```0.0.0.0:5555```, type in ```"go to the bookshelf"```, it should then navigate to the book shelf autonomously.

After these two tests, you would have validated that the following features work as expected:
```
spatial memory
navigation stack
object tracker
NavigateWithText skill
agent toolcalls
VLM query
depth camera module
transforms
```



