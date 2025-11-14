## Unitree Go2 ROS Control Setup 

Install unitree ros2 workspace as per instructions in https://github.com/dimensionalOS/go2_ros2_sdk/blob/master/README.md

Run the following command to source the workspace and add dimos to the python path:

```
source /home/ros/unitree_ros2_ws/install/setup.bash

export PYTHONPATH=/home/stash/dimensional/dimos:$PYTHONPATH
```

Run the following command to start the ROS control node:

```
ros2 launch go2_robot_sdk robot.launch.py
```

Run the following command to start the agent:

```
python3 dimos/robot/unitree/run_go2_ros.py
```


