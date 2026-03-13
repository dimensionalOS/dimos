# Source ROS environment
echo "Sourcing ROS environment..."
source /opt/ros/${ROS_DISTRO:-humble}/setup.bash
source /ros2_ws/install/setup.bash
source /opt/dimos-venv/bin/activate

cd /workspace/dimos/docker/navigation/ros-navigation-autonomy-stack

UNITY_EXECUTABLE="/workspace/dimos/docker/navigation/ros-navigation-autonomy-stack/src/base_autonomy/vehicle_simulator/mesh/unity/environment/Model.x86_64"
"$UNITY_EXECUTABLE" &
UNITY_PID=$!

setsid bash -c 'ros2 launch vehicle_simulator system_simulation_with_route_planner.launch.py enable_bridge:=false' &
ROS_PID=$!

ros2 run rviz2 rviz2 -d src/route_planner/far_planner/rviz/default.rviz &
RVIZ_PID=$!
