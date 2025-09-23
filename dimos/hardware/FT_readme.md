### Force Torque README

We have already migrated the FT sensor code from using ZMQ to using the dimos modules approach. The driver and the visualizer are run whenever we run the `ft_module_test.py` script, which deploys the FT driver and visualizer modules.

For the handle grasping, we can run normal_move_test.py, which takes the frames from the ZED camera and a user selected point on the handle and then estimates the normal at the selected point and move the arm to be in front of it. The changes that we need to make here are that we need to take the ZED data from the LCM streams instead of directly through the ZED SDK. We also need to integrate the querying with qwen as a part of the function itself so it should do the find point -> estimate normal -> move to normal direction all in one function. The next thing we need to do is to make that function run twice or thrice since the first time, the normal estimation is a little off, but it gets us close and the second or third times, we are close enough to be decently aligned with the handle. We will leave the code to move forward and actually grab the handle to a separate function/program.

For the opener, we have the olg logic in continuous_door_opener.py. This takes some parameters on which axis to rotate about and which direction to rotate in. This script needs a few changes. We need to get the data for the forces and torques from the LCM streams instead of from ZMQ since we have made that change. We also need to have a stop angle parameter that checks how far we have rotated in the global coordinate system to know how much the hinge has been opened by from when we originally started rotating. Currently, it's only tested to be realiable to rotate hinges in the Z axis- like microwave doors, fridge handles, etc. The logic should work for other axes and the logic is in there, but it's untested for other axes and probably has some bugs.

We also need the dim_cpp folder and the urdfs from the assets folder from https://github.com/dimensionalOS/dimos_utils/tree/openft_test/assets. This is so that we have the meshes, URDF, etc that Drake needs in order to establish context on the robot. Currently, I have the URDF set up and generated for the xarm6, but the arguments in the xacro file can be easily modified to make a URDF for the xarm7. We just need to change the mesh path and types to match the format that Drake is looking for- so no package:// tags and all the meshes need to be objs.

The force torque sensor also needs to be mounted in exactly the orientation that we had it on in the demo videos of the fridge and microwave/how it was on the xarm6. Otherwise, we won't be able to correlate the force axes to the axes of the force torque sensor in the URDF.

python3 dimos/hardware/ft_pull_test.py --xarm 192.168.1.210 --end-angle 45 --port /dev/ttyACM0 --calibration dimos/hardware/ft_calibration.json --auto-run

python handle_grab_test.py --xarm 192.168.1.210 --grab --qwen
