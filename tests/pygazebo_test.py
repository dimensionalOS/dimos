import asyncio
import pygazebo
from pygazebo.msg.pose_pb2 import Pose
from pygazebo.msg.vector3d_pb2 import Vector3d
from pygazebo.msg.quaternion_pb2 import Quaternion

async def publish_pose():
    manager = await pygazebo.connect()
    publisher = await manager.advertise('/gazebo/default/pose/info', 'gazebo.msgs.Pose')

    pose = Pose()
    pose.position.x = 1.0  # delta_x
    pose.position.y = 0.0  # delta_y
    pose.position.z = 0.0

    pose.orientation.w = 1.0
    pose.orientation.x = 0.0
    pose.orientation.y = 0.0
    pose.orientation.z = 0.0

    while True:
        await publisher.publish(pose)
        await asyncio.sleep(0.1)

loop = asyncio.get_event_loop()
loop.run_until_complete(publish_pose())
