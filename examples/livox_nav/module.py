from dimos.core import Module, rpc, In, Out
from dimos.hardware.sensors.lidar.fastlio2.module import FastLio2
from dimos.robot.unitree.go2.connection import GO2Connection
from dimos.core.blueprints import autoconnect
from dimos.visualization.rerun.bridge import rerun_bridge
from dimos.msgs.sensor_msgs import PointCloud2
from dimos.msgs.nav_msgs import Odometry
from dimos.msgs.geometry_msgs import Transform, Vector3, Quaternion


from dimos.core.introspection import to_svg
from dimos.mapping.costmapper import cost_mapper
from dimos.mapping.voxels import voxel_mapper
from dimos.navigation.frontier_exploration import wavefront_frontier_explorer
from dimos.navigation.replanning_a_star.module import replanning_a_star_planner
from dimos.robot.unitree.go2.blueprints.basic.unitree_go2_basic import unitree_go2_basic

from pathlib import Path
import time
import pickle
import math

voxel_size = 0.05

class RecordMid360Module(Module):
    lidar: In[PointCloud2]
    odometry: In[Odometry]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.file = None

    @rpc
    def start(self):
        self.file = open("lidar.pkl", "wb")
        self.file2 = open('odo.pkl', "wb")
        print("Started recording lidar data to lidar.pkl")

        def save_lidar(msg):
            pickle.dump(msg, self.file)
            print(f"Saved pointcloud at ts={msg.ts}")
            
        def save_odom(msg):
            pickle.dump(msg, self.file2)
            print(f"Saved odometry at ts={msg.ts}")

        self.lidar.subscribe(save_lidar)
        self.odometry.subscribe(save_odom)

    @rpc
    def stop(self):
        if self.file:
            self.file.close()
            print(f"Recording stopped.")
        super().stop()
        
class ReplayMid360Module(Module):
    lidar: Out[PointCloud2]

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.file = None

    @rpc
    def start(self):
        import threading
        self.file = open("lidar.pkl", "rb")
        self._running = True
        self._thread = threading.Thread(target=self._replay_loop, daemon=True)
        self._thread.start()

    def _replay_loop(self):
        floor_orienation = Transform(
            translation=Vector3(0, 0, 0),
            rotation=Quaternion.from_euler(Vector3(0,math.radians(24),0)),
        )
        try:
            print('Starting replay from lidar.pkl')
            while self._running:
                pcd: PointCloud2 = pickle.load(self.file)
                print(f"Replaying pointcloud at ts={pcd.ts}")
                self.lidar.publish(pcd.transform(floor_orienation))
                time.sleep(0.1)  # Add small delay between frames
        except EOFError:
            print("Replay finished - reached end of file")
            self._running = False

    @rpc
    def stop(self):
        self._running = False
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=2.0)
        if self.file:
            self.file.close()

unitree_go2 = autoconnect(
    unitree_go2_basic,                    # robot connection + visualization
    voxel_mapper(voxel_size=0.05),        # 3D voxel mapping
    cost_mapper(),                        # 2D costmap generation
    replanning_a_star_planner(),          # path planning
    wavefront_frontier_explorer(),        # exploration
).global_config(n_dask_workers=6, robot_model="unitree_go2")



record_mid360 = autoconnect(
    FastLio2.blueprint(voxel_size=voxel_size, map_voxel_size=voxel_size, map_freq=-1),
    rerun_bridge(
        visual_override={
            "world/lidar": lambda grid: grid.to_rerun(voxel_size=voxel_size, mode="boxes"),
        }
    ),
    RecordMid360Module.blueprint()
)

replay_mid360 = autoconnect(
    ReplayMid360Module.blueprint(),
    rerun_bridge(
    visual_override={
            "world/lidar": lambda grid: grid.to_rerun(voxel_size=voxel_size, mode="boxes"),
        }
    ),
)

replay_object_permanence_mid360 = autoconnect(
    ReplayMid360Module.blueprint(),
    voxel_mapper(voxel_size=voxel_size),
    rerun_bridge()
)

if __name__ == "__main__":
    # record_mid360.build().loop()
    # replay_mid360.build().loop()
    replay_object_permanence_mid360.build().loop()