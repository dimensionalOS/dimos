import threading
import time
import argparse

import pyzed.sl as sl
from queue import Queue
import cv2
from rx.subject import Subject

from dimos.robot.unitree_webrtc.type.lidar import LidarMessage, Vector
from dimos.robot.unitree_webrtc.type.map import ZedMap

from geometry_msgs.msg import Pose as Position
import os
import open3d as o3d
import numpy as np


class VideoViewer:
    def __init__(self, window_name: str = "ZED Video"):
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        self.window_name = window_name

    def update(self, img_mat: sl.Mat):
        arr = img_mat.get_data()
        if arr.shape[2] == 4:
            frame = cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
        else:
            frame = arr
        cv2.imshow(self.window_name, frame)
        # No waitKey here; main thread handles GUI


def log_pose(pos: Position):
    log_str = (
        f"{time.time():.3f}, "
        f"{pos.position.x:.2f}, {pos.position.y:.2f}, {pos.position.z:.2f}, "
        f"{pos.orientation.x:.2f}, {pos.orientation.y:.2f}, {pos.orientation.z:.2f}, {pos.orientation.w:.2f}"
    )
    log_path = "dimos/robot/zed_robot/zed_pose_log.csv"
    # Write header only if file does not exist or is empty
    write_header = not os.path.exists(log_path) or os.path.getsize(log_path) == 0
    with open(log_path, "a") as f:
        if write_header:
            f.write("timestamp,x,y,z,qx,qy,qz,qw\n")
        f.write(log_str + "\n")


class ZedRobot:
    def __init__(self,
                 init_params: sl.InitParameters = None,
                 enable_imu: bool = True):
        # Frame queue for main-thread display
        self.frame_q = Queue(maxsize=1)

        # Initialize camera
        self.zed = sl.Camera()
        self._init_params = init_params or sl.InitParameters(
            camera_resolution=sl.RESOLUTION.HD720,
            coordinate_units=sl.UNIT.METER,
            coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Z_UP
        )
        status = self.zed.open(self._init_params)
        if status != sl.ERROR_CODE.SUCCESS:
            raise RuntimeError(f"Failed to open ZED camera: {status}")

        # Enable positional tracking
        tracking_params = sl.PositionalTrackingParameters()
        tracking_params.enable_imu_fusion = enable_imu
        tracking_params.mode = sl.POSITIONAL_TRACKING_MODE.GEN_1
        status = self.zed.enable_positional_tracking(tracking_params)
        if status != sl.ERROR_CODE.SUCCESS:
            self.zed.close()
            raise RuntimeError(f"Failed to enable positional tracking: {status}")
        
        # Enable spatial mapping
        self.spatial_mapping_parameters = sl.SpatialMappingParameters(resolution = sl.MAPPING_RESOLUTION.MEDIUM,mapping_range =  sl.MAPPING_RANGE.MEDIUM,max_memory_usage = 2048,save_texture = True,reverse_vertex_order = False,map_type = sl.SPATIAL_MAP_TYPE.MESH)
        self.pymesh = sl.Mesh() 
        spatial_mapping_status = self.zed.enable_spatial_mapping(self.spatial_mapping_parameters)
        if spatial_mapping_status != sl.ERROR_CODE.SUCCESS:
            self.zed.disable_positional_tracking()
            self.zed.close()
            raise RuntimeError(f"Failed to enable spatial mapping: {spatial_mapping_status}")
        


        # Common parameters
        self.runtime = sl.RuntimeParameters()
        self.pose = sl.Pose()
        self.status = None

    def video_stream(self):
        """Start grab loop in a background thread."""
        def _vid_loop():
            while True:
                
                self.status = self.zed.grab(self.runtime)
                if self.status != sl.ERROR_CODE.SUCCESS:
                    break
                # 1) push image to queue
                img = sl.Mat()
                self.zed.retrieve_image(img, sl.VIEW.LEFT)
                if self.frame_q.full():
                    _ = self.frame_q.get()
                self.frame_q.put(img)
                time.sleep(0.03)

        threading.Thread(target=_vid_loop, daemon=True).start()

    def odometry_stream(self) -> Subject:
        """Returns a Subject emitting Position objects."""
        subject: Subject[Position] = Subject()

        def _odom_loop():
            while True:
                # Only get pose when grabbing succeeds
                if self.status != sl.ERROR_CODE.SUCCESS:
                    break
                self.zed.get_position(self.pose, sl.REFERENCE_FRAME.WORLD)
                t = self.pose.get_translation()
                q = self.pose.get_orientation()
                
                pose = Position()
                pose.position.x = round(t.get()[0], 2)
                pose.position.y = round(t.get()[1], 2)
                pose.position.z = round(t.get()[2], 2)
                pose.orientation.x = round(q.get()[0], 2)
                pose.orientation.y = round(q.get()[1], 2)
                pose.orientation.z = round(q.get()[2], 2)
                pose.orientation.w = round(q.get()[3], 2)

                subject.on_next(
                    pose
                )
                log_pose(pose)  # Log the pose to file

                time.sleep(0.03)
            subject.on_completed()

        threading.Thread(target=_odom_loop, daemon=True).start()
        return subject
    
    def mesh_stream(self) -> Subject:
        """Returns a Subject emitting Mesh objects."""
        subject: Subject[sl.Mesh] = Subject()
        frame_count = 0


        def _mesh_loop():
            nonlocal frame_count
            while True:
                if self.status != sl.ERROR_CODE.SUCCESS:
                    time.sleep(0.1)
                    continue
                err = self.zed.extract_whole_spatial_map(self.pymesh)
                print(f"Requesting mesh: {repr(err)}")
                if err == sl.ERROR_CODE.SUCCESS:
                    frame_count += 1
                    if frame_count % 10 == 0:
                        print(f"Mesh frame {frame_count} retrieved")
                        self.pymesh.save(f"dimos/robot/zed_robot/pcds/mesh_obj_{frame_count}.obj")
                    print("Mesh frame retrieved")
                    # self.lidar_message_wrapper()
                    subject.on_next(self.pymesh)
                time.sleep(0.5)
            # subject.on_completed()  # Unreachable, so removed

        threading.Thread(target=_mesh_loop, daemon=True).start()
        return 

    # def lidar_message_wrapper(self) -> Subject:
    #     """Wraps a raw mesh message into a more usable format."""
    #     # Here we can add any additional processing if needed
    #     subject: Subject[LidarMessage] = Subject()

    #     def mesh_to_pointcloud(mesh):
    #         """Convert sl.Mesh to a point cloud."""
    #         points = mesh.vertices
    #         pointcloud = o3d.geometry.PointCloud()
    #         pointcloud.points = o3d.utility.Vector3dVector(points)
    #         return pointcloud
        

    #     def _lidar_wrapper_loop():
    #         while True:
    #             if self.status != sl.ERROR_CODE.SUCCESS:
    #                 time.sleep(0.1)
    #                 continue
    #             trans = self.pose.get_translation().get()

    #             # Extract mesh and wrap it in a LidarMessage
    #             lidar_msg = LidarMessage(
    #                 ts=time.time(),
    #                 origin=Vector(trans),
    #                 resolution=0.05,
    #                 pointcloud=self.pcd,
    #                 raw_msg=None  # No raw message in this case
    #             )
    #             subject.on_next(lidar_msg)
    #             time.sleep(0.5)
    #         subject.on_completed()

    #     threading.Thread(target=_lidar_wrapper_loop, daemon=True).start()
    #     return subject
        

    def close(self):
        
        #Point cloud and mesh cleanup
        err = self.zed.extract_whole_spatial_map(self.pymesh)
        print(f"Extracted mesh: {repr(err)}")

        self.pymesh.apply_texture(sl.MESH_TEXTURE_FORMAT.RGB)

        filter_params = sl.MeshFilterParameters()
        filter_params.set(sl.MESH_FILTER.MEDIUM)
        # Apply filter to the mesh
        self.pymesh.filter(filter_params,True)

        print("save texture set to: {}".format(self.spatial_mapping_parameters))
        self.pymesh.apply_texture(sl.MESH_TEXTURE_FORMAT.RGBA)

        mesh_path = "dimos/robot/zed_robot/zed_mesh.obj"
        status = self.pymesh.save(mesh_path)
        if status:
            print(f"Mesh saved to {mesh_path}")
        else:
            print("Failed to save mesh.")

        self.zed.disable_positional_tracking()
        self.pymesh.clear()
        self.zed.close()


def main():
    init_params = sl.InitParameters(
        camera_resolution=sl.RESOLUTION.HD720,
        coordinate_units=sl.UNIT.METER,
        coordinate_system=sl.COORDINATE_SYSTEM.RIGHT_HANDED_Y_UP
    )

    # Instantiate ZedRobot
    robot = ZedRobot(init_params=init_params)
    robot.video_stream()
    


    # Create viewer
    vid_viewer = VideoViewer()

    # Start odometry stream
    odo_stream = robot.odometry_stream()
    mesh_stream = robot.mesh_stream()
    # lidar_stream = robot.lidar_message_wrapper()



    # Subscribe odometry
    odo_stream.subscribe(
        lambda pos: print(f"Pose: x={pos.position.x:.2f}, y={pos.position.y:.2f}, z={pos.position.z:.2f}, "
                          f"quat=({pos.orientation.x:.2f},{pos.orientation.y:.2f},{pos.orientation.z:.2f},{pos.orientation.w:.2f})")
    )
    
    

    try:
        while True:
            # Main thread handles GUI
            if not robot.frame_q.empty():
                frame = robot.frame_q.get()
                vid_viewer.update(frame)
            if cv2.waitKey(1) == ord('q'):
                break
            time.sleep(0.01)    
    except KeyboardInterrupt:
        pass
    finally:
        # Clean up
        print("Shutting down...")
        robot.close()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
