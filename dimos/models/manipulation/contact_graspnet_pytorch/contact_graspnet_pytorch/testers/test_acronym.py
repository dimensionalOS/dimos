import time
from scipy.spatial.transform import Rotation as R
import torch
import os
# os.environ['PYOPENGL_PLATFORM'] = 'egl'  # To get pyrender to work 
import open3d as o3d

from contact_graspnet_pytorch.acronym_dataloader import AcryonymDataset
from contact_graspnet_pytorch import config_utils

workspace_root = './'
# workspace_root = '../../'
ckpt_dir = os.path.join(workspace_root, 'checkpoints/contact_graspnet')
batch_size = 6
max_epoch = 100
data_path = os.path.join(workspace_root, 'acronym/')
global_config = config_utils.load_config(ckpt_dir, 
                                         batch_size=batch_size,
                                         max_epoch=max_epoch,
                                         data_path=data_path,
                                         arg_configs=[],
                                         save=False)


dataset = AcryonymDataset(global_config, debug=True, use_saved_renders=False)
saved_dataset = AcryonymDataset(global_config, debug=True, use_saved_renders=True)
sample_list = []
# for i in range(2):
#     sample = dataset.__getitem__(i)
#     sample_list.append(sample)

for i in range(3070, 3074):
    sample = saved_dataset.__getitem__(i)
    sample_list.append(sample)

# pc_cam = sample['pc_cam']

# pts_list = [
#     pc_cam,
# ]

pc_world_list = []
for sample in sample_list:
    pc_cam = sample['pc_cam']
    camera_pose = sample['camera_pose']
    camera_pose_inv = torch.linalg.inv(camera_pose)
    pc_world = sample['pc_cam'] @ camera_pose_inv[:3, :3].T + camera_pose_inv[:3, 3]
    pc_world_list.append(pc_world)
pts_list = pc_world_list

# pts_list = [sample['pc_cam'] for sample in sample_list]

colors = [
    [0, 1, 0],  # Green
    [1, 0, 0],  # Red
    [0, 0, 1],  # Blue
    [1, 1, 0],  # Yellow
    [1, 0, 1],  # Purple
    [0, 1, 1],  # Purple
]

vis = o3d.visualization.Visualizer()
vis.create_window()
for i, pts in enumerate(pts_list):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    # color = (1 + i) / (len(pts_list) + 2)
    # pcd.paint_uniform_color([color, color, color])
    pcd.paint_uniform_color(colors[i])
    vis.add_geometry(pcd)

opt = vis.get_render_option()
opt.show_coordinate_frame = True
vis.run()

print('here')








































xyz_cam = sample['xyz_cam']
scene_contact_points = sample['scene_contact_points']
pc_cam = sample['pc_cam']
camera_pose = sample['camera_pose']
camera_pose_inv = torch.linalg.inv(camera_pose)
camera_pose_inv = torch.eye(4)
camera_pose_inv[:3, :3]= camera_pose[:3, :3].T
camera_pose_inv[:3, 3] = -camera_pose[:3, :3].T @ camera_pose[:3, 3]

# rot = R.from_euler('y', 180, degrees=True)
# rot = rot.as_matrix()

# pc_world = debug['pc_cam'] @ (camera_pose[:3, :3].T) @ rot.T
pc_world = pc_cam @ (camera_pose_inv[:3, :3].T)
pc_world += camera_pose_inv[:3, 3][None, :]

# This should be in world
pos_contact_points = sample['pos_contact_points']

# pos_contact_points should be same as scene_contact_points
# We expect pc_world to be the same as scene contact points
# -- It is not


pts_list = [
    # pos_contact_points @ camera_pose[:3, :3].T + camera_pose[:3, 3][None, :],
    pos_contact_points,  # <-- This is the odd one here
    # xyz_cam,
    # pos_contact_points_cam,
    pos_contact_points_cam @ (camera_pose_inv[:3, :3].T) + camera_pose_inv[:3, 3][None, :],
    # scene_contact_points.reshape(-1, 3),  # <-- This one too
    # pc_cam,
    pc_world
]

colors = [
    [1, 0, 0],  # Red
    [0, 1, 0],  # Green
    [0, 0, 1],  # Blue
    [1, 1, 0],  # Yellow
    [1, 0, 1],  # Purple
    [0, 1, 1],  # Purple
]

vis = o3d.visualization.Visualizer()
vis.create_window()
for i, pts in enumerate(pts_list):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(pts)
    # color = (1 + i) / (len(pts_list) + 2)
    # pcd.paint_uniform_color([color, color, color])
    pcd.paint_uniform_color(colors[i])
    vis.add_geometry(pcd)

opt = vis.get_render_option()
opt.show_coordinate_frame = True
vis.run()

print('here')
