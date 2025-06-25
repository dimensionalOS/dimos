import glob
import os

import torch
from contact_graspnet_pytorch.contact_grasp_estimator_pytorch import GraspEstimator
from contact_graspnet_pytorch import config_utils

from contact_graspnet_pytorch.visualization_utils_o3d import visualize_grasps, show_image
from contact_graspnet_pytorch.checkpoints import CheckpointIO 
# from contact_graspnet_pytorch.visualization_utils_plotly import show_image

from data import load_available_input_data
ckpt_dir = 'checkpoints/contact_graspnet'
forward_passes = 1
arg_configs = []

global_config = config_utils.load_config(ckpt_dir, batch_size=forward_passes, arg_configs=arg_configs)

grasp_estimator = GraspEstimator(global_config)

model_checkpoint_dir = os.path.join(ckpt_dir, 'checkpoints')
checkpoint_io = CheckpointIO(checkpoint_dir=model_checkpoint_dir, model=grasp_estimator.model)

try:
    load_dict = checkpoint_io.load('model.pt')
except FileExistsError:
    print('No model checkpoint found')
    load_dict = {}

input_paths = 'test_data/*.npy'
local_regions = True
filter_grasps = True
K = None
skip_border_objects = False
z_range = [0.2,1.8]

# def inference(global_config, checkpoint_dir, input_paths, K=None, local_regions=True, 
#               skip_border_objects=False, filter_grasps=True, segmap_id=None, 
#               z_range=[0.2,1.8], forward_passes=1):

for p in glob.glob(input_paths)[1:]:
    print('Loading ', p)

    pc_segments = {}
    
    segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(p, K=K)

    if segmap is None and (local_regions or filter_grasps):
        raise ValueError('Need segmentation map to extract local regions or filter grasps')

    if pc_full is None:
        print('Converting depth to point cloud(s)...')
        pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(depth, cam_K, segmap=segmap, rgb=rgb,
                                                                                skip_border_objects=skip_border_objects, z_range=z_range)
    
    print(pc_full.shape)

    print('Generating Grasps...')
    pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(pc_full, 
                                                                                   pc_segments=pc_segments, 
                                                                                   local_regions=local_regions, 
                                                                                   filter_grasps=filter_grasps, 
                                                                                   forward_passes=forward_passes)  

    show_image(rgb, segmap)
    visualize_grasps(pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors)