import open3d as o3d
import numpy as np
from contact_graspnet_pytorch.visualization_utils_o3d import visualize_grasps, show_image

data = np.load('results/predictions_7.npz', allow_pickle=True)
pred_grasps_cam = data['pred_grasps_cam'].item()
scores = data['scores'].item()
pc_full = data['pc_full']
pc_colors = data['pc_colors']

visualize_grasps(pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors)
