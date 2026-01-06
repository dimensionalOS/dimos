import argparse
import glob
import os

from contact_graspnet_pytorch import config_utils  # type: ignore[import-not-found]
from contact_graspnet_pytorch.checkpoints import CheckpointIO  # type: ignore[import-not-found]
from contact_graspnet_pytorch.contact_grasp_estimator import (  # type: ignore[import-not-found]
    GraspEstimator,
)
from contact_graspnet_pytorch.data import (  # type: ignore[import-not-found]
    load_available_input_data,
)
import numpy as np

from dimos.utils.data import get_data


def inference(global_config,  # type: ignore[no-untyped-def]
              ckpt_dir,
              input_paths,
              local_regions: bool=True,
              filter_grasps: bool=True,
              skip_border_objects: bool=False,
              z_range = None,
              forward_passes: int=1,
              K=None,):
    """
    Predict 6-DoF grasp distribution for given model and input data

    :param global_config: config.yaml from checkpoint directory
    :param checkpoint_dir: checkpoint directory
    :param input_paths: .png/.npz/.npy file paths that contain depth/pointcloud and optionally intrinsics/segmentation/rgb
    :param K: Camera Matrix with intrinsics to convert depth to point cloud
    :param local_regions: Crop 3D local regions around given segments.
    :param skip_border_objects: When extracting local_regions, ignore segments at depth map boundary.
    :param filter_grasps: Filter and assign grasp contacts according to segmap.
    :param segmap_id: only return grasps from specified segmap_id.
    :param z_range: crop point cloud at a minimum/maximum z distance from camera to filter out outlier points. Default: [0.2, 1.8] m
    :param forward_passes: Number of forward passes to run on each point cloud. Default: 1
    """
    # Build the model
    if z_range is None:
        z_range = [0.2, 1.8]
    grasp_estimator = GraspEstimator(global_config)

    # Load the weights
    model_checkpoint_dir = get_data(ckpt_dir)
    checkpoint_io = CheckpointIO(checkpoint_dir=model_checkpoint_dir, model=grasp_estimator.model)
    try:
        checkpoint_io.load('model.pt')
    except FileExistsError:
        print('No model checkpoint found')


    os.makedirs('results', exist_ok=True)

    # Process example test scenes
    for p in glob.glob(input_paths):
        print('Loading ', p)

        pc_segments = {}
        segmap, rgb, depth, cam_K, pc_full, pc_colors = load_available_input_data(p, K=K)

        if segmap is None and (local_regions or filter_grasps):
            raise ValueError('Need segmentation map to extract local regions or filter grasps')

        if pc_full is None:
            print('Converting depth to point cloud(s)...')
            pc_full, pc_segments, pc_colors = grasp_estimator.extract_point_clouds(depth, cam_K, segmap=segmap, rgb=rgb,
                                                                                    skip_border_objects=skip_border_objects,
                                                                                    z_range=z_range)

        print(pc_full.shape)

        print('Generating Grasps...')
        pred_grasps_cam, scores, contact_pts, _ = grasp_estimator.predict_scene_grasps(pc_full,
                                                                                       pc_segments=pc_segments,
                                                                                       local_regions=local_regions,
                                                                                       filter_grasps=filter_grasps,
                                                                                       forward_passes=forward_passes)

        # Save results
        np.savez('results/predictions_{}'.format(os.path.basename(p.replace('png','npz').replace('npy','npz'))),
                  pc_full=pc_full, pred_grasps_cam=pred_grasps_cam, scores=scores, contact_pts=contact_pts, pc_colors=pc_colors)

        # Visualize results
        # show_image(rgb, segmap)
        # visualize_grasps(pc_full, pred_grasps_cam, scores, plot_opencv_cam=True, pc_colors=pc_colors)

    if not glob.glob(input_paths):
        print('No files found: ', input_paths)

if __name__ == "__main__":

    parser = argparse.ArgumentParser()
    parser.add_argument('--ckpt_dir', default='models_contact_graspnet', help='Log dir')
    parser.add_argument('--np_path', default='test_data/7.npy', help='Input data: npz/npy file with keys either "depth" & camera matrix "K" or just point cloud "pc" in meters. Optionally, a 2D "segmap"')
    parser.add_argument('--K', default=None, help='Flat Camera Matrix, pass as "[fx, 0, cx, 0, fy, cy, 0, 0 ,1]"')
    parser.add_argument('--z_range', default=[0.2,1.8], help='Z value threshold to crop the input point cloud')
    parser.add_argument('--local_regions', action='store_true', default=True, help='Crop 3D local regions around given segments.')
    parser.add_argument('--filter_grasps', action='store_true', default=True,  help='Filter grasp contacts according to segmap.')
    parser.add_argument('--skip_border_objects', action='store_true', default=False,  help='When extracting local_regions, ignore segments at depth map boundary.')
    parser.add_argument('--forward_passes', type=int, default=1,  help='Run multiple parallel forward passes to mesh_utils more potential contact points.')
    parser.add_argument('--arg_configs', nargs="*", type=str, default=[], help='overwrite config parameters')
    FLAGS = parser.parse_args()

    global_config = config_utils.load_config(FLAGS.ckpt_dir, batch_size=FLAGS.forward_passes, arg_configs=FLAGS.arg_configs)

    print(str(global_config))
    print(f'pid: {os.getpid()!s}')

    inference(global_config,
              FLAGS.ckpt_dir,
              FLAGS.np_path,
              local_regions=FLAGS.local_regions,
              filter_grasps=FLAGS.filter_grasps,
              skip_border_objects=FLAGS.skip_border_objects,
              z_range=eval(str(FLAGS.z_range)),
              forward_passes=FLAGS.forward_passes,
              K=eval(str(FLAGS.K)))
