import os
import sys
import random

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(os.path.join(BASE_DIR))
sys.path.append(os.path.join(BASE_DIR, 'Pointnet_Pointnet2_pytorch'))

from PIL import Image
import argparse
import numpy as np
import copy
import cv2
import glob
import trimesh.transformations as tra
from scipy.spatial import cKDTree

import provider
from contact_graspnet_pytorch.scene_renderer import SceneRenderer

import torch
import torch.nn.functional as F
from torch.utils.data import Dataset

from contact_graspnet_pytorch import utils
"""
Missing objects:

/home/user/Documents/grasp_generation/contact_graspnet_pytorch/acronym/meshes/47fc4a2417ca259f894dbff6c6a6be89.obj

/home/user/Documents/grasp_generation/contact_graspnet_pytorch/acronym/meshes/49a0a84ee5a91344b11c5cce13f76151.obj

/home/user/Documents/grasp_generation/contact_graspnet_pytorch/acronym/meshes/feb146982d0c64dfcbf4f3f04bbad8.obj

/home/user/Documents/grasp_generation/contact_graspnet_pytorch/acronym/meshes/8123f469a08a88e7761dc3477e65a72.obj


Skipping object (simplify failed): 330880146fd858e2cb6782d0f0acad78
Skipping object (simplify failed): a24e0ea758d43657a5e3e028709e0474
Skipping object (simplify failed): 37d510f7958b941b9163cfcfcc029614
Skipping object (simplify failed): d25b80f268f5731449c3792a0dc29860
"""

class AcryonymDataset(Dataset):
    """
    Class to load scenes, render point clouds and augment them during training

    Arguments:
        root_folder {str} -- acronym root folder
        batch_size {int} -- number of rendered point clouds per-batch

    Keyword Arguments:
        raw_num_points {int} -- Number of random/farthest point samples per scene (default: {20000})
        estimate_normals {bool} -- compute normals from rendered point cloud (default: {False})
        caching {bool} -- cache scenes in memory (default: {True})
        use_uniform_quaternions {bool} -- use uniform quaternions for camera sampling (default: {False})
        scene_obj_scales {list} -- object scales in scene (default: {None})
        scene_obj_paths {list} -- object paths in scene (default: {None})
        scene_obj_transforms {np.ndarray} -- object transforms in scene (default: {None})
        num_train_samples {int} -- training scenes (default: {None})
        num_test_samples {int} -- test scenes (default: {None})
        use_farthest_point {bool} -- use farthest point sampling to reduce point cloud dimension (default: {False})
        intrinsics {str} -- intrinsics to for rendering depth maps (default: {None})
        distance_range {tuple} -- distance range from camera to center of table (default: {(0.9,1.3)})
        elevation {tuple} -- elevation range (90 deg is top-down) (default: {(30,150)})
        pc_augm_config {dict} -- point cloud augmentation config (default: {None})
        depth_augm_config {dict} -- depth map augmentation config (default: {None})
    """
    def __init__(self, global_config, debug=False, train=True, device=None, use_saved_renders=False):
        if device is None:
            self.device = torch.device('cpu')
        else:
            self.device = device

        self.train = train
        self.use_saved_renders = use_saved_renders

        # if use_saved_renders:
        if not os.path.exists(os.path.join(global_config['DATA']['data_path'], 'renders')):
            os.mkdir(os.path.join(global_config['DATA']['data_path'], 'renders'))

        # TODO: Add train vs test split
        # -- Index data -- #
        self.scene_contacts_dir, self.valid_scene_contacts = \
            self._check_scene_contacts(global_config['DATA']['data_path'],
                                       global_config['DATA']['scene_contacts_path'],
                                       debug=debug)

        self._num_test_scenes = global_config['DATA']['num_test_scenes']
        self._num_train_scenes = len(self.valid_scene_contacts) - self._num_test_scenes

        if 'train_and_test' in global_config['DATA'] and global_config['DATA']['train_and_test']:
            self._num_train_samples = self._num_train_samples + self._num_test_samples
            self._num_test_samples = 0
            print('using train and test data')
        print('using %s meshes' % (self._num_train_scenes + self._num_test_scenes))


        # -- Grab configs -- #
        self.global_config = global_config

        self._root_folder = global_config['DATA']['data_path']

        self._batch_size = global_config['OPTIMIZER']['batch_size']
        self._raw_num_points = global_config['DATA']['raw_num_points']
        self._caching = True



        self._estimate_normals = global_config['DATA']['input_normals']
        self._use_farthest_point = global_config['DATA']['use_farthest_point']
        # self._scene_obj_scales = [c['obj_scales'] for c in contact_infos]
        # self._scene_obj_paths = [c['obj_paths'] for c in contact_infos]
        # self._scene_obj_transforms = [c['obj_transforms'] for c in contact_infos]
        self._distance_range = global_config['DATA']['view_sphere']['distance_range']
        self._pc_augm_config = global_config['DATA']['pc_augm']
        self._depth_augm_config = global_config['DATA']['depth_augm']
        self._return_segmap = False

        use_uniform_quaternions = global_config['DATA']['use_uniform_quaternions']
        intrinsics=global_config['DATA']['intrinsics']
        elevation=global_config['DATA']['view_sphere']['elevation']

        self._current_pc = None
        self._cache = {}

        self._renderer = SceneRenderer(caching=True, intrinsics=intrinsics)

        if use_uniform_quaternions:
            raise NotImplementedError
            quat_path = os.path.join(self._root_folder, 'uniform_quaternions/data2_4608.qua')
            quaternions = [l[:-1].split('\t') for l in open(quat_path, 'r').readlines()]

            quaternions = [[float(t[0]),
                            float(t[1]),
                            float(t[2]),
                            float(t[3])] for t in quaternions]
            quaternions = np.asarray(quaternions)
            quaternions = np.roll(quaternions, 1, axis=1)
            self._all_poses = [tra.quaternion_matrix(q) for q in quaternions]
        else:
            self._cam_orientations = []
            self._elevation = np.array(elevation)/180.
            for az in np.linspace(0, np.pi * 2, 30):
                for el in np.linspace(self._elevation[0], self._elevation[1], 30):
                    self._cam_orientations.append(tra.euler_matrix(0, -el, az))
            self._coordinate_transform = tra.euler_matrix(np.pi/2, 0, 0).dot(tra.euler_matrix(0, np.pi/2, 0))

        self._num_renderable_cam_poses = len(self._cam_orientations)
        self._num_saved_cam_poses = 200  # TODO: Make this a config


    def __getitem__(self, index):
        """
        Loads a scene and (potentially) renders a point cloud.

        Data is indexed as follows:
            For each scene, we have a set of camera poses.  The scene index is
            index // num_cam_poses, the camera pose index is index % num_cam_poses

        Arguments:
            index {int} -- scene_pose_index.

        Returns:
            dict -- data dictionary"""

        # Offset index to skip train samples
        if not self.train:
            index = index + self._num_train_scenes

        scene_index = index
        cam_pose_index = random.randint(0, self._num_saved_cam_poses)

        if self._estimate_normals:
            raise NotImplementedError

        if self._return_segmap:
            raise NotImplementedError

        estimate_normals = False

        # -- Load Scene Information -- #
        # scene_index = index // self._num_cam_poses
        # cam_pose_index = index % self._num_cam_poses
        scene_id = self.valid_scene_contacts[scene_index]

        scene_info = self._load_contacts(scene_id)

        scene_contact_points = scene_info['scene_contact_points']
        obj_paths = scene_info['obj_paths']
        obj_transforms = scene_info['obj_transforms']
        obj_scales = scene_info['obj_scales']
        grasp_transforms = scene_info['grasp_transforms']

        # -- Load Scene -- #
        if self.use_saved_renders:
            cam_pose_id = f'{cam_pose_index:03}'
            scene_dir = os.path.join(self._root_folder, 'renders', scene_id)
            render_path = os.path.join(scene_dir, f'{cam_pose_id}.npz')
            if os.path.exists(render_path):
                try:
                    pc_cam, camera_pose = self.load_scene(render_path)
                    pc_normals = np.array([])
                    depth = np.array([])
                except:
                    print('Loading error, re-rendering')
                    # Remove and re-render if the file is corrupted
                    os.remove(render_path)
                    return self.__getitem__(index)
            else:
                if not os.path.exists(scene_dir):
                    os.mkdir(scene_dir)
                # TODO: Fix this in the paths
                # for i in range(obj_paths.shape[0]):
                #     parts = obj_paths[i].split('/')
                #     obj_paths[i] = os.path.join(*parts[0:-2], parts[-1])

                # -- Build and Render Scene -- #
                obj_paths = [os.path.join(self._root_folder, p) for p in obj_paths]
                try:
                    self.change_scene(obj_paths, obj_scales, obj_transforms, visualize=False)
                except:
                    print('Error loading scene %s' % scene_id)
                    return self.__getitem__(self._generate_new_idx())
                pc_cam, pc_normals, camera_pose, depth = self.render_scene(estimate_normals=estimate_normals, augment=False)
                camera_pose, pc_cam = self._center_pc_convert_cam(camera_pose, pc_cam)
                np.savez_compressed(render_path, pc_cam=pc_cam, camera_pose=camera_pose)
                pc_cam = self._augment_pc(pc_cam)

        # -- Or Render Scene without Saving -- #
        else:
            # TODO: Fix this in the paths
            # for i in range(obj_paths.shape[0]):
            #     parts = obj_paths[i].split('/')
            #     obj_paths[i] = os.path.join(*parts[0:-2], parts[-1])

            # -- Build and Render Scene -- #
            obj_paths = [os.path.join(self._root_folder, p) for p in obj_paths]
            try:
                self.change_scene(obj_paths, obj_scales, obj_transforms, visualize=False)
            except:
                print('Error loading scene %s' % scene_id)
                return self.__getitem__(self._generate_new_idx())
            pc_cam, pc_normals, camera_pose, depth = self.render_scene(estimate_normals=estimate_normals)

            # Convert from OpenGL to OpenCV Coordinates
            camera_pose, pc_cam = self._center_pc_convert_cam(camera_pose, pc_cam)

        pc_cam = torch.from_numpy(pc_cam).type(torch.float32)
        pc_normals = torch.tensor(pc_normals).type(torch.float32) # Rn this is []
        camera_pose = torch.from_numpy(camera_pose).type(torch.float32)
        depth = torch.from_numpy(depth).type(torch.float32)

        try:
            pos_contact_points, pos_contact_dirs, pos_finger_diffs, \
                pos_approach_dirs = self._process_contacts(scene_contact_points, grasp_transforms)
        except ValueError:
            print('No positive contacts found')
            print('Scene id: %s' % scene_id)
            return self.__getitem__(self._generate_new_idx())


        pos_contact_points = pos_contact_points
        pos_contact_dirs = pos_contact_dirs
        pos_finger_diffs = pos_finger_diffs
        pos_approach_dirs = pos_approach_dirs

        data = dict(
            pc_cam=pc_cam,
            camera_pose=camera_pose,
            pos_contact_points=pos_contact_points,
            pos_contact_dirs=pos_contact_dirs,
            pos_finger_diffs=pos_finger_diffs,
            pos_approach_dirs=pos_approach_dirs,
        )

        return data

    def __len__(self):
        """
        Returns the number of rendered scenes in the dataset.

        Returns:
            int -- self._num_train_scenes * len(self._cam_orientations)
        """
        if self.train:
            return self._num_train_scenes
        else:
            return self._num_test_scenes

    def _generate_new_idx(self):
        """
        Randomly generates a new index for the dataset.

        Used if the current index is invalid (e.g. no positive contacts or failed to load)
        """
        if self.train:
            return torch.randint(self._num_train_scenes, (1,))[0]
        else:
            return torch.randint(self._num_test_scenes, (1,))[0]

    def _center_pc_convert_cam(self, cam_pose, point_clouds):
        """
        Converts from OpenGL to OpenCV coordinates, computes inverse of camera pose and centers point cloud

        :param cam_poses: (bx4x4) Camera poses in OpenGL format
        :param batch_data: (bxNx3) point clouds
        :returns: (cam_poses, batch_data) converted
        """

        # OpenCV OpenGL conversion
        cam_pose[:3, 1] = -cam_pose[:3, 1]
        cam_pose[:3, 2] = -cam_pose[:3, 2]
        cam_pose = utils.inverse_transform(cam_pose)

        pc_mean = np.mean(point_clouds, axis=0, keepdims=True)
        point_clouds[:,:3] -= pc_mean[:,:3]
        cam_pose[:3,3] -= pc_mean[0,:3]

        return cam_pose, point_clouds

    def _check_scene_contacts(self, dataset_folder, scene_contacts_path='scene_contacts_new', debug=False):
        """
        Attempt to load each scene contact in the scene_contacts_path directory.
        Returns a list of all valid paths

        Arguments:
            dataset_folder {str} -- path to dataset folder
            scene_contacts_path {str} -- path to scene contacts

        Returns:
            list(str) -- list of valid scene contact paths
        """
        scene_contacts_dir = os.path.join(dataset_folder, scene_contacts_path)
        scene_contacts_paths = sorted(glob.glob(os.path.join(scene_contacts_dir, '*')))
        valid_scene_contacts = []

        # Faster
        corrupt_list = [
            # Won't load
            '001780',
            '002100',
            '002486',
            '004861',
            '004866',
            '007176',
            '008283',

            # Newer won't load
            '005723'
            '001993',
            '001165',
            '006828',
            '007238',
            '008980',
            '005623',
            '001993',

            # No contacts
            '008509',
            '007996',
            '009296',
        ]

        for contact_path in scene_contacts_paths:
            if contact_path.split('/')[-1].split('.')[0] in corrupt_list:
                continue
            contact_id = contact_path.split('/')[-1].split('.')[0]
            valid_scene_contacts.append(contact_id)

        return scene_contacts_dir, valid_scene_contacts

    def _load_contacts(self, scene_id):
        """
        Loads contact information for a given scene
        """
        return np.load(os.path.join(self.scene_contacts_dir, scene_id + '.npz'), allow_pickle=True)

    def _process_contacts(self, scene_contact_points, grasp_transforms):
        """
        Processes contact information for a given scene

        num_contacts may not be the same for each scene.  We resample to make this
        the same

        Arguments:
            scene_contact_points {np.ndarray} -- (num_contacts, 2, 3) array of contact points
            grasp_transforms {np.ndarray} -- (num_contacts, 4, 4) array of grasp transforms

        Returns:
            Scene data
        """
        num_pos_contacts = self.global_config['DATA']['labels']['num_pos_contacts']

        contact_directions_01 = scene_contact_points[:,0,:] - scene_contact_points[:,1,:]
        all_contact_points = scene_contact_points.reshape(-1, 3)
        all_finger_diffs = np.maximum(np.linalg.norm(contact_directions_01,axis=1), np.finfo(np.float32).eps)
        all_contact_directions = np.empty((contact_directions_01.shape[0]*2, contact_directions_01.shape[1],))
        all_contact_directions[0::2] = -contact_directions_01 / all_finger_diffs[:,np.newaxis]
        all_contact_directions[1::2] = contact_directions_01 / all_finger_diffs[:,np.newaxis]
        all_contact_suc = np.ones_like(all_contact_points[:,0])
        all_grasp_transform = grasp_transforms.reshape(-1,4,4)
        all_approach_directions = all_grasp_transform[:,:3,2]

        pos_idcs = np.where(all_contact_suc>0)[0]
        if len(pos_idcs) == 0:
            raise ValueError('No positive contacts found')
        # print('total positive contact points ', len(pos_idcs))

        all_pos_contact_points = all_contact_points[pos_idcs]
        all_pos_finger_diffs = all_finger_diffs[pos_idcs//2]
        all_pos_contact_dirs = all_contact_directions[pos_idcs]
        all_pos_approach_dirs = all_approach_directions[pos_idcs//2]

        # -- Sample Positive Contacts -- #
        # Use all positive contacts then mesh_utils with replacement
        if num_pos_contacts > len(all_pos_contact_points)/2:
            pos_sampled_contact_idcs = np.arange(len(all_pos_contact_points))
            pos_sampled_contact_idcs_replacement = np.random.choice(
                np.arange(len(all_pos_contact_points)),
                num_pos_contacts*2 - len(all_pos_contact_points),
                replace=True)
            pos_sampled_contact_idcs= np.hstack((pos_sampled_contact_idcs,
                                                 pos_sampled_contact_idcs_replacement))
        else:
            pos_sampled_contact_idcs = np.random.choice(
                np.arange(len(all_pos_contact_points)),
                num_pos_contacts*2,
                replace=False)

        pos_contact_points = torch.from_numpy(all_pos_contact_points[pos_sampled_contact_idcs,:]).type(torch.float32)

        pos_contact_dirs = torch.from_numpy(all_pos_contact_dirs[pos_sampled_contact_idcs,:]).type(torch.float32)
        pos_contact_dirs = F.normalize(pos_contact_dirs, p=2, dim=1)

        pos_finger_diffs = torch.from_numpy(all_pos_finger_diffs[pos_sampled_contact_idcs]).type(torch.float32)

        pos_approach_dirs = torch.from_numpy(all_pos_approach_dirs[pos_sampled_contact_idcs]).type(torch.float32)
        pos_approach_dirs = F.normalize(pos_approach_dirs, p=2, dim=1)

        return pos_contact_points, pos_contact_dirs, pos_finger_diffs, pos_approach_dirs
        # return tf_pos_contact_points, tf_pos_contact_dirs, tf_pos_contact_approaches, tf_pos_finger_diffs, tf_scene_idcs

    def get_cam_pose(self, cam_orientation):
        """
        Samples camera pose on shell around table center

        Arguments:
            cam_orientation {np.ndarray} -- 3x3 camera orientation matrix

        Returns:
            [np.ndarray] -- 4x4 homogeneous camera pose
        """

        distance = self._distance_range[0] + np.random.rand()*(self._distance_range[1]-self._distance_range[0])

        extrinsics = np.eye(4)
        extrinsics[0, 3] += distance
        extrinsics = cam_orientation.dot(extrinsics)

        cam_pose = extrinsics.dot(self._coordinate_transform)
        # table height
        cam_pose[2,3] += self._renderer._table_dims[2]
        cam_pose[:3,:2]= -cam_pose[:3,:2]
        return cam_pose

    def _augment_pc(self, pc):
        """
        Augments point cloud with jitter and dropout according to config

        Arguments:
            pc {np.ndarray} -- Nx3 point cloud

        Returns:
            np.ndarray -- augmented point cloud
        """

        # not used because no artificial occlusion
        if 'occlusion_nclusters' in self._pc_augm_config and self._pc_augm_config['occlusion_nclusters'] > 0:
            pc = self.apply_dropout(pc,
                                    self._pc_augm_config['occlusion_nclusters'],
                                    self._pc_augm_config['occlusion_dropout_rate'])

        if 'sigma' in self._pc_augm_config and self._pc_augm_config['sigma'] > 0:
            pc = provider.jitter_point_cloud(pc[np.newaxis, :, :],
                                            sigma=self._pc_augm_config['sigma'],
                                            clip=self._pc_augm_config['clip'])[0]


        return pc[:,:3]

    def _augment_depth(self, depth):
        """
        Augments depth map with z-noise and smoothing according to config

        Arguments:
            depth {np.ndarray} -- depth map

        Returns:
            np.ndarray -- augmented depth map
        """

        if 'sigma' in self._depth_augm_config and self._depth_augm_config['sigma'] > 0:
            clip = self._depth_augm_config['clip']
            sigma = self._depth_augm_config['sigma']
            noise = np.clip(sigma*np.random.randn(*depth.shape), -clip, clip)
            depth += noise
        if 'gaussian_kernel' in self._depth_augm_config and self._depth_augm_config['gaussian_kernel'] > 0:
            kernel = self._depth_augm_config['gaussian_kernel']
            depth_copy = depth.copy()
            depth = cv2.GaussianBlur(depth,(kernel,kernel),0)
            depth[depth_copy==0] = depth_copy[depth_copy==0]

        return depth

    def apply_dropout(self, pc, occlusion_nclusters, occlusion_dropout_rate):
        """
        Remove occlusion_nclusters farthest points from point cloud with occlusion_dropout_rate probability

        Arguments:
            pc {np.ndarray} -- Nx3 point cloud
            occlusion_nclusters {int} -- noof cluster to remove
            occlusion_dropout_rate {float} -- prob of removal

        Returns:
            [np.ndarray] -- N > Mx3 point cloud
        """
        if occlusion_nclusters == 0 or occlusion_dropout_rate == 0.:
            return pc

        labels = utils.farthest_points(pc, occlusion_nclusters, utils.distance_by_translation_point)

        removed_labels = np.unique(labels)
        removed_labels = removed_labels[np.random.rand(removed_labels.shape[0]) < occlusion_dropout_rate]
        if removed_labels.shape[0] == 0:
            return pc
        mask = np.ones(labels.shape, labels.dtype)
        for l in removed_labels:
            mask = np.logical_and(mask, labels != l)
        return pc[mask]


    def render_scene(self, estimate_normals=False, camera_pose=None, augment=True):
        """
        Renders scene depth map, transforms to regularized pointcloud and applies augmentations

        Keyword Arguments:
            estimate_normals {bool} -- calculate and return normals (default: {False})
            camera_pose {[type]} -- camera pose to render the scene from. (default: {None})

        Returns:
            [pc, pc_normals, camera_pose, depth] -- [point cloud, point cloud normals, camera pose, depth]
        """
        if camera_pose is None:
            viewing_index = np.random.randint(0, high=len(self._cam_orientations))
            camera_orientation = self._cam_orientations[viewing_index]
            camera_pose = self.get_cam_pose(camera_orientation)

        in_camera_pose = copy.deepcopy(camera_pose)

        # 0.005 s
        _, depth, _, camera_pose = self._renderer.render(in_camera_pose, render_pc=False)
        depth = self._augment_depth(depth)

        pc = self._renderer._to_pointcloud(depth)
        pc = utils.regularize_pc_point_count(pc, self._raw_num_points, use_farthest_point=self._use_farthest_point)
        if augment:
            pc = self._augment_pc(pc)

        pc_normals = utils.estimate_normals_cam_from_pc(pc[:,:3], raw_num_points=self._raw_num_points) if estimate_normals else []

        return pc, pc_normals, camera_pose, depth

    def load_scene(self, render_path):
        """
        Return point cloud and camera pose.  Used for loading saved renders.
        Arguments:
            scene_id {str} -- scene index
            cam_pose_id {str} -- camera pose index as length 3 string with
                                leading zeros if necessary.

        Returns:
            [pc, camera_pose] -- [point cloud, camera pose]
            or returns False if not found
        """
        # print('Loading: ', render_path)
        data = np.load(render_path, allow_pickle=True)
        pc_cam = data['pc_cam']
        camera_pose = data['camera_pose']
        pc_cam_aug = self._augment_pc(pc_cam)
        return pc_cam_aug, camera_pose

    def change_object(self, cad_path, cad_scale):
        """
        Change object in pyrender scene

        Arguments:
            cad_path {str} -- path to CAD model
            cad_scale {float} -- scale of CAD model
        """

        self._renderer.change_object(cad_path, cad_scale)

    def change_scene(self, obj_paths, obj_scales, obj_transforms, visualize=False):
        """
        Change pyrender scene

        Arguments:
            obj_paths {list[str]} -- path to CAD models in scene
            obj_scales {list[float]} -- scales of CAD models
            obj_transforms {list[np.ndarray]} -- poses of CAD models

        Keyword Arguments:
            visualize {bool} -- whether to update the visualizer as well (default: {False})
        """
        self._renderer.change_scene(obj_paths, obj_scales, obj_transforms)
        if visualize:
            self._visualizer.change_scene(obj_paths, obj_scales, obj_transforms)

    def __del__(self):
        print('********** terminating renderer **************')


