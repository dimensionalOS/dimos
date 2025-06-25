# NO LONGER USED


# Pre-render all the camera poses and scenes for the acronym dataset
import os
import copy
import random
import glob
import numpy as np
import trimesh.transformations as tra
from tqdm import tqdm
from multiprocessing import Pool, cpu_count, current_process,shared_memory


from contact_graspnet_pytorch import config_utils, utils
from contact_graspnet_pytorch.datagen_scene_renderer import SceneRenderer

os.environ['PYOPENGL_PLATFORM'] = 'egl'  # To get pyrender to work 

class AcronymRenderScene():
    def __init__(self, global_config):
        """
        Class to render acronym dataset
        """
        self.data_path = global_config['DATA']['data_path']
        self.scene_contacts_dir, self.valid_scene_contacts = \
        self._check_scene_contacts(self.data_path,
                                    global_config['DATA']['scene_contacts_path'])

        self.intrinsics=global_config['DATA']['intrinsics']
        elevation=global_config['DATA']['view_sphere']['elevation']

        self._raw_num_points = global_config['DATA']['raw_num_points']
        self._use_farthest_point = global_config['DATA']['use_farthest_point']
        self._distance_range = global_config['DATA']['view_sphere']['distance_range']
        

        self._current_pc = None
        self._cache = {}

        # self._renderer = SceneRenderer(caching=False, intrinsics=intrinsics)

        elevation = global_config['DATA']['view_sphere']['elevation']
        self._cam_orientations = []
        self._elevation = np.array(elevation)/180.
        for az in np.linspace(0, np.pi * 2, 30):
            for el in np.linspace(self._elevation[0], self._elevation[1], 30):
                self._cam_orientations.append(tra.euler_matrix(0, -el, az))
        self._coordinate_transform = tra.euler_matrix(np.pi/2, 0, 0).dot(tra.euler_matrix(0, np.pi/2, 0))

        self.num_cam_poses = len(self._cam_orientations)

        self._estimate_normals = global_config['DATA']['input_normals']
        self._return_segmap = False


    def render_scene(self, scene_id):
        """
        Render a scene at all poses
        """
        # Data is indexed as follows:
        # For each scene, we have a set of camera poses.  The scene index is
        # index // num_cam_poses, the camera pose index is index % num_cam_poses


        # Get lowest id not in use
        tqdm_id = int(scene_id) + 1

        renderer = SceneRenderer(caching=False, intrinsics=self.intrinsics)

        if self._estimate_normals:
            raise NotImplementedError

        if self._return_segmap:
            raise NotImplementedError

        scene_info = np.load(os.path.join(self.scene_contacts_dir, scene_id + '.npz'), allow_pickle=False)

        scene_contact_points = scene_info['scene_contact_points']
        obj_paths = scene_info['obj_paths']
        obj_transforms = scene_info['obj_transforms']
        obj_scales = scene_info['obj_scales']
        grasp_transforms = scene_info['grasp_transforms']

        render_dir_path = os.path.join(self.data_path, 'renders')

        # TODO: Fix this in the paths
        # for i in range(obj_paths.shape[0]):
        #     parts = obj_paths[i].split('/')
        #     obj_paths[i] = os.path.join(*parts[0:-2], parts[-1])

        # -- Build and Render Scene -- #
        obj_paths = [os.path.join(self.data_path, p) for p in obj_paths]
        self.change_scene(renderer, obj_paths, obj_scales, obj_transforms, visualize=False)
        # try:
        #     self.change_scene(renderer, obj_paths, obj_scales, obj_transforms, visualize=False)
        # except:
        #     print('Error loading scene %s' % scene_id)
        #     return

        if not os.path.exists(os.path.join(render_dir_path, scene_id)):
            os.makedirs(os.path.join(render_dir_path, scene_id))
        pbar = tqdm(self._cam_orientations, position=tqdm_id)
        # pbar = self._cam_orientations
        for i, cam_ori in enumerate(pbar):
            save_pose_path = os.path.join(render_dir_path, scene_id, f'{i:03}.npz')
            if os.path.exists(save_pose_path):
                continue
            camera_pose = self.get_cam_pose(renderer, cam_ori)
            pc_cam, pc_normals, camera_pose, depth = self.render_pose(renderer, camera_pose, estimate_normals=self._estimate_normals)

            # Convert from OpenGL to OpenCV Coordinates
            camera_pose, pc_cam = self._center_pc_convert_cam(camera_pose, pc_cam)

            np.savez(save_pose_path, pc_cam=pc_cam, camera_pose=camera_pose)
        
    
    def change_scene(self, renderer, obj_paths, obj_scales, obj_transforms, visualize=False):
        """
        Change pyrender scene

        Arguments:
            obj_paths {list[str]} -- path to CAD models in scene
            obj_scales {list[float]} -- scales of CAD models
            obj_transforms {list[np.ndarray]} -- poses of CAD models

        Keyword Arguments:
            visualize {bool} -- whether to update the visualizer as well (default: {False})
        """
        renderer.change_scene(obj_paths, obj_scales, obj_transforms)
        if visualize:
            self._visualizer.change_scene(obj_paths, obj_scales, obj_transforms)


    def render_pose(self, renderer, camera_pose, estimate_normals=False):
        """
        Renders scene depth map, transforms to regularized pointcloud.
        Note, we do not apply augmentations

        Arguments:
            camera_pose {[type]} -- camera pose to render the scene from. (default: {None})

        Keyword Arguments:
            estimate_normals {bool} -- calculate and return normals (default: {False})

        Returns:
            [pc, pc_normals, camera_pose, depth] -- [point cloud, point cloud normals, camera pose, depth]
        """
        in_camera_pose = copy.deepcopy(camera_pose)

        # 0.005 s
        _, depth, _, camera_pose = renderer.render(in_camera_pose, render_pc=False)
        # depth = self._augment_depth(depth)

        pc = renderer._to_pointcloud(depth)
        pc = self._regularize_pc_point_count(pc, self._raw_num_points, use_farthest_point=self._use_farthest_point)
        # pc = self._augment_pc(pc)

        if estimate_normals:
            raise NotImplementedError
        else:
            pc_normals = []
        # pc_normals = estimate_normals_cam_from_pc(pc[:,:3], raw_num_points=self._raw_num_points) if estimate_normals else []

        return pc, pc_normals, camera_pose, depth
    
    def get_cam_pose(self, renderer, cam_orientation):
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
        cam_pose[2,3] += renderer._table_dims[2]
        cam_pose[:3,:2]= -cam_pose[:3,:2]
        return cam_pose
    
    def _check_scene_contacts(self, dataset_folder, scene_contacts_path='scene_contacts_new'):
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

            # No contacts
            '008509',  
            '007996',
            '009296',

            # '001439',
            # '006090',
            # '002731',
            # '001519',
        ]


        # Just check if each scene is in the bad list
        for contact_path in scene_contacts_paths:
            if contact_path.split('/')[-1].split('.')[0] in corrupt_list:
                continue
            contact_id = contact_path.split('/')[-1].split('.')[0]
            valid_scene_contacts.append(contact_id)
        return scene_contacts_dir, valid_scene_contacts
    
    @staticmethod
    def _regularize_pc_point_count(pc, npoints, use_farthest_point=False):
        """
        If point cloud pc has less points than npoints, it oversamples.
        Otherwise, it downsample the input pc to have npoint points.
        use_farthest_point: indicates

        :param pc: Nx3 point cloud
        :param npoints: number of points the regularized point cloud should have
        :param use_farthest_point: use farthest point sampling to downsample the points, runs slower.
        :returns: npointsx3 regularized point cloud
        """

        def distance_by_translation_point(p1, p2):
            """
            Gets two nx3 points and computes the distance between point p1 and p2.
            """
            return np.sqrt(np.sum(np.square(p1 - p2), axis=-1))

        if pc.shape[0] > npoints:
            if use_farthest_point:
                _, center_indexes = utils.farthest_points(pc, npoints, distance_by_translation_point, return_center_indexes=True)
            else:
                center_indexes = np.random.choice(range(pc.shape[0]), size=npoints, replace=False)
            pc = pc[center_indexes, :]
        else:
            required = npoints - pc.shape[0]
            if required > 0:
                index = np.random.choice(range(pc.shape[0]), size=required)
                pc = np.concatenate((pc, pc[index, :]), axis=0)
        return pc

    @staticmethod
    def _center_pc_convert_cam(cam_pose, point_clouds):
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
    

# -- Multiprocessing rendering functions -- #

def render_scene(data):
    scene_id, global_config = data
    acronym_render = AcronymRenderScene(global_config)
    acronym_render.render_scene(scene_id)

def check_scene_contacts(dataset_folder, scene_contacts_path='scene_contacts_new'):
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

        # No contacts
        '008509',  
        '007996',
        '009296',

        # '001439',
        # '006090',
        # '002731',
        # '001519',
    ]


    # Just check if each scene is in the bad list
    for contact_path in scene_contacts_paths:
        if contact_path.split('/')[-1].split('.')[0] in corrupt_list:
            continue
        contact_id = contact_path.split('/')[-1].split('.')[0]
        valid_scene_contacts.append(contact_id)
    return scene_contacts_dir, valid_scene_contacts

def render_acronym(global_config):
    """
    Render all the scenes in the acronym dataset
    """
    # for scene_id in tqdm(self.valid_scene_contacts):
    #     self.render_scene(scene_id)

    data_path = global_config['DATA']['data_path']
    scene_contacts_dir, valid_scene_contacts = check_scene_contacts(data_path,
                                global_config['DATA']['scene_contacts_path'])
    
    data_list = [(id, global_config) for id in valid_scene_contacts]

    n_pools = min(10, cpu_count()-2)
    with Pool(n_pools) as p:
        list(tqdm(
            p.imap_unordered(render_scene, data_list), 
                            total=len(valid_scene_contacts), 
                            position=0))


if __name__ == '__main__':

    ckpt_dir = 'checkpoints/contact_graspnet'
    forward_passes = 1
    arg_configs = []

    # config_dir = 'contact_graspnet_pytorch'
    global_config = config_utils.load_config(ckpt_dir, batch_size=forward_passes, arg_configs=arg_configs)

    render_acronym(global_config)


    # render.render_scene('000001')