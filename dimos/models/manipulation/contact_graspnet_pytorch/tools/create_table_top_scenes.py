import glob
import os
import random
import trimesh
import numpy as np
import json
import argparse
import signal
import trimesh.transformations as tra

from acronym_tools import Scene, load_mesh, create_gripper_marker

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

def load_contacts(root_folder, data_splits, splits=['train'], min_pos_contacts=1):
    """
    Load grasps and contacts into memory

    Arguments:
        root_folder {str} -- path to acronym data
        data_splits {dict} -- dict of categories of train/test object grasp files

    Keyword Arguments:
        splits {list} -- object data split(s) to use for scene generation
        min_pos_contacts {int} -- minimum successful grasp contacts to consider object

    Returns:
        [dict] -- h5 file names as keys and grasp transforms + contact points as values
    """
    contact_infos = {}
    for category_paths in data_splits.values():
        for split in splits:
            for grasp_path in category_paths[split]:
                contact_path = os.path.join(root_folder, 'mesh_contacts', grasp_path.replace('.h5','.npz'))
                if os.path.exists(contact_path):
                    npz = np.load(contact_path)
                    if 'contact_points' in npz:
                        all_contact_suc = npz['successful'].reshape(-1)
                        pos_idcs = np.where(all_contact_suc>0)[0]
                        if len(pos_idcs) > min_pos_contacts:
                            contact_infos[grasp_path] = {}
                            contact_infos[grasp_path]['successful'] = npz['successful']
                            contact_infos[grasp_path]['grasp_transform'] = npz['grasp_transform']
                            contact_infos[grasp_path]['contact_points'] = npz['contact_points']
    if not contact_infos:
        print('Warning: No mesh_contacts found. Please run tools/create_contact_infos.py first!')
    return contact_infos

def load_splits(root_folder):
    """
    Load splits of training and test objects

    Arguments:
        root_folder {str} -- path to acronym data

    Returns:
        [dict] -- dict of category-wise train/test object grasp files
    """
    split_dict = {}
    split_paths = glob.glob(os.path.join(root_folder, 'splits/*.json'))
    for split_p in split_paths:
        category = os.path.basename(split_p).split('.json')[0]
        splits = json.load(open(split_p,'r'))
        split_dict[category] = {}
        split_dict[category]['train'] = [obj_p.replace('.json', '.h5') for obj_p in splits['train']]
        split_dict[category]['test'] = [obj_p.replace('.json', '.h5') for obj_p in splits['test']]
    return split_dict

class TableScene(Scene):
    """
    Holds current table-top scene, samples object poses and checks grasp collisions.

    Arguments:
        root_folder {str} -- path to acronym data
        gripper_path {str} -- relative path to gripper collision mesh

    Keyword Arguments:
        lower_table {float} -- lower table to permit slight grasp collisions between table and object/gripper (default: {0.02})
    """

    def __init__(self, root_folder, gripper_path, lower_table=0.02, splits=['train']):
        
        super().__init__()
        self.root_folder = root_folder
        self.splits= splits
        self.gripper_mesh = trimesh.load(os.path.join(BASE_DIR, gripper_path))

        self._table_dims = [1.0, 1.2, 0.6]
        self._table_support = [0.6, 0.6, 0.6]
        self._table_pose = np.eye(4)
        self.table_mesh = trimesh.creation.box(self._table_dims)
        self.table_support = trimesh.creation.box(self._table_support)

        self.data_splits = load_splits(root_folder)
        self.category_list = list(self.data_splits.keys())
        self.contact_infos = load_contacts(root_folder, self.data_splits, splits=self.splits)
        
        self._lower_table = lower_table
    
        self._scene_count = 0
        
    def get_random_object(self):
        
        """Return random scaled but not yet centered object mesh

        Returns:
            [trimesh.Trimesh, str] -- ShapeNet mesh from a random category, h5 file path
        """
        
        while True:
            random_category = random.choice(self.category_list)
            cat_obj_paths = [obj_p for split in self.splits for obj_p in self.data_splits[random_category][split]]
            if cat_obj_paths:
                random_grasp_path = random.choice(cat_obj_paths)
                if random_grasp_path in self.contact_infos:
                    break
        
        obj_mesh = load_mesh(os.path.join(self.root_folder, 'grasps', random_grasp_path), self.root_folder)
        
        mesh_mean =  np.mean(obj_mesh.vertices, 0, keepdims=True)
        obj_mesh.vertices -= mesh_mean
        
        return obj_mesh, random_grasp_path
    
    def _get_random_stable_pose(self, stable_poses, stable_poses_probs, thres=0.005):
        """Return a stable pose according to their likelihood.

        Args:
            stable_poses (list[np.ndarray]): List of stable poses as 4x4 matrices.
            stable_poses_probs (list[float]): List of probabilities.
            thres (float): Threshold of pose stability to include for sampling

        Returns:
            np.ndarray: homogeneous 4x4 matrix
        """
        
        
        # Random pose with unique (avoid symmetric poses) stability prob > thres
        _,unique_idcs = np.unique(stable_poses_probs.round(decimals=3), return_index=True)
        unique_idcs = unique_idcs[::-1]
        unique_stable_poses_probs = stable_poses_probs[unique_idcs]
        n = len(unique_stable_poses_probs[unique_stable_poses_probs>thres])
        index = unique_idcs[np.random.randint(n)]
            
        # index = np.random.choice(len(stable_poses), p=stable_poses_probs)
        inplane_rot = tra.rotation_matrix(
            angle=np.random.uniform(0, 2.0 * np.pi), direction=[0, 0, 1]
        )
        return inplane_rot.dot(stable_poses[index])
    
    def find_object_placement(self, obj_mesh, max_iter):
        """Try to find a non-colliding stable pose on top of any support surface.

        Args:
            obj_mesh (trimesh.Trimesh): Object mesh to be placed.
            max_iter (int): Maximum number of attempts to place to object randomly.

        Raises:
            RuntimeError: In case the support object(s) do not provide any support surfaces.

        Returns:
            bool: Whether a placement pose was found.
            np.ndarray: Homogenous 4x4 matrix describing the object placement pose. Or None if none was found.
        """
        support_polys, support_T = self._get_support_polygons()
        if len(support_polys) == 0:
            raise RuntimeError("No support polygons found!")

        # get stable poses for object
        stable_obj = obj_mesh.copy()
        stable_obj.vertices -= stable_obj.center_mass
        stable_poses, stable_poses_probs = stable_obj.compute_stable_poses(
            threshold=0, sigma=0, n_samples=20
        )
        # stable_poses, stable_poses_probs = obj_mesh.compute_stable_poses(threshold=0, sigma=0, n_samples=1)

        # Sample support index
        support_index = max(enumerate(support_polys), key=lambda x: x[1].area)[0]

        iter = 0
        colliding = True
        while iter < max_iter and colliding:

            # Sample position in plane
            pts = trimesh.path.polygons.sample(
                support_polys[support_index], count=1
            )

            # To avoid collisions with the support surface
            pts3d = np.append(pts, 0)

            # Transform plane coordinates into scene coordinates
            placement_T = np.dot(
                support_T[support_index],
                trimesh.transformations.translation_matrix(pts3d),
            )

            pose = self._get_random_stable_pose(stable_poses, stable_poses_probs)

            placement_T = np.dot(
                np.dot(placement_T, pose), tra.translation_matrix(-obj_mesh.center_mass)
            )

            # Check collisions
            colliding = self.is_colliding(obj_mesh, placement_T)

            iter += 1

        return not colliding, placement_T if not colliding else None

    def is_colliding(self, mesh, transform, eps=1e-6):
        """
        Whether given mesh collides with scene

        Arguments:
            mesh {trimesh.Trimesh} -- mesh 
            transform {np.ndarray} -- mesh transform

        Keyword Arguments:
            eps {float} -- minimum distance detected as collision (default: {1e-6})

        Returns:
            [bool] -- colliding or not
        """
        dist = self.collision_manager.min_distance_single(mesh, transform=transform)
        return dist < eps
    
    def load_suc_obj_contact_grasps(self, grasp_path):
        """
        Loads successful object grasp contacts

        Arguments:
            grasp_path {str} -- acronym grasp path

        Returns:
            [np.ndarray, np.ndarray] -- Mx4x4 grasp transforms, Mx3 grasp contacts
        """
        contact_info = self.contact_infos[grasp_path]
        
        suc_grasps = contact_info['successful'].reshape(-1)
        gt_grasps = contact_info['grasp_transform'].reshape(-1,4,4)
        gt_contacts = contact_info['contact_points'].reshape(-1,3)
        
        suc_gt_contacts = gt_contacts[np.repeat(suc_grasps,2)>0]
        suc_gt_grasps = gt_grasps[suc_grasps>0]
        
        return suc_gt_grasps, suc_gt_contacts

    def set_mesh_transform(self, name, transform):
        """
        Set mesh transform for collision manager

        Arguments:
            name {str} -- mesh name
            transform {np.ndarray} -- 4x4 homog mesh pose
        """
        self.collision_manager.set_transform(name, transform)
        self._poses[name] = transform
    
    def save_scene_grasps(self, output_dir, scene_filtered_grasps, scene_filtered_contacts, obj_paths, obj_transforms, obj_scales, obj_grasp_idcs):
        """
        Save scene_contact infos in output_dir

        Arguments:
            output_dir {str} -- absolute output directory
            scene_filtered_grasps {np.ndarray} -- Nx4x4 filtered scene grasps
            scene_filtered_contacts {np.ndarray} -- Nx2x3 filtered scene contacts
            obj_paths {list} -- list of object paths in scene
            obj_transforms {list} -- list of object transforms in scene
            obj_scales {list} -- list of object scales in scene
            obj_grasp_idcs {list} -- list of starting grasp idcs for each object 
        """
        if not os.path.exists(output_dir):
            os.makedirs(output_dir)

        contact_info = {}
        contact_info['obj_paths'] = obj_paths
        contact_info['obj_transforms'] = obj_transforms
        contact_info['obj_scales'] = obj_scales
        contact_info['grasp_transforms'] = scene_filtered_grasps
        contact_info['scene_contact_points'] = scene_filtered_contacts
        contact_info['obj_grasp_idcs'] = np.array(obj_grasp_idcs)
        output_path = os.path.join(output_dir, '{:06d}.npz'.format(self._scene_count))
        while os.path.exists(output_path):
            self._scene_count += 1
            output_path = os.path.join(output_dir, '{:06d}.npz'.format(self._scene_count))
        np.savez(output_path, **contact_info)
        self._scene_count += 1
        
    def _transform_grasps(self, grasps, contacts, obj_transform):
        """
        Transform grasps and contacts into given object transform

        Arguments:
            grasps {np.ndarray} -- Nx4x4 grasps
            contacts {np.ndarray} -- 2Nx3 contacts
            obj_transform {np.ndarray} -- 4x4 mesh pose

        Returns:
            [np.ndarray, np.ndarray] -- transformed grasps and contacts
        """
        transformed_grasps = np.matmul(obj_transform, grasps)
        contacts_homog = np.concatenate((contacts, np.ones((contacts.shape[0], 1))),axis=1)
        transformed_contacts = np.dot(contacts_homog, obj_transform.T)[:,:3]
        return transformed_grasps, transformed_contacts

    def _filter_colliding_grasps(self, transformed_grasps, transformed_contacts):
        """
        Filter out colliding grasps

        Arguments:
            transformed_grasps {np.ndarray} -- Nx4x4 grasps
            transformed_contacts {np.ndarray} -- 2Nx3 contact points

        Returns:
            [np.ndarray, np.ndarray] -- Mx4x4 filtered grasps, Mx2x3 filtered contact points
        """
        filtered_grasps = []
        filtered_contacts = []
        for i,g in enumerate(transformed_grasps):
            if not self.is_colliding(self.gripper_mesh, g):
                filtered_grasps.append(g)
                filtered_contacts.append(transformed_contacts[2*i:2*(i+1)])
        return np.array(filtered_grasps).reshape(-1,4,4), np.array(filtered_contacts).reshape(-1,2,3)
    
    def reset(self):
        """
        Reset, i.e. remove scene objects
        """
        for name in self._objects:
            self.collision_manager.remove_object(name)
        self._objects = {}
        self._poses = {}
        self._support_objects = []
    
    def load_existing_scene(self, path):
        """
        Load an existing scene_contacts scene for visualization

        Arguments:
            path {str} -- abs path to scene_contacts npz file

        Returns:
            [np.ndarray, list, list] -- scene_grasps, list of obj paths, list of object transforms
        """
        self.add_object('table', self.table_mesh, self._table_pose)
        self._support_objects.append(self.table_support)        

        inp = np.load(os.path.join(self.root_folder, path))
        scene_filtered_grasps = inp['grasp_transforms']
        scene_contacts = inp['scene_contact_points']
        obj_transforms = inp['obj_transforms']
        obj_paths = inp['obj_paths']
        obj_scales = inp['obj_scales']

        for obj_path,obj_transform,obj_scale in zip(obj_paths,obj_transforms,obj_scales):
            obj_mesh = trimesh.load(os.path.join(self.root_folder, obj_path))
            obj_mesh = obj_mesh.apply_scale(obj_scale)
            mesh_mean =  np.mean(obj_mesh.vertices, 0, keepdims=True)
            obj_mesh.vertices -= mesh_mean
            self.add_object(obj_path, obj_mesh, obj_transform)
        return scene_filtered_grasps, scene_contacts, obj_paths, obj_transforms
    
    
    def handler(self, signum, frame):
        raise Exception("Could not place object ")
        
    def arrange(self, num_obj, max_iter=100, time_out = 8):
        """
        Arrange random table top scene with contact grasp annotations

        Arguments:
            num_obj {int} -- number of objects

        Keyword Arguments:
            max_iter {int} -- maximum iterations to try placing an object (default: {100})
            time_out {int} -- maximum time to try placing an object (default: {8})

        Returns:
            [np.ndarray, np.ndarray, list, list, list, list] -- 
            scene_filtered_grasps, scene_filtered_contacts, obj_paths, obj_transforms, obj_scales, obj_grasp_idcs

        """

        self._table_pose[2,3] -= self._lower_table
        self.add_object('table', self.table_mesh, self._table_pose)       
        
        self._support_objects.append(self.table_support)    

        obj_paths = []
        obj_transforms = []
        obj_scales = []
        grasp_paths = []
        
        for i in range(num_obj):
            obj_mesh, random_grasp_path = self.get_random_object()
            signal.signal(signal.SIGALRM, self.handler)
            signal.alarm(8)
            try:
                success, placement_T = self.find_object_placement(obj_mesh, max_iter)
            except Exception as exc: 
                print(exc, random_grasp_path, " after {} seconds!".format(time_out))
                continue
            signal.alarm(0)
            if success:
                self.add_object(random_grasp_path, obj_mesh, placement_T)
                obj_scales.append(float(random_grasp_path.split('_')[-1].split('.h5')[0]))
                obj_paths.append(os.path.join('meshes', '/'.join(random_grasp_path.split('_')[:2]) + '.obj'))
                obj_transforms.append(placement_T)
                grasp_paths.append(random_grasp_path)
            else:
                print("Couldn't place object", random_grasp_path, " after {} iterations!".format(max_iter))
        print('Placed {} objects'.format(len(obj_paths)))

        # self.set_mesh_transform('table', self._table_pose)

        scene_filtered_grasps = []
        scene_filtered_contacts = []
        obj_grasp_idcs = []
        grasp_count = 0
        
        for obj_transform, grasp_path in zip(obj_transforms, grasp_paths):
            grasps, contacts = self.load_suc_obj_contact_grasps(grasp_path)
            transformed_grasps, transformed_contacts = self._transform_grasps(grasps, contacts, obj_transform)
            filtered_grasps, filtered_contacts = self._filter_colliding_grasps(transformed_grasps, transformed_contacts)
            
            scene_filtered_grasps.append(filtered_grasps)
            scene_filtered_contacts.append(filtered_contacts)
            grasp_count += len(filtered_contacts)
            obj_grasp_idcs.append(grasp_count)

        scene_filtered_grasps = np.concatenate(scene_filtered_grasps,0)
        scene_filtered_contacts = np.concatenate(scene_filtered_contacts,0)
        
        self._table_pose[2,3] += self._lower_table
        self.set_mesh_transform('table', self._table_pose)

        return scene_filtered_grasps, scene_filtered_contacts, obj_paths, obj_transforms, obj_scales, obj_grasp_idcs
        
    def visualize(self, scene_grasps, scene_contacts=None):
        """
        Visualizes table top scene with grasps

        Arguments:
            scene_grasps {np.ndarray} -- Nx4x4 grasp transforms
            scene_contacts {np.ndarray} -- Nx2x3 grasp contacts
        """
        print('Visualizing scene and grasps.. takes time')
        
        gripper_marker = create_gripper_marker(color=[0, 255, 0])
        gripper_markers = [gripper_marker.copy().apply_transform(t) for t in scene_grasps]
        
        colors = np.ones((scene_contacts.shape[0]*2,4))*255
        colors[:,0:2] = 0
        scene_contact_scene = trimesh.Scene(trimesh.points.PointCloud(scene_contacts.reshape(-1,3), colors=colors))
        
        # show scene together with successful and collision-free grasps of all objects
        trimesh.scene.scene.append_scenes(
            [self.colorize().as_trimesh_scene(), trimesh.Scene(gripper_markers), scene_contact_scene]
        ).show()
        
if __name__ == "__main__":

    parser = argparse.ArgumentParser(description="Grasp data reader")
    parser.add_argument('root_folder', help='Root dir with grasps, meshes, mesh_contacts and splits', type=str)
    parser.add_argument('--num_grasp_scenes', type=int, default=10000)
    parser.add_argument('--splits','--list', nargs='+')
    parser.add_argument('--max_iterations', type=int, default=100)
    parser.add_argument('--gripper_path', type=str, default='gripper_models/panda_gripper/panda_gripper.obj')
    parser.add_argument('--min_num_objects', type=int, default=8)
    parser.add_argument('--max_num_objects', type=int, default=12)
    parser.add_argument('--start_index', type=int, default=0)
    parser.add_argument('--load_existing', type=str, default=None)
    parser.add_argument('--output_dir', type=str, default='scene_contacts')
    parser.add_argument('-vis', action='store_true', default=False)
    args = parser.parse_args()

    root_folder = args.root_folder
    splits = args.splits if args.splits else ['train']
    max_iterations = args.max_iterations
    gripper_path = args.gripper_path
    number_of_scenes = args.num_grasp_scenes
    min_num_objects = args.min_num_objects
    max_num_objects = args.max_num_objects
    start_index = args.start_index
    load_existing = args.load_existing
    output_dir = args.output_dir
    visualize = args.vis

    table_scene = TableScene(root_folder, gripper_path, splits=splits)
    table_scene._scene_count = start_index

    print('Root folder', args.root_folder)
    output_dir = os.path.join(root_folder, output_dir)

    while table_scene._scene_count < number_of_scenes:
        
        table_scene.reset()
                
        if load_existing is None:
            print('generating %s/%s' % (table_scene._scene_count, number_of_scenes))
            num_objects = np.random.randint(min_num_objects,max_num_objects+1)
            scene_grasps, scene_contacts, obj_paths, obj_transforms, obj_scales, obj_grasp_idcs = table_scene.arrange(num_objects, max_iterations)
            if not visualize:
                table_scene.save_scene_grasps(output_dir, scene_grasps, scene_contacts, obj_paths, obj_transforms, obj_scales, obj_grasp_idcs)
        else:
            scene_grasps,scene_contacts, _,_ = table_scene.load_existing_scene(load_existing)
            
        if visualize:
            table_scene.visualize(scene_grasps, scene_contacts)
            table_scene._scene_count +=1