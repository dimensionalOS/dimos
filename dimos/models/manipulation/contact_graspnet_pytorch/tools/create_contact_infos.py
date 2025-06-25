import os
import numpy as np
import h5py
import glob
import argparse
import sys
import tqdm
from multiprocessing import Pool, cpu_count

ROOT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(ROOT_DIR)

from contact_graspnet_pytorch.data import PointCloudReader
from contact_graspnet_pytorch.mesh_utils import in_collision_with_gripper, grasp_contact_location

def grasps_contact_info(grasp_tfs, successfuls, obj_mesh, check_collisions=True):
    """
    Check the collision of the grasps and compute contact points, normals and directions

    Arguments:
        grasp_tfs {np.ndarray} -- Mx4x4 grasp transformations
        successfuls {np.ndarray} -- Binary Mx1 successful grasps
        obj_mesh {trimesh.base.Trimesh} -- Mesh of the object

    Keyword Arguments:
        check_collisions {bool} -- whether to check for collisions (default: {True})

    Returns:
        [dict] -- object contact dictionary with all necessary information
    """
    print('evaluating {} grasps'.format(len(grasp_tfs)))
    if check_collisions:
        collisions, _ = in_collision_with_gripper(
            obj_mesh,
            grasp_tfs,
            gripper_name='panda',
            silent=True,
        )
    contact_dicts = grasp_contact_location(
        grasp_tfs,
        successfuls,
        collisions if check_collisions else [0]*len(successfuls),
        object_mesh=obj_mesh,
        gripper_name='panda',
        silent=True,
    )

    return contact_dicts

def read_object_grasp_data_acronym(root_folder, h5_path):
    """
    Read object grasp data from the acronym dataset and loads mesh

    Arguments:
        root_folder {str} -- root folder of acronym dataset
        h5_path {str} -- relative path to grasp h5 file

    Returns:
        [grasps, success, cad_path, cad_scale] -- grasp trafos, grasp success, absolute mesh path, mesh scale
    """
    
    abs_h5_path = os.path.join(root_folder, 'grasps', h5_path)
    data = h5py.File(abs_h5_path, "r")
    mesh_fname = os.path.join(root_folder, data["object/file"][()].decode('utf-8'))
    mesh_scale = data["object/scale"][()]
    grasps = np.array(data["grasps/transforms"])
    success = np.array(data["grasps/qualities/flex/object_in_gripper"])

    positive_grasps = grasps[success==1, :, :]
    negative_grasps = grasps[success==0, :, :]

    print('positive grasps: {} negative grasps: {}'.format(positive_grasps.shape, negative_grasps.shape))

    return grasps, success, mesh_fname, mesh_scale

def save_contact_data(pcreader, grasp_path, target_path='mesh_contacts'):
    """
    Maps acronym grasp data to contact information on meshes and saves them as npz file

    Arguments:
        grasp_path {str} -- path to grasp json file 
        pcreader {Object} -- PointCloudReader instance from data.py 

    """

    target_path = os.path.join(pcreader._root_folder, target_path)
    if not os.path.exists(target_path):
        os.makedirs(target_path)
    
    contact_dir_path = os.path.join(target_path, os.path.basename(grasp_path.replace('.h5','.npz')))
    if not os.path.exists(os.path.dirname(contact_dir_path)):
        os.makedirs(os.path.dirname(contact_dir_path))
    if os.path.exists(contact_dir_path):
        return

    output_grasps, output_labels, cad_path, cad_scale = read_object_grasp_data_acronym(pcreader._root_folder, grasp_path)
    # pcreader.change_object(cad_path, cad_scale)
    pcreader.change_scene([cad_path], [cad_scale], [np.eye(4)])

    context = pcreader._renderer._cache[(cad_path,cad_scale)]
    obj_mesh = context['tmesh']
    obj_mesh_mean = context['mesh_mean']

    output_grasps[:,:3,3] -= obj_mesh_mean
    contact_dicts = grasps_contact_info(output_grasps, list(output_labels), obj_mesh, check_collisions=False)

    contact_dict_of_arrays = {}
    for d in contact_dicts:
        for k in d:
            contact_dict_of_arrays.setdefault(k,[]).append(d[k])

    np.savez(contact_dir_path, **contact_dict_of_arrays)

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Grasp data reader")
    parser.add_argument('root_folder', help='Root dir with acronym grasps, meshes and splits', type=str)
    args = parser.parse_args()
    print('Root folder', args.root_folder)

    pcreader = PointCloudReader(root_folder=args.root_folder)

    grasp_paths = glob.glob(os.path.join(args.root_folder, 'grasps', '*.h5'))
    grasp_paths = [grasp_path.split('/')[-1] for grasp_path in grasp_paths]

    # -- Remove objects that failed to waterproof -- #
    with open(os.path.join(args.root_folder, 'failed.txt'), 'r') as f:
        failed_meshes = [line.strip() for line in f.readlines()]
    
    good_grasp_paths = []
    for grasp_path in grasp_paths:
        shapenet_id = grasp_path.split('/')[-1].split('_')[1]
        if shapenet_id not in failed_meshes:
            good_grasp_paths.append(grasp_path)
        else:
            print('Skipping failed mesh', shapenet_id)
    
    grasp_paths = good_grasp_paths
        
    print('Computing grasp contacts...')
    for grasp_path in tqdm.tqdm(grasp_paths):
        print('Reading: ', grasp_path)
        save_contact_data(pcreader, grasp_path)

    # def save_contact_data_wrapper(grasp_path):
    #     save_contact_data(pcreader, grasp_path)
    
    # with Pool(cpu_count() - 4) as p:
    #     examples = list(
    #         tqdm.tqdm(
    #             p.imap_unordered(save_contact_data_wrapper,
    #                              grasp_paths),
    #             total=len(grasp_paths)
    #         )
    #     )