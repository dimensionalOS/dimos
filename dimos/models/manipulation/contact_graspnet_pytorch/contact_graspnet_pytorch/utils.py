import os
import numpy as np
import torch
import cv2

from PIL import Image
from scipy.spatial import cKDTree

def send_dict_to_device(data, device):
    """
    Send all data in dict to torch device
    """
    for k, v in data.items():
        data[k] = v.to(device)
    return None

def index_points(points, idx):
    """
    From pointnet2 repo

    Input:
        points: input points data, [B, N, C]
        idx: sample index data, [B, S]
    Return:
        new_points:, indexed points data, [B, S, C]
    """
    device = points.device
    B = points.shape[0]
    view_shape = list(idx.shape)
    view_shape[1:] = [1] * (len(view_shape) - 1)
    repeat_shape = list(idx.shape)
    repeat_shape[0] = 1
    batch_indices = torch.arange(B, dtype=torch.long).to(device).view(view_shape).repeat(repeat_shape)
    new_points = points[batch_indices, idx, :]
    return new_points

def inverse_transform(trans):
    """
    Computes the inverse of 4x4 transform.

    Arguments:
        trans {np.ndarray} -- 4x4 transform.

    Returns:
        [np.ndarray] -- inverse 4x4 transform
    """
    rot = trans[:3, :3]
    t = trans[:3, 3]
    rot = np.transpose(rot)
    t = -np.matmul(rot, t)
    output = np.zeros((4, 4), dtype=np.float32)
    output[3][3] = 1
    output[:3, :3] = rot
    output[:3, 3] = t

    return output


def build_6d_grasp(approach_dirs, base_dirs, contact_pts, thickness, use_torch=False, gripper_depth = 0.1034, device=None):
    """
    Build 6-DoF grasps + width from point-wise network predictions

    Arguments:
        approach_dirs {np.ndarray/torch.tensor} -- BxNx3 approach direction vectors
        base_dirs {np.ndarray/torch.tensor} -- BxNx3 base direction vectors
        contact_pts {np.ndarray/torch.tensor} -- BxNx3 contact points
        thickness {np.ndarray/torch.tensor} -- BxNx1 grasp width

    Keyword Arguments:
        use_tf {bool} -- whether inputs and outputs are tf tensors (default: {False})
        gripper_depth {float} -- distance from gripper coordinate frame to gripper baseline in m (default: {0.1034})

    Returns:
        np.ndarray / torch.tensor -- BxNx4x4 grasp poses in camera coordinates
    """
    # We are trying to build a stack of 4x4 homogeneous transform matricies of size B x N x 4 x 4.
    # To do so, we calculate the rotation and translation portions according to the paper.
    # This gives us positions as shown:
    # [ R R R T ]
    # [ R R R T ]
    # [ R R R T ]
    # [ 0 0 0 1 ]                    Note that the ^ dim is 2 and the --> dim is 3
    # We need to pad with zeros and ones to get the final shape so we generate
    # ones and zeros and stack them.
    if thickness.ndim == 2:
        thickness = thickness.unsqueeze(2)  # B x N x 1
    if use_torch:
        if device is None:
            device = torch.device('cpu')
        grasp_R = torch.stack([base_dirs, torch.cross(approach_dirs,base_dirs),approach_dirs], dim=3)  # B x N x 3 x 3
        grasp_t = contact_pts + (thickness / 2) * base_dirs - gripper_depth * approach_dirs  # B x N x 3
        grasp_t = grasp_t.unsqueeze(3)  # B x N x 3 x 1
        ones = torch.ones((contact_pts.shape[0], contact_pts.shape[1], 1, 1), dtype=torch.float32).to(device)  # B x N x 1 x 1
        zeros = torch.zeros((contact_pts.shape[0], contact_pts.shape[1], 1, 3), dtype=torch.float32).to(device)  # B x N x 1 x 3
        homog_vec = torch.cat([zeros, ones], dim=3)  # B x N x 1 x 4
        grasps = torch.cat([torch.cat([grasp_R, grasp_t], dim=3), homog_vec], dim=2)  # B x N x 4 x 4

    else:
        raise NotImplementedError("Need to test this more")
        grasps = []
        for i in range(len(contact_pts)):
            grasp = np.eye(4)

            grasp[:3,0] = base_dirs[i] / np.linalg.norm(base_dirs[i])
            grasp[:3,2] = approach_dirs[i] / np.linalg.norm(approach_dirs[i])
            grasp_y = np.cross( grasp[:3,2],grasp[:3,0])
            grasp[:3,1] = grasp_y / np.linalg.norm(grasp_y)
            # base_gripper xyz = contact + thickness / 2 * baseline_dir - gripper_d * approach_dir
            grasp[:3,3] = contact_pts[i] + thickness[i] / 2 * grasp[:3,0] - gripper_depth * grasp[:3,2]
            # grasp[0,3] = finger_width
            grasps.append(grasp)
        grasps = np.array(grasps)

    return grasps

def farthest_points(data, nclusters, dist_func, return_center_indexes=False, return_distances=False, verbose=False):
    """
      Performs farthest point sampling on data points.
      Args:
        data: numpy array of the data points.
        nclusters: int, number of clusters.
        dist_dunc: distance function that is used to compare two data points.
        return_center_indexes: bool, If True, returns the indexes of the center of
          clusters.
        return_distances: bool, If True, return distances of each point from centers.

      Returns clusters, [centers, distances]:
        clusters: numpy array containing the cluster index for each element in
          data.
        centers: numpy array containing the integer index of each center.
        distances: numpy array of [npoints] that contains the closest distance of
          each point to any of the cluster centers.
    """
    if nclusters >= data.shape[0]:
        if return_center_indexes:
            return np.arange(data.shape[0], dtype=np.int32), np.arange(data.shape[0], dtype=np.int32)

        return np.arange(data.shape[0], dtype=np.int32)

    clusters = np.ones((data.shape[0],), dtype=np.int32) * -1
    distances = np.ones((data.shape[0],), dtype=np.float32) * 1e7
    centers = []
    for iter in range(nclusters):
        index = np.argmax(distances)
        centers.append(index)
        shape = list(data.shape)
        for i in range(1, len(shape)):
            shape[i] = 1

        broadcasted_data = np.tile(np.expand_dims(data[index], 0), shape)
        new_distances = dist_func(broadcasted_data, data)
        distances = np.minimum(distances, new_distances)
        clusters[distances == new_distances] = iter
        if verbose:
            print('farthest points max distance : {}'.format(np.max(distances)))

    if return_center_indexes:
        if return_distances:
            return clusters, np.asarray(centers, dtype=np.int32), distances
        return clusters, np.asarray(centers, dtype=np.int32)

    return clusters



def depth2pc(depth, K, rgb=None):
    """
    Currently not used

    Convert depth and intrinsics to point cloud and optionally point cloud color
    :param depth: hxw depth map in m
    :param K: 3x3 Camera Matrix with intrinsics
    :returns: (Nx3 point cloud, point cloud color)
    """

    mask = np.where(depth > 0)
    x,y = mask[1], mask[0]

    normalized_x = (x.astype(np.float32) - K[0,2])
    normalized_y = (y.astype(np.float32) - K[1,2])

    world_x = normalized_x * depth[y, x] / K[0,0]
    world_y = normalized_y * depth[y, x] / K[1,1]
    world_z = depth[y, x]

    if rgb is not None:
        rgb = rgb[y,x,:]

    pc = np.vstack((world_x, world_y, world_z)).T
    return (pc, rgb)

def reject_median_outliers(data, m=-1.4, z_only=False):
    """
    Currently not used

    Reject outliers with median absolute distance m

    Arguments:
        data {[np.ndarray]} -- Numpy array such as point cloud

    Keyword Arguments:
        m {[float]} -- Maximum absolute distance from median in m (default: {-1.4})
        z_only {[bool]} -- filter only via z_component (default: {False})

    Returns:
        [np.ndarray] -- Filtered data without outliers
    """
    if z_only:
        d = np.abs(data[:,1:3] - np.median(data[:,2:3]))
    else:
        d = np.abs(data - np.median(data, axis=-1, keepdims=True))

    return data[np.sum(d, axis=0) < m]


def distance_by_translation_point(p1, p2):
    """
      Gets two nx3 points and computes the distance between point p1 and p2.
    """
    return np.sqrt(np.sum(np.square(p1 - p2), axis=-1))


def regularize_pc_point_count(pc, npoints, use_farthest_point=False):
    """
      If point cloud pc has less points than npoints, it oversamples.
      Otherwise, it downsample the input pc to have npoint points.
      use_farthest_point: indicates

      :param pc: Nx3 point cloud
      :param npoints: number of points the regularized point cloud should have
      :param use_farthest_point: use farthest point sampling to downsample the points, runs slower.
      :returns: npointsx3 regularized point cloud
    """

    if pc.shape[0] > npoints:
        if use_farthest_point:
            _, center_indexes = farthest_points(pc, npoints, distance_by_translation_point, return_center_indexes=True)
        else:
            center_indexes = np.random.choice(range(pc.shape[0]), size=npoints, replace=False)
        pc = pc[center_indexes, :]
    else:
        required = npoints - pc.shape[0]
        if required > 0:
            index = np.random.choice(range(pc.shape[0]), size=required)
            pc = np.concatenate((pc, pc[index, :]), axis=0)
    return pc


def load_available_input_data(p, K=None):
    """
    Load available data from input file path.

    Numpy files .npz/.npy should have keys
    'depth' + 'K' + (optionally) 'segmap' + (optionally) 'rgb'
    or for point clouds:
    'xyz' + (optionally) 'xyz_color'

    png files with only depth data (in mm) can be also loaded.
    If the image path is from the GraspNet dataset, corresponding rgb, segmap and intrinic are also loaded.

    :param p: .png/.npz/.npy file path that contain depth/pointcloud and optionally intrinsics/segmentation/rgb
    :param K: 3x3 Camera Matrix with intrinsics
    :returns: All available data among segmap, rgb, depth, cam_K, pc_full, pc_colors
    """

    segmap, rgb, depth, pc_full, pc_colors = None, None, None, None, None

    if K is not None:
        if isinstance(K,str):
            cam_K = eval(K)
        cam_K = np.array(K).reshape(3,3)

    if '.np' in p:
        data = np.load(p, allow_pickle=True)
        if '.npz' in p:
            keys = data.files
        else:
            keys = []
            if len(data.shape) == 0:
                data = data.item()
                keys = data.keys()
            elif data.shape[-1] == 3:
                pc_full = data
            else:
                depth = data

        if 'depth' in keys:
            depth = data['depth']
            if K is None and 'K' in keys:
                cam_K = data['K'].reshape(3,3)
            if 'segmap' in keys:
                segmap = data['segmap']
            if 'seg' in keys:
                segmap = data['seg']
            if 'rgb' in keys:
                rgb = data['rgb']
                rgb = np.array(cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB))
        elif 'xyz' in keys:
            pc_full = np.array(data['xyz']).reshape(-1,3)
            if 'xyz_color' in keys:
                pc_colors = data['xyz_color']
    elif '.png' in p:
        if os.path.exists(p.replace('depth', 'label')):
            # graspnet data
            depth, rgb, segmap, K = load_graspnet_data(p)
        elif os.path.exists(p.replace('depths', 'images').replace('npy', 'png')):
            rgb = np.array(Image.open(p.replace('depths', 'images').replace('npy', 'png')))
        else:
            depth = np.array(Image.open(p))
    else:
        raise ValueError('{} is neither png nor npz/npy file'.format(p))

    return segmap, rgb, depth, cam_K, pc_full, pc_colors


def load_graspnet_data(rgb_image_path):

    """
    Loads data from the GraspNet-1Billion dataset
    # https://graspnet.net/

    :param rgb_image_path: .png file path to depth image in graspnet dataset
    :returns: (depth, rgb, segmap, K)
    """

    depth = np.array(Image.open(rgb_image_path))/1000. # m to mm
    segmap = np.array(Image.open(rgb_image_path.replace('depth', 'label')))
    rgb = np.array(Image.open(rgb_image_path.replace('depth', 'rgb')))

    # graspnet images are upside down, rotate for inference
    # careful: rotate grasp poses back for evaluation
    depth = np.rot90(depth,2)
    segmap = np.rot90(segmap,2)
    rgb = np.rot90(rgb,2)

    if 'kinect' in rgb_image_path:
        # Kinect azure:
        K=np.array([[631.54864502 ,  0.    ,     638.43517329],
                    [  0.    ,     631.20751953, 366.49904066],
                    [  0.    ,       0.    ,       1.        ]])
    else:
        # Realsense:
        K=np.array([[616.36529541 ,  0.    ,     310.25881958],
                    [  0.    ,     616.20294189, 236.59980774],
                    [  0.    ,       0.    ,       1.        ]])

    return depth, rgb, segmap, K



def estimate_normals_cam_from_pc(self, pc_cam, max_radius=0.05, k=12):
    """
    NOT CURRENTLY USED

    Estimates normals in camera coords from given point cloud.

    Arguments:
        pc_cam {np.ndarray} -- Nx3 point cloud in camera coordinates

    Keyword Arguments:
        max_radius {float} -- maximum radius for normal computation (default: {0.05})
        k {int} -- Number of neighbors for normal computation (default: {12})

    Returns:
        [np.ndarray] -- Nx3 point cloud normals
    """
    tree = cKDTree(pc_cam, leafsize=pc_cam.shape[0]+1)
    _, ndx = tree.query(pc_cam, k=k, distance_upper_bound=max_radius, n_jobs=-1) # num_points x k

    for c,idcs in enumerate(ndx):
        idcs[idcs==pc_cam.shape[0]] = c
        ndx[c,:] = idcs
    neighbors = np.array([pc_cam[ndx[:,n],:] for n in range(k)]).transpose((1,0,2))
    pc_normals = vectorized_normal_computation(pc_cam, neighbors)
    return pc_normals


def vectorized_normal_computation(pc, neighbors):
    """
    Vectorized normal computation with numpy

    Arguments:
        pc {np.ndarray} -- Nx3 point cloud
        neighbors {np.ndarray} -- Nxkx3 neigbours

    Returns:
        [np.ndarray] -- Nx3 normal directions
    """
    diffs = neighbors - np.expand_dims(pc, 1) # num_point x k x 3
    covs = np.matmul(np.transpose(diffs, (0, 2, 1)), diffs) # num_point x 3 x 3
    covs /= diffs.shape[1]**2
    # takes most time: 6-7ms
    eigen_values, eigen_vectors = np.linalg.eig(covs) # num_point x 3, num_point x 3 x 3
    orders = np.argsort(-eigen_values, axis=1) # num_point x 3
    orders_third = orders[:,2] # num_point
    directions = eigen_vectors[np.arange(pc.shape[0]),:,orders_third]  # num_point x 3
    dots = np.sum(directions * pc, axis=1) # num_point
    directions[dots >= 0] = -directions[dots >= 0]
    return directions