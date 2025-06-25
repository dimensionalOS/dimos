import os
import sys
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


# Import pointnet library
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT_DIR = os.path.dirname(os.path.dirname(BASE_DIR))

sys.path.append(os.path.join(BASE_DIR))
sys.path.append(os.path.join(BASE_DIR, 'Pointnet_Pointnet2_pytorch'))
# sys.path.append(os.path.join(BASE_DIR, 'pointnet2', 'utils'))

# from tf_sampling import farthest_point_sample, gather_point
# from tf_grouping import query_ball_point, group_point, knn_point

from models.pointnet2_utils import PointNetSetAbstractionMsg, PointNetSetAbstraction, PointNetFeaturePropagation
from contact_graspnet_pytorch import mesh_utils, utils

class ContactGraspnet(nn.Module):
    def __init__(self, global_config, device):
        super(ContactGraspnet, self).__init__()

        self.device = device
        # self.device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

        # -- Extract config -- #
        self.global_config = global_config
        self.model_config = global_config['MODEL']
        self.data_config = global_config['DATA']

        radius_list_0 = self.model_config['pointnet_sa_modules_msg'][0]['radius_list']
        radius_list_1 = self.model_config['pointnet_sa_modules_msg'][1]['radius_list']
        radius_list_2 = self.model_config['pointnet_sa_modules_msg'][2]['radius_list']

        nsample_list_0 = self.model_config['pointnet_sa_modules_msg'][0]['nsample_list']
        nsample_list_1 = self.model_config['pointnet_sa_modules_msg'][1]['nsample_list']
        nsample_list_2 = self.model_config['pointnet_sa_modules_msg'][2]['nsample_list']

        mlp_list_0 = self.model_config['pointnet_sa_modules_msg'][0]['mlp_list'] # list of lists
        mlp_list_1 = self.model_config['pointnet_sa_modules_msg'][1]['mlp_list']
        mlp_list_2 = self.model_config['pointnet_sa_modules_msg'][2]['mlp_list']

        npoint_0 = self.model_config['pointnet_sa_modules_msg'][0]['npoint']
        npoint_1 = self.model_config['pointnet_sa_modules_msg'][1]['npoint']
        npoint_2 = self.model_config['pointnet_sa_modules_msg'][2]['npoint']

        fp_mlp_0 = self.model_config['pointnet_fp_modules'][0]['mlp']
        fp_mlp_1 = self.model_config['pointnet_fp_modules'][1]['mlp']
        fp_mlp_2 = self.model_config['pointnet_fp_modules'][2]['mlp']

        sa_mlp = self.model_config['pointnet_sa_module']['mlp']
        sa_group_all = self.model_config['pointnet_sa_module']['group_all']

        self.input_normals = self.data_config['input_normals']
        self.offset_bins = self.data_config['labels']['offset_bins']
        self.joint_heads = self.model_config['joint_heads']

        # For adding additional features
        additional_channel = 3 if self.input_normals else 0

        if 'asymmetric_model' in self.model_config and self.model_config['asymmetric_model']:
            # It looks like there are xyz_points and "points".  The points are normals I think?
            self.sa1 = PointNetSetAbstractionMsg(npoint=npoint_0,
                                                radius_list=radius_list_0,
                                                nsample_list=nsample_list_0,
                                                in_channel = 3 + additional_channel,
                                                mlp_list=mlp_list_0)

            # Sum the size of last layer of each mlp in mlp_list
            sa1_out_channels = sum([mlp_list_0[i][-1] for i in range(len(mlp_list_0))])
            self.sa2 = PointNetSetAbstractionMsg(npoint=npoint_1,
                                                radius_list=radius_list_1,
                                                nsample_list=nsample_list_1,
                                                in_channel=sa1_out_channels,
                                                mlp_list=mlp_list_1)

            sa2_out_channels = sum([mlp_list_1[i][-1] for i in range(len(mlp_list_1))])
            self.sa3 = PointNetSetAbstractionMsg(npoint=npoint_2,
                                                radius_list=radius_list_2,
                                                nsample_list=nsample_list_2,
                                                in_channel=sa2_out_channels,
                                                mlp_list=mlp_list_2)

            sa3_out_channels = sum([mlp_list_2[i][-1] for i in range(len(mlp_list_2))])
            self.sa4 = PointNetSetAbstraction(npoint=None,
                                                radius=None,
                                                nsample=None,
                                                in_channel=3 + sa3_out_channels,
                                                mlp=sa_mlp,
                                                group_all=sa_group_all)

            self.fp3 = PointNetFeaturePropagation(in_channel=sa_mlp[-1] + sa3_out_channels,
                                                mlp=fp_mlp_2)

            self.fp2 = PointNetFeaturePropagation(in_channel=fp_mlp_2[-1] + sa2_out_channels,
                                                mlp=fp_mlp_1)

            self.fp1 = PointNetFeaturePropagation(in_channel=fp_mlp_1[-1] + sa1_out_channels,
                                                mlp=fp_mlp_0)
        else:
            raise NotImplementedError

        if self.joint_heads:
            raise NotImplementedError
        else:
            # Theres prob some bugs here

            # Head for grasp direction
            self.grasp_dir_head = nn.Sequential(
                nn.Conv1d(fp_mlp_0[-1], 128, 1, padding=0),
                nn.BatchNorm1d(128),
                nn.ReLU(),
                nn.Dropout(0.3), # p = 1 - keep_prob  (tf is inverse of torch)
                nn.Conv1d(128, 3, 1, padding=0)
            )

            # Remember to normalize the output of this head

            # Head for grasp approach
            self.grasp_approach_head = nn.Sequential(
                nn.Conv1d(fp_mlp_0[-1], 128, 1, padding=0),
                nn.BatchNorm1d(128),
                nn.ReLU(),
                nn.Dropout(0.3),
                nn.Conv1d(128, 3, 1, padding=0)
            )

            # Head for grasp width
            if self.model_config['dir_vec_length_offset']:
                raise NotImplementedError
            elif self.model_config['bin_offsets']:
                self.grasp_offset_head = nn.Sequential(
                    nn.Conv1d(fp_mlp_0[-1], 128, 1, padding=0),
                    nn.BatchNorm1d(128),
                    nn.ReLU(),
                    nn.Conv1d(128, len(self.offset_bins) - 1, 1, padding=0)
                )
            else:
                raise NotImplementedError

            # Head for contact points
            self.binary_seg_head = nn.Sequential(
                nn.Conv1d(fp_mlp_0[-1], 128, 1, padding=0),
                nn.BatchNorm1d(128),
                nn.ReLU(),
                nn.Dropout(0.5),  # 0.5 in original code
                nn.Conv1d(128, 1, 1, padding=0)
            )

    def forward(self, point_cloud):

        # expensive, rather use random only
        if 'raw_num_points' in self.data_config and self.data_config['raw_num_points'] != self.data_config['ndataset_points']:
            raise NotImplementedError
            point_cloud = gather_point(point_cloud, farthest_point_sample(data_config['ndataset_points'], point_cloud))

        # Convert from tf to torch ordering
        point_cloud = torch.transpose(point_cloud, 1, 2) # Now we have batch x channels (3 or 6) x num_points


        l0_xyz = point_cloud[:, :3, :]
        l0_points = point_cloud[:, 3:6, :] if self.input_normals else l0_xyz.clone()

        # -- PointNet Backbone -- #
        # Set Abstraction Layers
        l1_xyz, l1_points = self.sa1(l0_xyz, l0_points)
        l2_xyz, l2_points = self.sa2(l1_xyz, l1_points)
        l3_xyz, l3_points = self.sa3(l2_xyz, l2_points)
        l4_xyz, l4_points = self.sa4(l3_xyz, l3_points)

        # Feature Propagation Layers
        l3_points = self.fp3(l3_xyz, l4_xyz, l3_points, l4_points)
        l2_points = self.fp2(l2_xyz, l3_xyz, l2_points, l3_points)
        l1_points = self.fp1(l1_xyz, l2_xyz, l1_points, l2_points)

        l0_points = l1_points
        pred_points = l1_xyz

        # -- Heads -- #
        # Grasp Direction Head
        grasp_dir_head = self.grasp_dir_head(l0_points)
        grasp_dir_head_normed = F.normalize(grasp_dir_head, p=2, dim=1)  # normalize along channels

        # Grasp Approach Head
        approach_dir_head = self.grasp_approach_head(l0_points)

        # computer gram schmidt orthonormalization
        dot_product = torch.sum(grasp_dir_head_normed * approach_dir_head, dim=1, keepdim=True)
        projection = dot_product * grasp_dir_head_normed
        approach_dir_head_orthog = F.normalize(approach_dir_head - projection, p=2, dim=1)

        # Grasp Width Head
        if self.model_config['dir_vec_length_offset']:
            raise NotImplementedError
            grasp_offset_head = F.normalize(grasp_dir_head, p=2, dim=1)
        elif self.model_config['bin_offsets']:
            grasp_offset_head = self.grasp_offset_head(l0_points)

        # Binary Segmentation Head
        binary_seg_head = self.binary_seg_head(l0_points)

        # -- Create end_points -- #
        # I don't think this is needed anymore
        # end_points = {}
        # end_points['grasp_dir_head'] = grasp_dir_head
        # end_points['binary_seg_head'] = binary_seg_head
        # end_points['binary_seg_pred'] = torch.sigmoid(binary_seg_head)
        # end_points['grasp_offset_head'] = grasp_offset_head
        # if self.model_config['bin_offsets']:
        #     end_points['grasp_offset_pred'] = torch.sigmoid(grasp_offset_head)
        # else:
        #     end_points['grasp_offset_pred'] = grasp_offset_head
        # end_points['approach_dir_head'] = approach_dir_head_orthog
        # end_points['pred_points'] = pred_points

        # # Get back to tf ordering
        # for k, points in end_points.items():
        #     end_points[k] = points.permute(0, 2, 1)

        # Expected outputs:
                # pred_grasps_cam, pred_scores, pred_points, offset_pred = self.model(pc_batch)

        # -- Construct Output -- #
        # Get 6 DoF grasp pose
        torch_bin_vals = self.get_bin_vals()

        # PyTorch equivalent of tf.gather_nd with conditional
        # I think the output should be B x N
        if self.model_config['bin_offsets']:
            argmax_indices = torch.argmax(grasp_offset_head, dim=1, keepdim=True)
            offset_bin_pred_vals = torch_bin_vals[argmax_indices]  # kinda sketch but works?
            # expand_dims_indices = argmax_indices.unsqueeze(1)
            # offset_bin_pred_vals = torch.gather(torch_bin_vals, 1, argmax_indices)
        else:
            offset_bin_pred_vals = grasp_offset_head[:, 0, :]

        # TODO Start here
        pred_grasps_cam = self.build_6d_grasp(approach_dir_head_orthog.permute(0, 2, 1),
                                              grasp_dir_head.permute(0, 2, 1),
                                              pred_points.permute(0, 2, 1),
                                              offset_bin_pred_vals.permute(0, 2, 1),
                                              use_torch=True)  # B x N x 4 x 4

        # Get pred scores
        pred_scores = torch.sigmoid(binary_seg_head).permute(0, 2, 1)

        # Get pred points
        pred_points = pred_points.permute(0, 2, 1)

        # Get pred offsets
        offset_pred = offset_bin_pred_vals

        # -- Values to compute loss on -- #
        # intermediates = {}
        # intermediates['grasp_offset_head'] = grasp_offset_head

        pred = dict(
            pred_grasps_cam=pred_grasps_cam,
            pred_scores=pred_scores,
            pred_points=pred_points,
            offset_pred=offset_pred,
            grasp_offset_head=grasp_offset_head # For loss
        )

        # return pred_grasps_cam, pred_scores, pred_points, offset_pred, intermediates
        return pred


    def get_bin_vals(self):
        """
        Creates bin values for grasping widths according to bounds defined in config

        Arguments:
            global_config {dict} -- config

        Returns:
            torch.tensor -- bin value tensor
        """
        bins_bounds = np.array(self.data_config['labels']['offset_bins'])
        if self.global_config['TEST']['bin_vals'] == 'max':
            bin_vals = (bins_bounds[1:] + bins_bounds[:-1])/2
            bin_vals[-1] = bins_bounds[-1]
        elif self.global_config['TEST']['bin_vals'] == 'mean':
            bin_vals = bins_bounds[1:]
        else:
            raise NotImplementedError

        if not self.global_config['TEST']['allow_zero_margin']:
            bin_vals = np.minimum(bin_vals, self.global_config['DATA']['gripper_width'] \
                                  -self.global_config['TEST']['extra_opening'])

        bin_vals = torch.tensor(bin_vals, dtype=torch.float32).to(self.device)
        return bin_vals



    # def build_6d_grasp(self, approach_dirs, base_dirs, contact_pts, thickness, use_tf=False, gripper_depth = 0.1034):
    def build_6d_grasp(self, approach_dirs, base_dirs, contact_pts, thickness, use_torch=False, gripper_depth = 0.1034):
        """
        Build 6-DoF grasps + width from point-wise network predictions

        Arguments:
            approach_dirs {np.ndarray/tf.tensor} -- Nx3 approach direction vectors
            base_dirs {np.ndarray/tf.tensor} -- Nx3 base direction vectors
            contact_pts {np.ndarray/tf.tensor} -- Nx3 contact points
            thickness {np.ndarray/tf.tensor} -- Nx1 grasp width

        Keyword Arguments:
            use_tf {bool} -- whether inputs and outputs are tf tensors (default: {False})
            gripper_depth {float} -- distance from gripper coordinate frame to gripper baseline in m (default: {0.1034})

        Returns:
            np.ndarray -- Nx4x4 grasp poses in camera coordinates
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
        if use_torch:
            grasp_R = torch.stack([base_dirs, torch.cross(approach_dirs,base_dirs),approach_dirs], dim=3)  # B x N x 3 x 3
            grasp_t = contact_pts + (thickness / 2) * base_dirs - gripper_depth * approach_dirs  # B x N x 3
            grasp_t = grasp_t.unsqueeze(3)  # B x N x 3 x 1
            ones = torch.ones((contact_pts.shape[0], contact_pts.shape[1], 1, 1), dtype=torch.float32).to(self.device)  # B x N x 1 x 1
            zeros = torch.zeros((contact_pts.shape[0], contact_pts.shape[1], 1, 3), dtype=torch.float32).to(self.device)  # B x N x 1 x 3
            homog_vec = torch.cat([zeros, ones], dim=3)  # B x N x 1 x 4
            grasps = torch.cat([torch.cat([grasp_R, grasp_t], dim=3), homog_vec], dim=2)  # B x N x 4 x 4

        else:
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

class ContactGraspnetLoss(nn.Module):
    def __init__(self, global_config, device):
        super(ContactGraspnetLoss, self).__init__()
        self.global_config = global_config

        # -- Process config -- #
        config_losses = [
            'pred_contact_base',
            'pred_contact_success', # True
            'pred_contact_offset',  # True
            'pred_contact_approach',
            'pred_grasps_adds',  # True
            'pred_grasps_adds_gt2pred'
        ]

        config_weights = [
            'dir_cosine_loss_weight',
            'score_ce_loss_weight',  # True
            'offset_loss_weight',  # True
            'approach_cosine_loss_weight',
            'adds_loss_weight',  # True
            'adds_gt2pred_loss_weight'
        ]

        self.device = device

        bin_weights = global_config['DATA']['labels']['bin_weights']
        self.bin_weights = torch.tensor(bin_weights).to(self.device)
        self.bin_vals = self._get_bin_vals().to(self.device)

        for config_loss, config_weight in zip(config_losses, config_weights):
            if global_config['MODEL'][config_loss]:
                setattr(self, config_weight, global_config['OPTIMIZER'][config_weight])
            else:
                setattr(self, config_weight, 0.0)

        self.gripper = mesh_utils.create_gripper('panda')
        # gripper_control_points = self.gripper.get_control_point_tensor(self.global_config['OPTIMIZER']['batch_size']) # b x 5 x 3
        # sym_gripper_control_points = self.gripper.get_control_point_tensor(self.global_config['OPTIMIZER']['batch_size'], symmetric=True)

        # self.gripper_control_points_homog = torch.cat([gripper_control_points,
        #     torch.ones((self.global_config['OPTIMIZER']['batch_size'], gripper_control_points.shape[1], 1))], dim=2)  # b x 5 x 4
        # self.sym_gripper_control_points_homog = torch.cat([sym_gripper_control_points,
        #     torch.ones((self.global_config['OPTIMIZER']['batch_size'], gripper_control_points.shape[1], 1))], dim=2)  # b x 5 x 4

        n_copies = 1  # We will repeat this according to the batch size
        gripper_control_points = self.gripper.get_control_point_tensor(n_copies) # b x 5 x 3
        sym_gripper_control_points = self.gripper.get_control_point_tensor(n_copies, symmetric=True)

        self.gripper_control_points_homog = torch.cat([gripper_control_points,
            torch.ones((n_copies, gripper_control_points.shape[1], 1))], dim=2)  # b x 5 x 4
        self.sym_gripper_control_points_homog = torch.cat([sym_gripper_control_points,
            torch.ones((n_copies, gripper_control_points.shape[1], 1))], dim=2)  # b x 5 x 4

        self.gripper_control_points_homog = self.gripper_control_points_homog.to(self.device)
        self.sym_gripper_control_points_homog = self.sym_gripper_control_points_homog.to(self.device)


    def forward(self, pred, target):
        """
        Computes loss terms from pointclouds, network predictions and labels

        Arguments:
            pointclouds_pl {tf.placeholder} -- bxNx3 input point clouds
            end_points {dict[str:tf.variable]} -- endpoints of the network containing predictions
            dir_labels_pc_cam {tf.variable} -- base direction labels in camera coordinates (bxNx3)
            offset_labels_pc {tf.variable} -- grasp width labels (bxNx1)
            grasp_success_labels_pc {tf.variable} -- contact success labels (bxNx1)
            approach_labels_pc_cam {tf.variable} -- approach direction labels in camera coordinates (bxNx3)
            global_config {dict} -- config dict

        Returns:
            [dir_cosine_loss, bin_ce_loss, offset_loss, approach_cosine_loss, adds_loss,
            adds_loss_gt2pred, gt_control_points, pred_control_points, pos_grasps_in_view] -- All losses (not all are used for training)
        """
        pred_grasps_cam = pred['pred_grasps_cam']           # B x N x 4 x 4
        pred_scores = pred['pred_scores']                   # B x N x 1
        pred_points = pred['pred_points']                   # B x N x 3
        # offset_pred = pred['offset_pred']                   # B x N  # We use the grasp_offset_head instead of this
        grasp_offset_head = pred['grasp_offset_head'].permute(0, 2, 1)       # B x N x 10


        # # Generated in acronym_dataloader.py
        # grasp_success_labels_pc = target['grasp_success_label']  # B x N
        # grasp_offset_labels_pc = target['grasp_diff_label']    # B x N x 3

        # approach_labels_pc_cam = target['grasp_approach_label']    # B x N x 3
        # dir_labels_pc_cam = target['grasp_dir_label']              # B x N x 3
        # pointclouds_pl = target['pc_cam']                    # B x N x 3

        # -- Interpolate Labels -- #
        pos_contact_points = target['pos_contact_points']    # B x M x 3
        pos_contact_dirs = target['pos_contact_dirs']        # B x M x 3
        pos_finger_diffs = target['pos_finger_diffs']        # B x M
        pos_approach_dirs = target['pos_approach_dirs']      # B x M x 3
        camera_pose = target['camera_pose']                  # B x 4 x 4

        dir_labels_pc_cam, \
        grasp_offset_labels_pc, \
        grasp_success_labels_pc, \
        approach_labels_pc_cam, \
        debug = self._compute_labels(pred_points, 
                                     camera_pose,
                                     pos_contact_points,
                                     pos_contact_dirs,
                                     pos_finger_diffs,
                                     pos_approach_dirs)
            
        # I think this is the number of positive grasps that are in view
        min_geom_loss_divisor = float(self.global_config['LOSS']['min_geom_loss_divisor'])  # This is 1.0
        pos_grasps_in_view = torch.clamp(grasp_success_labels_pc.sum(dim=1), min=min_geom_loss_divisor)  # B
        # pos_grasps_in_view = torch.maximum(grasp_success_labels_pc.sum(dim=1), min_geom_loss_divisor)  # B

        total_loss = 0.0

        if self.dir_cosine_loss_weight > 0:
            raise NotImplementedError

        # -- Grasp Confidence Loss -- #
        if self.score_ce_loss_weight > 0:  # TODO (bin_ce_loss)
            bin_ce_loss = F.binary_cross_entropy(pred_scores, grasp_success_labels_pc, reduction='none')  # B x N x 1
            if 'topk_confidence' in self.global_config['LOSS'] \
                and self.global_config['LOSS']['topk_confidence']:
                bin_ce_loss, _ = torch.topk(bin_ce_loss.squeeze(), k=self.global_config['LOSS']['topk_confidence'])
            bin_ce_loss = torch.mean(bin_ce_loss)

            total_loss += self.score_ce_loss_weight * bin_ce_loss

        # -- Grasp Offset / Thickness Loss -- #
        if self.offset_loss_weight > 0:  # TODO  (offset_loss)
            if self.global_config['MODEL']['bin_offsets']:
                # Convert labels to multihot
                bin_vals = self.global_config['DATA']['labels']['offset_bins']
                grasp_offset_labels_multihot = self._bin_label_to_multihot(grasp_offset_labels_pc, 
                                                                           bin_vals)

                if self.global_config['LOSS']['offset_loss_type'] == 'softmax_cross_entropy':
                    raise NotImplementedError

                else:
                    offset_loss = F.binary_cross_entropy_with_logits(grasp_offset_head,
                                                                     grasp_offset_labels_multihot, reduction='none')  # B x N x 1
                    if 'too_small_offset_pred_bin_factor' in self.global_config['LOSS'] \
                        and self.global_config['LOSS']['too_small_offset_pred_bin_factor']:
                        raise NotImplementedError

                    # Weight loss for each bin
                    shaped_bin_weights = self.bin_weights[None, None, :]
                    offset_loss = (shaped_bin_weights * offset_loss).mean(axis=2)
            else:
                raise NotImplementedError
            masked_offset_loss = offset_loss * grasp_success_labels_pc.squeeze()
            # Divide each batch by the number of successful grasps in the batch
            offset_loss = torch.mean(torch.sum(masked_offset_loss, axis=1, keepdim=True) / pos_grasps_in_view)

            total_loss += self.offset_loss_weight * offset_loss

        if self.approach_cosine_loss_weight > 0:
            raise NotImplementedError

        # -- 6 Dof Pose Loss -- #
        if self.adds_loss_weight > 0:  # TODO  (adds_loss)
            # Build groudn truth grasps and compare distances to predicted grasps

            ### ADS Gripper PC Loss
            # Get 6 DoF pose of predicted grasp
            if self.global_config['MODEL']['bin_offsets']:
                thickness_gt = self.bin_vals[torch.argmax(grasp_offset_labels_pc, dim=2)]
            else:
                thickness_gt = grasp_offset_labels_pc[:, :, 0]

            # TODO: Move this to dataloader? 
            pred_grasps = pred_grasps_cam  # B x N x 4 x 4
            gt_grasps_proj = utils.build_6d_grasp(approach_labels_pc_cam, 
                                                            dir_labels_pc_cam, 
                                                            pred_points, 
                                                            thickness_gt, 
                                                            use_torch=True,
                                                            device=self.device) # b x N x 4 x 4
            # Select positive grasps I think?
            success_mask = grasp_success_labels_pc.bool()[:, :, :, None] # B x N x 1 x 1
            success_mask = torch.broadcast_to(success_mask, gt_grasps_proj.shape) # B x N x 4 x 4
            pos_gt_grasps_proj = torch.where(success_mask, gt_grasps_proj, torch.ones_like(gt_grasps_proj) * 100000) # B x N x 4 x 4

            # Expand gripper control points to match number of points
            # only use per point pred grasps but not per point gt grasps
            control_points = self.gripper_control_points_homog.unsqueeze(1)  # 1 x 1 x 5 x 4
            control_points = control_points.repeat(pred_points.shape[0], pred_points.shape[1], 1, 1)  # b x N x 5 x 4

            sym_control_points = self.sym_gripper_control_points_homog.unsqueeze(1)  # 1 x 1 x 5 x 4
            sym_control_points = sym_control_points.repeat(pred_points.shape[0], pred_points.shape[1], 1, 1)  # b x N x 5 x 4

            pred_control_points = torch.matmul(control_points, pred_grasps.permute(0, 1, 3, 2))[:, :, :, :3]  # b x N x 5 x 3

            # Transform control points to ground truth locations
            gt_control_points = torch.matmul(control_points, pos_gt_grasps_proj.permute(0, 1, 3, 2))[:, :, :, :3]  # b x N x 5 x 3
            sym_gt_control_points = torch.matmul(sym_control_points, pos_gt_grasps_proj.permute(0, 1, 3, 2))[:, :, :, :3]  # b x N x 5 x 3

            # Compute distances between predicted and ground truth control points
            expanded_pred_control_points = pred_control_points.unsqueeze(2)         # B x N x 1 x 5 x 3
            expanded_gt_control_points = gt_control_points.unsqueeze(1)             # B x 1 x N' x 5 x 3  I think N' == N
            expanded_sym_gt_control_points = sym_gt_control_points.unsqueeze(1)     # B x 1 x N' x 5 x 3  I think N' == N

            # Sum of squared distances between all points
            squared_add = torch.sum((expanded_pred_control_points - expanded_gt_control_points)**2, dim=(3, 4))  # B x N x N'
            sym_squared_add = torch.sum((expanded_pred_control_points - expanded_sym_gt_control_points)**2, dim=(3, 4))  # B x N x N'

            # Combine distances between gt and symmetric gt grasps
            squared_adds = torch.concat([squared_add, sym_squared_add], dim=2)  # B x N x 2N'

            # Take min distance to gt grasp for each predicted grasp
            squared_adds_k = torch.topk(squared_adds, k=1, dim=2, largest=False)[0]  # B x N

            # Mask negative grasps
            # TODO: If there are bugs, its prob here.  The original code sums on axis=1
            # Which just determines if there is a successful grasp in the batch.  
            # I think we just want to select the positive grasps so the sum is redundant.
            sum_grasp_success_labels = torch.sum(grasp_success_labels_pc, dim=2, keepdim=True)
            binary_grasp_success_labels = torch.clamp(sum_grasp_success_labels, 0, 1)
            min_adds = binary_grasp_success_labels * torch.sqrt(squared_adds_k)  # B x N x 1
            adds_loss = torch.sum(pred_scores * min_adds, dim=(1), keepdim=True)  # B x 1
            adds_loss = adds_loss.squeeze() / pos_grasps_in_view.squeeze()  # B x 1
            adds_loss = torch.mean(adds_loss)
            total_loss += self.adds_loss_weight * adds_loss

        if self.adds_gt2pred_loss_weight > 0:
            raise NotImplementedError
        
        loss_info = {
            'bin_ce_loss': bin_ce_loss,  # Grasp success loss
            'offset_loss': offset_loss,  # Grasp width loss
            'adds_loss': adds_loss,  # Pose loss
        }

        return total_loss, loss_info

    def _get_bin_vals(self):
        """
        Creates bin values for grasping widths according to bounds defined in config

        Arguments:
            global_config {dict} -- config

        Returns:
            tf.constant -- bin value tensor
        """
        bins_bounds = np.array(self.global_config['DATA']['labels']['offset_bins'])
        if self.global_config['TEST']['bin_vals'] == 'max':
            bin_vals = (bins_bounds[1:] + bins_bounds[:-1])/2
            bin_vals[-1] = bins_bounds[-1]
        elif self.global_config['TEST']['bin_vals'] == 'mean':
            bin_vals = bins_bounds[1:]
        else:
            raise NotImplementedError

        if not self.global_config['TEST']['allow_zero_margin']:
            bin_vals = np.minimum(bin_vals, self.global_config['DATA']['gripper_width']-self.global_config['TEST']['extra_opening'])

        bin_vals = torch.tensor(bin_vals, dtype=torch.float32)
        return bin_vals
    
    def _bin_label_to_multihot(self, cont_labels, bin_boundaries):
        """
        Computes binned grasp width labels from continuous labels and bin boundaries

        Arguments:
            cont_labels {torch.Tensor} -- continuous labels
            bin_boundaries {list} -- bin boundary values

        Returns:
            torch.Tensor -- one/multi hot bin labels
        """
        bins = []
        for b in range(len(bin_boundaries)-1):
            bins.append(torch.logical_and(torch.greater_equal(cont_labels, bin_boundaries[b]), torch.less(cont_labels, bin_boundaries[b+1])))
        multi_hot_labels = torch.cat(bins, dim=2)
        multi_hot_labels = multi_hot_labels.to(torch.float32)

        return multi_hot_labels

    
    def _compute_labels(self, 
                        processed_pc_cams: torch.Tensor, 
                        camera_poses: torch.Tensor, 
                        pos_contact_points: torch.Tensor,
                        pos_contact_dirs: torch.Tensor,
                        pos_finger_diffs: torch.Tensor, 
                        pos_approach_dirs: torch.Tensor):
        """
        Project grasp labels defined on meshes onto rendered point cloud 
        from a camera pose via nearest neighbor contacts within a maximum radius. 
        All points without nearby successful grasp contacts are considered 
        negative contact points.

        Here N is the number of points returned by the PointNet Encoder (2048) while
        M is the number of points in the ground truth data.  B is the batch size.
        We are trying to assign a label to each of the PointNet points by 
        sampling the nearest ground truth points.

        Arguments:
            pc_cam_pl (torch.Tensor): (B, N, 3) point cloud in camera frame
            camera_pose_pl (torch.Tensor): (B, 4, 4) homogenous camera pose
            pos_contact_points (torch.Tensor): (B, M, 3) contact points in world frame (3 DoF points)
            pos_contact_dirs (torch.Tensor): (B, M, 3) contact directions (origin centered vectors?)
            pos_finger_diffs (torch.Tensor): (B, M, ) finger diffs in world frame  (scalar distances)
            pos_approach_dirs (torch.Tensor): (B, M, 3) approach directions in world frame (origin centered vectors?)
        """
        label_config = self.global_config['DATA']['labels']

        nsample = label_config['k']  # Currently set to 1
        radius = label_config['max_radius']
        filter_z = label_config['filter_z']
        z_val = label_config['z_val']

        _, N, _ = processed_pc_cams.shape
        B, M, _ = pos_contact_points.shape

        # -- Make sure pcd is B x N x 3 -- #
        if processed_pc_cams.shape[2] != 3:
            xyz_cam = processed_pc_cams[:,:,:3]  # N x 3
        else:
            xyz_cam = processed_pc_cams

        # -- Transform Ground Truth to Camera Frame -- #
        # Transform contact points to camera frame  (This is a homogenous transform)
        # We use matmul to accommodate batch
        # pos_contact_points_cam = pos_contact_points @ (camera_poses[:3,:3].T) + camera_poses[:3,3][None,:]
        pos_contact_points_cam = torch.matmul(pos_contact_points, camera_poses[:, :3, :3].transpose(1, 2)) \
            + camera_poses[:,:3,3][:, None,:]

        # Transform contact directions to camera frame (Don't translate because its a direction vector)
        # pos_contact_dirs_cam = pos_contact_dirs @ camera_poses[:3,:3].T
        pos_contact_dirs_cam = torch.matmul(pos_contact_dirs, camera_poses[:, :3,:3].transpose(1, 2))
        
        # Make finger diffs B x M x 1
        pos_finger_diffs = pos_finger_diffs[:, :, None]

        # Transform approach directions to camera frame (Don't translate because its a direction vector)
        # pos_approach_dirs_cam = pos_approach_dirs @ camera_poses[:3,:3].T
        pos_approach_dirs_cam = torch.matmul(pos_approach_dirs, camera_poses[:, :3,:3].transpose(1, 2))

        # -- Filter Direction -- #
        # TODO: Figure out what is going on here
        if filter_z:
            # Filter out directions that are too far
            dir_filter_passed = (pos_contact_dirs_cam[:, :, 2:3] > z_val).repeat(1, 1, 3)
            pos_contact_points_cam = torch.where(dir_filter_passed, 
                                                 pos_contact_points_cam, 
                                                 torch.ones_like(pos_contact_points_cam) * 10000)
        
        # -- Compute Distances -- #
        # We want to compute the distance between each point in the point cloud and each contact point
        # We can do this by expanding the dimensions of the tensors and then summing the squared differences
        xyz_cam_expanded = torch.unsqueeze(xyz_cam, 2)  # B x N x 1 x 3
        pos_contact_points_cam_expanded = torch.unsqueeze(pos_contact_points_cam, 1)  # B x 1 x M x 3
        squared_dists_all = torch.sum((xyz_cam_expanded - pos_contact_points_cam_expanded)**2, dim=3)  # B x N x M

        # B x N x k, B x N x k
        squared_dists_k, close_contact_pt_idcs = torch.topk(squared_dists_all, 
            k=nsample, dim=2, largest=False, sorted=False)

        # -- Group labels -- #
        grouped_contact_dirs_cam = utils.index_points(pos_contact_dirs_cam, close_contact_pt_idcs)  # B x N x k x 3
        grouped_finger_diffs = utils.index_points(pos_finger_diffs, close_contact_pt_idcs)  # B x N x k x 1
        grouped_approach_dirs_cam = utils.index_points(pos_approach_dirs_cam, close_contact_pt_idcs)  # B x N x k x 3

        # grouped_contact_dirs_cam = pos_contact_dirs_cam[close_contact_pt_idcs, :]  # B x N x k x 3
        # grouped_finger_diffs = pos_finger_diffs[close_contact_pt_idcs]  # B x N x k x 1
        # grouped_approach_dirs_cam = pos_approach_dirs_cam[close_contact_pt_idcs, :]  # B x N x k x 3

        # -- Compute Labels -- #
        # Take mean over k nearest neighbors and normalize
        dir_label = grouped_contact_dirs_cam.mean(dim=2)  # B x N x 3
        dir_label = F.normalize(dir_label, p=2, dim=2)  # B x N x 3

        diff_label = grouped_finger_diffs.mean(dim=2)# B x N x 1

        approach_label = grouped_approach_dirs_cam.mean(dim=2)  # B x N x 3
        approach_label = F.normalize(approach_label, p=2, dim=2)  # B x N x 3

        grasp_success_label = torch.mean(squared_dists_k, dim=2, keepdim=True) < radius**2  # B x N x 1 
        grasp_success_label = grasp_success_label.type(torch.float32)  

        # debug = dict(
        #     xyz_cam = xyz_cam,
        #     pos_contact_points_cam = pos_contact_points_cam,
        # )
        debug = {}


        return dir_label, diff_label, grasp_success_label, approach_label, debug






