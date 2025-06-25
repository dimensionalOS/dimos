import numpy as np
import matplotlib.pyplot as plt
from dict_diff import findDiff
import sys
import glob2
import os
import yaml
sys.path.append('/home/msundermeyer/ngc_ws/6dof-graspnet/contact_graspnet')
import utilities
import copy
from color import color
from scipy import spatial


def metric_coverage_success_rate(grasps_list, scores_list, flex_outcomes_list, gt_grasps_list, visualize, num_scenes=100):
    """
    Computes the coverage success rate for grasps of multiple objects.

    Args:
        grasps_list: list of numpy array, each numpy array is the predicted
        grasps for each object. Each numpy array has shape (n, 4, 4) where
        n is the number of predicted grasps for each object.
        scores_list: list of numpy array, each numpy array is the predicted
        scores for each grasp of the corresponding object.
        flex_outcomes_list: list of numpy array, each element of the numpy
        array indicates whether that grasp succeeds in grasping the object
        or not.
        gt_grasps_list: list of numpy array. Each numpy array has shape of
        (m, 4, 4) where m is the number of groundtruth grasps for each
        object.
        visualize: bool. If True, it will plot the curve.
    
    Returns:
        auc: float, area under the curve for the success-coverage plot.
    """
    RADIUS = 0.02

    all_trees = []
    all_grasps = []
    all_object_indexes = []
    all_scores = []
    all_flex_outcomes = []
    visited = set()
    tot_num_gt_grasps = 0
    for i in range(num_scenes):
        print('building kd-tree {}/{}'.format(i+1, len(grasps_list[:num_scenes])))
        gt_grasps = np.asarray(gt_grasps_list[i]).copy()
        all_trees.append(spatial.KDTree(gt_grasps[:, :3, 3]))
        tot_num_gt_grasps += gt_grasps.shape[0]
        print(np.mean(flex_outcomes_list[i]))
        try:
            print(len(grasps_list[i]), len(scores_list[i].reshape(-1)), len(flex_outcomes_list[i]))
        except:
            import pdb; pdb.set_trace()
        for g, s, f in zip(grasps_list[i], scores_list[i].reshape(-1), flex_outcomes_list[i]):
            all_grasps.append(np.asarray(g).copy())
            all_object_indexes.append(i)
            all_scores.append(s)
            all_flex_outcomes.append(f)
    
    all_grasps = np.asarray(all_grasps)
    
    all_scores = np.asarray(all_scores)
    order = np.argsort(-all_scores)
    num_covered_so_far = 0
    correct_grasps_so_far = 0
    num_visited_grasps_so_far = 0

    precisions = []
    recalls = []
    prev_score = None

    for oindex, index in enumerate(order):
        if oindex % 1000 == 0:
            print(oindex, len(order))
        
        object_id = all_object_indexes[index]
        close_indexes = all_trees[object_id].query_ball_point(all_grasps[index, :3, 3], RADIUS)

        num_new_covered_gt_grasps = 0

        for close_index in close_indexes:
            key = (object_id, close_index)
            if key in visited:
                continue
            
            visited.add(key)
            num_new_covered_gt_grasps += 1

        correct_grasps_so_far += all_flex_outcomes[index]
        num_visited_grasps_so_far += 1
        num_covered_so_far += num_new_covered_gt_grasps
        if prev_score is not None and abs(prev_score - all_scores[index]) < 1e-3:
            precisions[-1] = float(correct_grasps_so_far) / num_visited_grasps_so_far
            recalls[-1] = float(num_covered_so_far) / tot_num_gt_grasps
        else:
            precisions.append(float(correct_grasps_so_far) / num_visited_grasps_so_far)
            recalls.append(float(num_covered_so_far) / tot_num_gt_grasps)
            prev_score = all_scores[index]
    
    auc = 0
    for i in range(1, len(precisions)):
        auc += (recalls[i] - recalls[i-1]) * (precisions[i] + precisions[i-1]) * 0.5
    
    if visualize:
        import matplotlib.pyplot as plt
        plt.plot(recalls, precisions)
        plt.title('Simulator Precision-Coverage Curves auc = {0:02f}'.format(auc))
        plt.xlabel('Coverage')
        plt.ylabel('Precision')
        plt.xlim([0, 0.6]) 
        plt.ylim([0,1.0])
        # plt.show()
        
    
    return auc, {'precisions': precisions, 'recalls': recalls, 'auc': auc, 'cfg': None}

# pr_data = glob2.glob(os.path.join(sys.argv[1],'*','*','outfile.npy')) + glob2.glob(os.path.join(sys.argv[1],'*','outfile.npy')) + glob2.glob(os.path.join(sys.argv[1],'outfile.npy'))
pr_data = glob2.glob(os.path.join(sys.argv[1],'*','*','all_full_results.npz')) + glob2.glob(os.path.join(sys.argv[1],'*','all_full_results.npz')) + glob2.glob(os.path.join(sys.argv[1],'all_full_results.npz'))

default_compare = True
if default_compare:
    default_config = utilities.load_config('/home/msundermeyer/ngc_ws/6dof-graspnet/contact_graspnet')
else:
    default_config = np.load(pr_data[0], allow_pickle=True).item()['cfg']

legends = []
all_diff_dicts = {}
cfgs = {}
aucs_01 = {}
name_dict = {}

gt_grasps = []
for p in range(100):
    y=np.load('/home/msundermeyer/datasets/visibility_filtered_gt_grasp/{}_filtered_gt_grasps.npz'.format(p), allow_pickle=True)
    gt_grasps.append(y['gt_grasp_scene_trafos'])



for abc in pr_data:
    try:
        x=np.load(os.path.join(os.path.dirname(abc), 'flex_temp', 'all_full_results.npz'), allow_pickle=True)
    except:
        x=np.load(os.path.join(os.path.dirname(os.path.dirname(abc)), 'flex_temp', 'all_full_results.npz'), allow_pickle=True)

    auc, _ = metric_coverage_success_rate(x['grasps'], x['scores'],  x['flex_outcomes'], gt_grasps, True)

    base_dir = os.path.dirname(os.path.dirname(abc))
    outfile_data = glob2.glob(os.path.join(base_dir,'*','*','outfile.npy')) + glob2.glob(os.path.join(base_dir,'*','outfile.npy')) + glob2.glob(os.path.join(base_dir,'outfile.npy'))
    
    if outfile_data:
        d = outfile_data[0]
        print(d)
        a=np.load(d, allow_pickle=True).item()
        
        if d.split('/')[2] == 'outfile.npy':
            names = os.listdir(os.path.dirname(d))
            print(names) 
            name = names[0]
        else:
            try:
                name = d.split('/')[5]
            except:    
                name = d.split('/')[4]
            
        print(50*'#')
        print(name)
        all_diff_dicts[name] = copy.deepcopy(findDiff(default_config, a['cfg'], diff_dict={}))
        cfgs[name] = a['cfg']
        print(all_diff_dicts)

        b=[]
        for g in x['scores']:
            b += list(g)
        print(np.histogram(b))
        i = np.argsort(b)
        o = []
        for g in x['flex_outcomes']:
            o += list(g)
        # np.histogram(np.array(o)[i[-1000:]])
        # np.histogram(np.array(o)[i[-4000:-2000]])
        # np.histogram(np.array(o)[i[-4000:-3000]])
        print(np.histogram(o))
        print(a['auc'])
        aucs_01[name] = a['auc']
        print(50*'#')
        recalls = np.array(a['recalls'])
        precisions = np.array(a['precisions'])
        recalls_01 = recalls[recalls<0.1]
        precisions_01 = precisions[recalls<0.1]

        deltas = recalls_01[1:]-recalls_01[:-1]
        aucs_01[name] = np.dot(deltas,precisions_01[:-1])*10
        

        # plt.plot(a['recalls'], a['precisions'])
        legends.append(d.split('/')[1] + ' - ' + name + ' (auc: {:.4f})'.format(auc))
        # legends.append(d.split('/')[1] + ' - ' + name + ' (auc: {:.4f})'.format(a['auc']) + ' (auc01: {:.4f})'.format(aucs_01[name]))
    else:
        legends.append(base_dir)


all_changed_keys = {k:v for d in all_diff_dicts for k,v in all_diff_dicts[d].items()}
stri = (1+len(all_diff_dicts))*"{:<10}"

print("{:<30}".format("Parameter") + stri.format('default' ,*[d for d in all_diff_dicts]))
for k,v in all_changed_keys.items():
    if '_dictpath' in k:
        keys = v.split('->')
        cfg_tmp = copy.deepcopy(default_config) 
        if keys[0] == "":
            continue
        for k1 in keys:          
            cfg_tmp = cfg_tmp[k1]

        string = "{:<30} {:<10}".format(k[:-9], cfg_tmp[k[:-9]]) 
        for d in all_diff_dicts:
            if k in all_diff_dicts[d]:
                value = all_diff_dicts[d][k[:-9]]
                string += color.GREEN + "{:<10}".format(value) + color.END
            else:
                value = cfg_tmp[k[:-9]]
                string += "{:<10}".format(value)

        print(string)



plt.title('Simulator Precision-Coverage Curves')
plt.legend(legends)
plt.xlabel('Coverage')
plt.ylabel('Precision')
plt.xlim([0, 0.6]) 
plt.ylim([0,1.0])

plt.show()

