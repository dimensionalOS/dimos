from __future__ import print_function
import sys
import numpy as np
from scipy.spatial import cKDTree
import time
import tensorflow as tf
from tqdm import tqdm

N = 5000
k = 1
dim = 3
radius = 0.1

np_matmul_time = []
np_sort_time = []
kd_build_time = []
kd_query_time = []
tf_time = []
tf_pointnet_time = []

@tf.function
def tf_nn(x):
    dists = tf.norm(tf.expand_dims(x,0)-tf.expand_dims(x,1),axis=2)
    _,topk = tf.math.top_k(dists, k=k, sorted=False)
    return topk

# np.random.randint*
@tf.function
def tf_queryball(x):
    dists = tf.norm(tf.expand_dims(x,0)-tf.expand_dims(x,1),axis=2)
    queries = tf.math.less(dists, radius)
    idcs = tf.where(queries)
    return idcs
    # _,topk = tf.math.top_k(dists, k=k, sorted=False)
    # return topk

@tf.function
def tf_knn_max_dist(x):
    
    squared_dists_all = tf.norm(tf.expand_dims(x,0)-tf.expand_dims(x,1),axis=2)#tf.reduce_sum((tf.expand_dims(x,0)-tf.expand_dims(x,1))**2,axis=2)
    neg_squared_dists_k, close_contact_pt_idcs = tf.math.top_k(-squared_dists_all, k=k, sorted=False)
    squared_dists_k = -neg_squared_dists_k
    loss_mask_pc = tf.cast(tf.reduce_mean(squared_dists_k, axis=1) < radius**2, tf.float32)
    return loss_mask_pc

# warmup tf
a=np.random.rand(N,dim).astype(np.float32)
tf_queryball(a)
# tf_pointnet(a)
tf_knn_max_dist(a)

for i in tqdm(range(10)):
    a=np.random.rand(N,dim).astype(np.float32)

    # start_time_tf = time.time()
    # tf_time.append(time.time()-start_time_tf)
    start_time_tf = time.time()
    f= tf_nn(a)
    tf_time.append(time.time()-start_time_tf)
    
    
    start_time_tf = time.time()
    tf_queryball(a)
    tf_pointnet_time.append(time.time()-start_time_tf)

    start_time_np = time.time()
    d=np.linalg.norm(a[np.newaxis,:,:]-a[:,np.newaxis,:],axis=2)
    np_matmul_time.append(time.time() - start_time_np)
    start_time_np = time.time()
    sorted = np.argpartition(d, k, axis=1)
    np_sort_time.append(time.time()-start_time_np)

    start_time_kd = time.time()
    tree = cKDTree(a, leafsize=a.shape[0]+1)
    kd_build_time.append(time.time() - start_time_kd)
    start_time_kd = time.time()
    distances, ndx = tree.query(a, k=k, n_jobs=-1)
    # tree.query_ball_point(a, 0.1)
    kd_query_time.append(time.time()-start_time_kd)

print('np_matmul_time: ', np.mean(np_matmul_time))
print('np_sort_time: ', np.mean(np_sort_time))
print('kd_build_time: ', np.mean(kd_build_time))
print('kd_query_time: ', np.mean(kd_query_time))
print('#'*100)
print('np_brute: ', np.mean(np_sort_time) +  np.mean(np_matmul_time))
print('tf_brute: ', np.mean(tf_time))
print('tf_pointnet: ', np.mean(tf_pointnet_time))
print('kd: ', np.mean(kd_build_time) + np.mean(kd_query_time))

# tf_pointnet_query ball: 0.0019501209259033202 (N=1024, k =20, radius=0.1)
# tf_brute_force_query_ball: 0.0015201091766357422 (N=1024, k=all, radius=0.1)
# tf_brute_force_knn_with_dist_thres: 0.0009593963623046875 (N=1024, k =20, radius=0.1)
# ckdtree_time_total: 0.00626897811889648 (N=1024, k =20)
# Notes:
# 1.) queury ball + sampling is different than knn + dist thres, but for our disjoint sets we should use the latter.
# 2.) If you import the cuda kernels, the tf_brute_force knn becomes 3x slower (0.0024s). That means that the cu kernels also change the tf behavior in different parts of the code.
# 3.) When the pointnet guys created their repo, tf had no gpu implementation of some functions like tf.math.top_k(). Now they have, and it seems to be faster.
