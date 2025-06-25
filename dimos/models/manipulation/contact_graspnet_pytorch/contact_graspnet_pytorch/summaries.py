
import numpy as np
import os

try:
    import tensorflow.compat.v1 as tf
    tf.disable_eager_execution()
    TF2 = True
except:
    import tensorflow as tf
    TF2 = False

from tensorboard import summary as summary_lib


def top_grasp_acc_summaries(ops, thres=[0.62,0.65,0.68]):
    """
    Calculates the accuracy of grasp contact predictions at different thresholds

    Arguments:
        ops {dict} -- dict of tf tensors

    Keyword Arguments:
        thres {list} -- thresholds to report (default: {[0.62,0.65,0.68]})

    Returns:
        [tf.variable] -- summary update ops
    """
    
    binary_seg = tf.reshape(ops['binary_seg_pred'],[ops['binary_seg_pred'].shape[0],ops['binary_seg_pred'].shape[1]])
    max_conf_idcs = tf.argmax(binary_seg, axis=1)
    max_confs = tf.math.reduce_max(binary_seg, axis=1)

    row_idcs = tf.cast(tf.range(ops['grasp_suc_labels_pc'].shape[0]), tf.int64)
    nd_gather_idcs = tf.stack((row_idcs, max_conf_idcs), axis=1)
    top_labels = tf.gather_nd(ops['grasp_suc_labels_pc'], nd_gather_idcs)

    acc_update_ops = [0] * len(thres)
    accs = [0] * len(thres)
    for i,thr in enumerate(thres):
        accs[i], acc_update_ops[i] = tf.metrics.accuracy(top_labels, tf.greater(max_confs, tf.constant(thr)))
        tf.summary.scalar('Top Grasp Accuracy Thres %s' % thr, accs[i])

    return acc_update_ops

def build_summary_ops(ops, sess, global_config):
    """
    Collect tensorboard summaries

    Arguments:
        ops {dict} -- dict of tf tensors
        sess {tf.Session} -- tf session
        global_config {dict} -- global config

    Returns:
        [dict] -- summary update ops
    """

    dir_loss = ops['dir_loss']
    bin_ce_loss = ops['bin_ce_loss']
    grasp_suc_labels_pc = ops['grasp_suc_labels_pc']
    binary_seg_pred = ops['binary_seg_pred']

    tf.summary.scalar('total loss', ops['loss'])
    tf.summary.scalar('dir_loss', ops['dir_loss'])
    tf.summary.scalar('approach_loss', ops['approach_loss'])
    tf.summary.scalar('adds_loss', ops['adds_loss'])
    tf.summary.scalar('adds_gt2pred_loss', ops['adds_gt2pred_loss'])
    tf.summary.scalar('bin_ce_loss', ops['bin_ce_loss'])
    tf.summary.scalar('offset_loss_x%s' % global_config['OPTIMIZER']['offset_loss_weight'], ops['offset_loss'])

    tf.summary.histogram('labels_grasp_suc', ops['grasp_suc_labels_pc'])
    tf.summary.histogram('preds_grasp_suc', ops['binary_seg_pred'])

    tf.summary.histogram('offset_predictions', ops['grasp_offset_pred'])
    tf.summary.histogram('offset_labels', ops['offset_labels_pc'])

    # classification:
    if global_config['MODEL']['bin_offsets']:

        tf.summary.histogram('offset_predictions_binned', ops['offset_pred_idcs_pc'])
        tf.summary.histogram('offset_labels_binned', ops['offset_label_idcs_pc'])
        tf.summary.scalar('offset_bin_classification_accuracy', tf.reduce_mean(tf.cast(tf.equal(ops['offset_pred_idcs_pc'], ops['offset_label_idcs_pc']),tf.float32)))

        bin_mean_preds = ops['offset_bin_pred_vals']
        tf.summary.scalar('offset_mean_abs_error', tf.reduce_mean(tf.abs(bin_mean_preds - tf.layers.flatten(tf.abs(ops['offset_orig_labels_vals'])))))
        tf.summary.scalar('offset_max_abs_error', tf.reduce_max(tf.abs(bin_mean_preds - tf.layers.flatten(tf.abs(ops['offset_orig_labels_vals'])))))

    merged = tf.summary.merge_all()

    acc_update_ops = top_grasp_acc_summaries(ops, thres=[0.62,0.65,0.68])

    auc, auc_update_op = tf.metrics.auc(tf.layers.flatten(tf.cast(ops['grasp_suc_labels_pc'], tf.bool)), 
                                        tf.layers.flatten(ops['binary_seg_pred']), curve='PR', summation_method='careful_interpolation')
    tf.summary.scalar('AUCPR', auc)

    pr_curve_streaming_func = summary_lib.v1.pr_curve_streaming_op if TF2 else summary_lib.pr_curve_streaming_op
    _, pr_update_op = pr_curve_streaming_func(name='pr_contact_success',
                                                predictions=tf.layers.flatten(ops['binary_seg_pred']),
                                                labels=tf.layers.flatten(tf.cast(ops['grasp_suc_labels_pc'], tf.bool)),
                                                num_thresholds = 201)

    pr_reset_op = tf.variables_initializer(tf.get_collection(tf.GraphKeys.METRIC_VARIABLES))
    merged_eval = tf.summary.merge_all()
    
    sess.run(tf.local_variables_initializer())

    summary_ops = {'merged': merged, 'merged_eval': merged_eval, 'pr_update_op':pr_update_op, 'auc_update_op':auc_update_op, 'acc_update_ops':acc_update_ops, 'pr_reset_op': pr_reset_op}
    return summary_ops

def build_file_writers(sess, log_dir):
    """
    Create TF FileWriters for train and test

    Arguments:
        sess {tf.Session} -- tf session
        log_dir {str} -- Checkpoint directory

    Returns:
        [dict] -- tf.summary.FileWriter
    """

    train_writer = tf.summary.FileWriter(os.path.join(log_dir, 'train'))#, sess.graph) creates large event files!
    test_writer = tf.summary.FileWriter(os.path.join(log_dir, 'test'))#, sess.graph)
    writers ={'train_writer': train_writer, 'test_writer': test_writer}

    return writers