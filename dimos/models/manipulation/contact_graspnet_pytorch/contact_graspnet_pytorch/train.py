from genericpath import exists
import os
import sys
import argparse
from datetime import datetime
import numpy as np
import time
from tqdm import tqdm
from tensorboardX import SummaryWriter

import torch

os.environ['PYOPENGL_PLATFORM'] = 'egl'  # To get pyrender to work headless

# Import pointnet library
CONTACT_DIR = os.path.dirname(os.path.abspath(__file__))
BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ROOT_DIR = os.path.dirname(os.path.dirname(BASE_DIR))

sys.path.append(os.path.join(BASE_DIR))
sys.path.append(os.path.join(BASE_DIR, 'Pointnet_Pointnet2_pytorch'))

import config_utils
from acronym_dataloader import AcryonymDataset
from contact_graspnet_pytorch.contact_graspnet import ContactGraspnet, ContactGraspnetLoss
from contact_graspnet_pytorch import utils
from contact_graspnet_pytorch.checkpoints import CheckpointIO 

def train(global_config, log_dir):
    """
    Trains Contact-GraspNet.  Configure the training process by modifying the 
    config.yaml file.

    Arguments:
        global_config {dict} -- config dict
        log_dir {str} -- Checkpoint directory

    """
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    # device = torch.device('cpu')
    batch_size = global_config['OPTIMIZER']['batch_size']
    num_workers = 0  # Increase after debug
    train_dataset = AcryonymDataset(global_config, train=True, device=device, use_saved_renders=True)
    test_dataset = AcryonymDataset(global_config, train=False, device=device, use_saved_renders=True)

    train_dataloader = torch.utils.data.DataLoader(train_dataset,
                                                   batch_size=batch_size,
                                                   shuffle=True,
                                                   num_workers=num_workers)
    test_dataloader = torch.utils.data.DataLoader(test_dataset,
                                                    batch_size=batch_size,
                                                    shuffle=True,
                                                    num_workers=num_workers)

    grasp_estimator = ContactGraspnet(global_config, device).to(device)
    loss_fn = ContactGraspnetLoss(global_config, device).to(device)
    opt = torch.optim.Adam(grasp_estimator.parameters(),
                           lr=global_config['OPTIMIZER']['learning_rate'])
    
    logger = SummaryWriter(os.path.join(log_dir, 'logs'))
    checkpoint_dir = os.path.join(log_dir, 'checkpoints')
    checkpoint_io = CheckpointIO(checkpoint_dir, model=grasp_estimator, opt=opt)

    try:
        load_dict = checkpoint_io.load('model.pt')
    except FileExistsError:
        load_dict = dict()
    
    cur_epoch = load_dict.get('epoch_it', 0)
    it = load_dict.get('it', 0)
    metric_val_best = load_dict.get('loss_val_best', np.inf)

    print_every = global_config['OPTIMIZER']['print_every'] \
        if 'print_every' in global_config['OPTIMIZER'] else 0
    checkpoint_every = global_config['OPTIMIZER']['checkpoint_every'] \
        if 'checkpoint_every' in global_config['OPTIMIZER'] else 0
    backup_every = global_config['OPTIMIZER']['backup_every'] \
        if 'backup_every' in global_config['OPTIMIZER'] else 0
    val_every = global_config['OPTIMIZER']['val_every'] \
        if 'val_every' in global_config['OPTIMIZER'] else 0

    for epoch_it in range(cur_epoch, global_config['OPTIMIZER']['max_epoch']):
        log_string('**** EPOCH %03d ****' % epoch_it)
        grasp_estimator.train()
        
        # # Start when we left of in dataloader 
        # skip_amount = it % len(train_dataloader)

        pbar = tqdm(train_dataloader)
        for i, data in enumerate(pbar):
            # if i < skip_amount:
            #     print('skipping')
            #     continue 

            utils.send_dict_to_device(data, device)
            # Target contains input and target values
            pc_cam = data['pc_cam']

            pred = grasp_estimator(pc_cam)
            loss, loss_info = loss_fn(pred, data)
            opt.zero_grad()
            loss.backward()
            opt.step()

            for k, v in loss_info.items():
                logger.add_scalar(f'train/{k}', v, it)

            # -- Logging -- #
            # if print_every and it % print_every == 0:
                # print('[Epoch %02d] it=%03d, loss=%.4f, adds_loss=%.4f'% (epoch_it, it, loss, loss_info['adds_loss']))
            
            if checkpoint_every and it % checkpoint_every == 0:
                checkpoint_io.save('model.pt', epoch_it=epoch_it, it=it,
                    loss_val_best=metric_val_best)
            
            if backup_every and it % backup_every == 0:
                checkpoint_io.save('model_%d.pt' % it, epoch_it=epoch_it, it=it,
                    loss_val_best=metric_val_best)
            
            logger.add_scalar('train/loss', loss.item(), it)
            pbar.set_postfix({'loss': loss.item(),
                              'epoch': epoch_it})

            it += 1

        # -- Run Validation -- #
        if val_every and epoch_it % val_every == 0:
            grasp_estimator.eval()
            with torch.no_grad():
                loss_log = []
                for val_it, data in enumerate(tqdm(test_dataloader)):
                    utils.send_dict_to_device(data, device)
                    # Target contains input and target values
                    pc_cam = data['pc_cam']

                    pred = grasp_estimator(pc_cam)
                    loss, loss_info = loss_fn(pred, data)
                    loss_log.append(loss.item())
                val_loss = np.mean(loss_log)
                logger.add_scalar('val/val_loss', val_loss, it)
        

            if val_loss < metric_val_best:
                metric_val_best = val_loss
                checkpoint_io.save('model_best.pt', epoch_it=epoch_it, it=it,
                    loss_val_best=metric_val_best)


if __name__ == "__main__":
    # Usage: 
    # To continue training: python train_pytorch.py --ckpt_dir {Pcurrent_ckpt_dir}
    # e.g. python3 train_pytorch.py --ckpt_dir ../checkpoints/contact_graspnet_2
    # To start training from scratch: python train_pytorch.py


    parser = argparse.ArgumentParser()
    parser.add_argument('--ckpt_dir', type=str, default=None, help='Checkpoint dir')
    parser.add_argument('--data_path', type=str, default=None, help='Grasp data root dir')
    parser.add_argument('--max_epoch', type=int, default=None, help='Epochs to run')
    parser.add_argument('--batch_size', type=int, default=None, help='Batch Size during training')
    parser.add_argument('--arg_configs', nargs="*", type=str, default=[], help='overwrite config parameters')
    FLAGS = parser.parse_args()

    ckpt_dir = FLAGS.ckpt_dir
    if ckpt_dir is None:
        # ckpt_dir is contact_graspnet_year_month_day_hour_minute_second
        ckpt_dir = os.path.join(CONTACT_DIR, '../', f'checkpoints/contact_graspnet_{datetime.now().strftime("Y%YM%mD%d_H%HM%M")}')
    
    data_path = FLAGS.data_path
    if data_path is None:
        data_path = os.path.join(CONTACT_DIR, '../', 'acronym/')

    if not os.path.exists(ckpt_dir):
        if not os.path.exists(os.path.dirname(ckpt_dir)):
            ckpt_dir = os.path.join(BASE_DIR, ckpt_dir)
        os.makedirs(ckpt_dir, exist_ok=True)

    os.environ["CUDA_DEVICE_ORDER"] = "PCI_BUS_ID"
    os.system('cp {} {}'.format(os.path.join(CONTACT_DIR, 'contact_graspnet.py'), ckpt_dir)) # bkp of model def
    os.system('cp {} {}'.format(os.path.join(CONTACT_DIR, 'train.py'), ckpt_dir)) # bkp of train procedure

    LOG_FOUT = open(os.path.join(ckpt_dir, 'log_train.txt'), 'w')
    LOG_FOUT.write(str(FLAGS)+'\n')
    def log_string(out_str):
        LOG_FOUT.write(out_str+'\n')
        LOG_FOUT.flush()
        print(out_str)

    global_config = config_utils.load_config(ckpt_dir, batch_size=FLAGS.batch_size, max_epoch=FLAGS.max_epoch,
                                          data_path= FLAGS.data_path, arg_configs=FLAGS.arg_configs, save=True)

    log_string(str(global_config))
    log_string('pid: %s'%(str(os.getpid())))

    train(global_config, ckpt_dir)

    LOG_FOUT.close()
