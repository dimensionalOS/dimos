import os
import yaml

def recursive_key_value_assign(d,ks,v):
    """
    Recursive value assignment to a nested dict

    Arguments:
        d {dict} -- dict
        ks {list} -- list of hierarchical keys
        v {value} -- value to assign
    """
    
    if len(ks) > 1:
        recursive_key_value_assign(d[ks[0]],ks[1:],v)
    elif len(ks) == 1:
        d[ks[0]] = v
 
def load_config(checkpoint_dir, batch_size=None, max_epoch=None, data_path=None, arg_configs=[], save=False):
    """
    Loads yaml config file and overwrites parameters with function arguments and --arg_config parameters

    Arguments:
        checkpoint_dir {str} -- Checkpoint directory where config file was copied to

    Keyword Arguments:
        batch_size {int} -- [description] (default: {None})
        max_epoch {int} -- "epochs" (number of scenes) to train (default: {None})
        data_path {str} -- path to scenes with contact grasp data (default: {None})
        arg_configs {list} -- Overwrite config parameters by hierarchical command line arguments (default: {[]})
        save {bool} -- Save overwritten config file (default: {False})

    Returns:
        [dict] -- Config
    """

    config_path = os.path.join(checkpoint_dir, 'config.yaml')
    config_path = config_path if os.path.exists(config_path) else os.path.join(os.path.dirname(__file__),'config.yaml')
    with open(config_path,'r') as f:
        global_config = yaml.safe_load(f)

    for conf in arg_configs:
        k_str, v = conf.split(':')
        try:
            v = eval(v)
        except:
            pass
        ks = [int(k) if k.isdigit() else k for k in k_str.split('.')]
        
        recursive_key_value_assign(global_config, ks, v)
        
    if batch_size is not None:
        global_config['OPTIMIZER']['batch_size'] = int(batch_size)
    if max_epoch is not None:
        global_config['OPTIMIZER']['max_epoch'] = int(max_epoch)
    if data_path is not None:
        global_config['DATA']['data_path'] = data_path
        
    global_config['DATA']['classes'] = None
    
    if save:
        with open(os.path.join(checkpoint_dir, 'config.yaml'),'w') as f:
            yaml.dump(global_config, f)

    return global_config

