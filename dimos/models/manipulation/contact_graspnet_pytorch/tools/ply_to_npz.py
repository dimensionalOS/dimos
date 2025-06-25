import numpy as np
import sys

def read_ply_file(ply_file, load_cols=True):
    """Naive conversion from ply to numpy arrays

    Arguments:
        ply_file {str} -- [description]

    Keyword Arguments:
        load_cols {bool} -- load vertex colors (default: {True})

    Returns:
        dict -- vertex coordinates and optionally colors 
    """
    ret_dict = {}
    assert ply_file.endswith('.ply')
    with open(ply_file, 'r') as f:
        lines = f.readlines()
        verts_num = int(lines[3].split(' ')[-1])
        verts_lines = lines[11:11 + 2*verts_num:2]
        ret_dict['xyz'] = np.array([list(map(float, l.strip().split(' '))) for l in verts_lines])
        if load_cols:
            cols_lines = lines[12:12 + 2*verts_num:2]
            ret_dict['xyz_color'] = np.array([list(map(int, l.strip().split(' '))) for l in cols_lines])

    return ret_dict
    
file_name = sys.argv[1]
ret_dict = read_ply_file(file_name)

# OpenGL to OpenCV
ret_dict['xyz'][:,1:] = -ret_dict['xyz'][:,1:]

np.savez(file_name.replace('.ply','.npz'), xyz=ret_dict['xyz'], xyz_color=ret_dict['xyz_color'])