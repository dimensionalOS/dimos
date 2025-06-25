import os
import tqdm


BASE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACRONYM_PATH = os.path.join(BASE_PATH, 'acronym')
GRASPDIR = os.path.join(ACRONYM_PATH, "grasps")
MESHDIR = os.path.join(ACRONYM_PATH, "meshes")

with open(os.path.join(ACRONYM_PATH, 'failed.txt'), 'r') as f:
    failed_meshes = f.readlines()
    failed_meshes = [line.strip() for line in failed_meshes]

for fname in tqdm.tqdm(os.listdir(GRASPDIR)):
    tokens = fname.split("_")
    assert(len(tokens) == 3)
    obj_type = tokens[0]
    obj_hash = tokens[1]
    obj_scale = tokens[2].split(".")[0]

    if not os.path.exists(os.path.join(MESHDIR, f'{obj_type}')):
        os.makedirs(os.path.join(MESHDIR, f'{obj_type}'))
        # move object to folder
    
    ori_path = os.path.join(MESHDIR, f'{obj_hash}.obj')
    new_path = os.path.join(MESHDIR, f'{obj_type}', f'{obj_hash}.obj')
    if not os.path.exists(new_path) and obj_hash not in failed_meshes:
        os.rename(ori_path, new_path)
