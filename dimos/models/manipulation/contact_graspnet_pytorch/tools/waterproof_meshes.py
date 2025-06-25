"""
convert_meshes.py
Process the .obj files and make them waterproof and simplified.

Based on: https://github.com/NVlabs/acronym/issues/6
"""
import os
import glob
import yaml
import subprocess
from tqdm import tqdm
from multiprocessing import Pool, cpu_count

BASE_PATH = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ACRONYM_PATH = os.path.join(BASE_PATH, 'acronym')
GRASPDIR = os.path.join(ACRONYM_PATH, "grasps")
MANIFOLD_PATH = os.path.join(ACRONYM_PATH, "Manifold", "build", "manifold")
SIMPLIFY_PATH = os.path.join(ACRONYM_PATH, "Manifold", "build", "simplify")

# -- Create log for failed files -- #
failed_file = os.path.join(ACRONYM_PATH, 'failed.txt')
if not os.path.isfile(failed_file):
    with open(failed_file, 'w') as f:
        f.write('')

dne_file = os.path.join(ACRONYM_PATH, 'dne.txt')
if not os.path.isfile(dne_file):
    with open(dne_file, 'w') as f:
        f.write('')

# -- Get all hashes -- #
hashes = []
for fname in os.listdir(GRASPDIR):
    tokens = fname.split("_")
    assert(len(tokens) == 3)
    hashes.append(tokens[1])

print(len(hashes))
print(len(os.listdir(os.path.join(ACRONYM_PATH, 'models'))))

# -- Create output directory if it doesn't exist -- #
out_name = 'meshes'
if not os.path.exists(os.path.join(ACRONYM_PATH, out_name)):
    os.makedirs(os.path.join(ACRONYM_PATH, out_name))

REGENENERATE = [
    # '49a0a84ee5a91344b11c5cce13f76151',
    # 'feb146982d0c64dfcbf4f3f04bbad8',
    # '202fd2497d2e85f0dd6c14adedcbd4c3',
    # '47fc4a2417ca259f894dbff6c6a6be89',
    # '2a7d62b731a04f5fa54b9afa882a89ed',
    # '8123f469a08a88e7761dc3477e65a72',

    # '202fd2497d2e85f0dd6c14adedcbd4c3'
    # 'feb146982d0c64dfcbf4f3f04bbad8',
    # '47fc4a2417ca259f894dbff6c6a6be89',
    # 'feb146982d0c64dfcbf4f3f04bbad8',
    # '2a7d62b731a04f5fa54b9afa882a89ed',
    # 'b54e412afd42fe6da7c64d6a7060b75b',
    '41efae18a5376bb4fc50236094ae9e18',
]

def write_failed(h):
    with open(failed_file, 'a') as f:
        f.write(f'{h}\n')

def write_dne(h):
    with open(dne_file, 'a') as f:
        f.write(f'{h}\n')

def remove_temp(temp_name):
    if os.path.isfile(temp_name):
        os.remove(temp_name)

## Define function to process a single file
def process_hash(h):
    """Process a single object file by calling a subshell with the mesh processing script.

    Args:
        h (string): the hash denoting the file type
    """
    obj = os.path.join(ACRONYM_PATH, 'models/', h + ".obj")
    temp_name = os.path.join(ACRONYM_PATH, f"temp.{h}.watertight.obj")
    outfile = os.path.join(ACRONYM_PATH, "meshes/", h + ".obj")
    outfile_search = os.path.join(ACRONYM_PATH, "meshes/", '*/', h + ".obj")
    
    if h in REGENENERATE:
        print(f'Regenerating: {h}')
    else:
        # File already done
        # if os.path.isfile(outfile):
        if glob.glob(outfile_search) or glob.glob(outfile):
            # print(f'{outfile} already done')
            # Get rid of existing temp files
            if os.path.isfile(temp_name):
                os.remove(temp_name)
                print(f'removed: {temp_name}')
            return

        if not os.path.isfile(obj):
            print(f'File does not exist: {h}')
            write_dne(h)
            return
    
    # Waterproof the object
    completed = subprocess.CompletedProcess(args=[], returncode=0)
    if not os.path.isfile(temp_name):
        completed = subprocess.run(["timeout", "-sKILL", "30", MANIFOLD_PATH, obj, temp_name], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    watertight_conversion = completed.returncode
    if completed.returncode != 0:
        print(f"Skipping object (manifold failed): {h}")
        write_failed(h)
        remove_temp(temp_name)
        return
            
    # Simplify the object
    completed = subprocess.run(["timeout", "-sKILL", "60", SIMPLIFY_PATH, "-i", temp_name, "-o", outfile, "-m", "-r", "0.02"],  stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    if completed.returncode != 0:
        print(f"Skipping object (simplify failed): {h}")
        write_failed(h)
        remove_temp(temp_name)
        return

    if not os.path.exists(outfile):
        print(f"Skipping object (outfile not created): {h}")
        write_failed(h)
        remove_temp(temp_name)
        return

    print(f"Finished object: {h}")
    remove_temp(temp_name)

# -- Issue the commands in a multiprocessing pool -- #
with Pool(cpu_count()-4) as p:
# with Pool(1) as p:
    examples = list(
        tqdm(
            p.imap_unordered(process_hash, hashes),
            total=len(hashes)
        )
    )
