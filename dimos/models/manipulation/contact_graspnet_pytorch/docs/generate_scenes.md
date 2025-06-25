# Generate New Scenes

## Introduction
This is an optional procedure to generate custom scenes using the grasps
from the acronym dataset.  Instructions are based off the original instructions
in the Contact-GraspNet repo

The scene_contacts downloaded above are generated from the Acronym dataset. To generate/visualize table-top scenes yourself, also pip install the [acronym_tools](https://github.com/NVlabs/acronym) package in your conda environment as described in the acronym repository.

## Generate Contact Points


DOES NOT WORK YET

First, object-wise 6-DoF grasps are mapped to their contact points saved in mesh_contacts.
These will be used when we train the network.

This assumes your data is in the `acronym/` directory.  If you have a different
data directory, change the path accordingly.

```
python3 tools/create_contact_infos.py acronym/
```