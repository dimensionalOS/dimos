# Third-party content in the DimOS AppImage

DimOS is licensed under the Apache License 2.0, see `LICENSE` in this folder. The
image also redistributes the work of others. The files in this folder say what, and
under which terms.

| File | What it lists |
|---|---|
| `data.tsv` | The data archives unpacked in `opt/data`. |
| `models.tsv` | The models in `opt/models`. |
| `robot_descriptions.tsv` | The robot descriptions in `opt/robot_assets`. |
| `python-packages.tsv` | The Python packages in `opt/python`. Each package keeps its licence files in its `.dist-info` folder. |
| `native-libraries.tsv` | The libraries next to the native modules in `opt/native`, with the nixpkgs package each one comes from. |
| `system-libraries.tsv` | The libraries in `usr/lib`, with the Ubuntu package each one comes from. The `.copyright` files are those packages' copyright files. |

## Attributions

### Mersus Office

`opt/data/mujoco_sim/scene_office1` and `scene_office1.xml` are made from "Mersus
Office" by Mersus Technologies, licensed under
[CC BY 4.0](https://creativecommons.org/licenses/by/4.0/).
Source: <https://sketchfab.com/3d-models/mersus-office-8714be387bcd406898b2615f7dae3a47>.
Changes: the model was split into parts and converted to OBJ meshes. Convex collision
shapes and a MuJoCo scene description were added.

### MuJoCo Menagerie

`opt/data/mujoco_menagerie` holds the Unitree Go1 and Unitree G1 meshes of the
[MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie) at commit
`1b86ece576591213e2b666ebf59508454200ca97`. They are licensed under the BSD 3-Clause
License. The `LICENSE` file of each model is next to its meshes.
