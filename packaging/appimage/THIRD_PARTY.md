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

### Licence files added by the build

The archives of these folders hold no licence file. The build adds one to each.

| Folder in `opt/data` | Added | Terms | Where the text is from |
|---|---|---|---|
| `groot` | `LICENSE`, `Notice.txt` | NVIDIA Open Model License. `Notice.txt` is the attribution that it asks for. | [GR00T-WholeBodyControl](https://github.com/NVlabs/GR00T-WholeBodyControl/blob/1e1ec4f5f27e6c3e03048f693bd7c60cce04afda/decoupled_wbc/sim2mujoco/resources/robots/g1/policy/NVIDIA%20Open%20Model%20License) |
| `piper` | `LICENSE` | MIT, for the AgileX model. | [MuJoCo Menagerie](https://github.com/google-deepmind/mujoco_menagerie/blob/eea4c7431b63da8278b0ff31638e0b0e941cec0e/agilex_piper/LICENSE) |
| `g1_urdf` | `LICENSE` | BSD 3-Clause, for the Unitree model. | `go2_sim/unitree_go2/LICENSE`, the same Unitree text. |
| `xarm6` | `LICENSE` | BSD 3-Clause, for the UFACTORY meshes. | `xarm7/LICENSE`, the same UFACTORY text. |
| `yam_description` | `LICENSE` | Apache-2.0, for the OpenYAM model. Upstream declares it in `package.xml` and has no licence file, as `SOURCE_COMMIT.txt` says. | The standard Apache-2.0 text. |
| `yam_description/i2rt` | `LICENSE` | MIT, for the I2RT model. | `dual_openyam_abc_box_v2/I2RT_YAM_LICENSE`, the same I2RT text. |
| `models_edgetam` | `LICENSE` | Apache-2.0, for the EdgeTAM checkpoint. | The licence file of the `edgetam_dimos` package. |

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
