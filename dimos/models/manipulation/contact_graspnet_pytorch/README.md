# Contact-GraspNet Pytorch

This is a pytorch implementation of Contact-GraspNet. The original tensorflow
implementation can be found at [https://github.com/NVlabs/contact_graspnet](https://github.com/NVlabs/contact_graspnet).

### Disclaimer
This is not an official implementation of Contact-GraspNet.  The results shown here have been evaluated
empirically and may not match the results in the original paper.  This code is provided as-is and is not
guaranteed to work.  Please use at your own risk.

Additionally, this code implements the core features of Contact-GraspNet as presented
by the authors.  It does not implement all possible configuration as implemented in the original
tensorflow implementation.  If you implement additional features, please consider submitting a pull request.


### Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes
Martin Sundermeyer, Arsalan Mousavian, Rudolph Triebel, Dieter Fox
ICRA 2021

[paper](https://arxiv.org/abs/2103.14127), [project page](https://research.nvidia.com/publication/2021-03_Contact-GraspNet%3A--Efficient), [video](http://www.youtube.com/watch?v=qRLKYSLXElM)

<p align="center">
  <img src="examples/2.gif" width="640" title="UOIS + Contact-GraspNet"/>
</p>

## Installation
This code has been tested with python 3.9.

Create the conda env.
```
conda env create -f contact_graspnet_env.yml
```

Install as a package.
```
pip3 install -e .
```

### Troubleshooting

N/A


### Hardware
Training:
  Tested with 1x Nvidia GPU >= 24GB VRAM.  Reduce batch size if you have less VRAM.

Inference: 1x Nvidia GPU >= 8GB VRAM (might work with less).


## Inference
Model weights are included in the `checkpoints` directory.  Test data can be found in the `test_data` directory.

Contact-GraspNet can directly predict a 6-DoF grasp distribution from a raw scene point cloud. However, to obtain object-wise grasps, remove background grasps and to achieve denser proposals it is highly recommended to use (unknown) object segmentation.  We used [FastSAM](https://github.com/CASIA-IVA-Lab/FastSAM) for unknown object segmentation (in contrast to the original tensorflow implementation which uses [UIOS](https://github.com/chrisdxie/uois)).  Note: Infrastructure for segmentation is not included in this repository.

Given a .npy/.npz file with a depth map (in meters), camera matrix K and (optionally) a 2D segmentation map, execute:

```shell
python contact_graspnet_pytorch/inference.py \
       --np_path="test_data/*.npy" \
       --local_regions --filter_grasps
```

<p align="center">
  <img src="examples/7.png" width="640" title="UOIS + Contact-GraspNet"/>
</p>
Note: This image is from the original Contact-GraspNet repo.  Results may vary.
--> close the window to go to next scene

Given a .npy/.npz file with just a 3D point cloud (in meters), execute [for example](examples/realsense_crop_sigma_001.png):
```shell
python contact_graspnet/inference.py --np_path=/path/to/your/pc.npy \
                                     --forward_passes=5 \
                                     --z_range=[0.2,1.1]
```

`--np_path`: input .npz/.npy file(s) with 'depth', 'K' and optionally 'segmap', 'rgb' keys. For processing a Nx3 point cloud instead use 'xzy' and optionally 'xyz_color' as keys.
`--ckpt_dir`: relative path to checkpooint directory. By default `checkpoint/scene_test_2048_bs3_hor_sigma_001` is used. For very clean / noisy depth data consider `scene_2048_bs3_rad2_32` / `scene_test_2048_bs3_hor_sigma_0025` trained with no / strong noise.
`--local_regions`: Crop 3D local regions around object segments for inference. (only works with segmap)
`--filter_grasps`: Filter grasp contacts such that they only lie on the surface of object segments. (only works with segmap)
`--skip_border_objects` Ignore segments touching the depth map boundary.
`--forward_passes` number of (batched) forward passes. Increase to sample more potential grasp contacts.
`--z_range` [min, max] z values in meter used to crop the input point cloud, e.g. to avoid grasps in the foreground/background(as above).
`--arg_configs TEST.second_thres:0.19 TEST.first_thres:0.23` Overwrite config confidence thresholds for successful grasp contacts to get more/less grasp proposals


## Training

### Set up Acronym Dataset

Follow the instructions at [docs/acronym_setup.md](docs/acronym_setup.md) to set up the Acronym dataset.

### Set Environment Variables
When training on a headless server set the environment variable
```shell
export PYOPENGL_PLATFORM='egl'
```
This is also done automatically in the training script.

### Quickstart Training

Start training with config `contact_graspnet_pytorch/config.yaml`
```
python3 contact_graspnet_pytorch/train.py --data_path acronym/
```

### Additional Training Options

To set a custom model name and custom data path:

```
python contact_graspnet/train.py --ckpt_dir checkpoints/your_model_name \
                                 --data_path /path/to/acronym/data
```

To restart a previous batch
```
python contact_graspnet/train.py --ckpt_dir checkpoints/previous_model_name \
                                 --data_path /path/to/acronym/data
```

### Generate Contact Grasps and Scenes yourself (optional)

See [docs/generate_scenes.md](docs/generate_scenes.md) for instructions on how to generate scenes and grasps yourself.

## Citation
If you find this work useful, please consider citing the author's original work and starring this repo.

```
@article{sundermeyer2021contact,
  title={Contact-GraspNet: Efficient 6-DoF Grasp Generation in Cluttered Scenes},
  author={Sundermeyer, Martin and Mousavian, Arsalan and Triebel, Rudolph and Fox, Dieter},
  booktitle={2021 IEEE International Conference on Robotics and Automation (ICRA)},
  year={2021}
}
```
