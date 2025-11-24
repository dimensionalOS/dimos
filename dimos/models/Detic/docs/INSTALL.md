# Installation

### Requirements
- Linux or macOS with Python ≥ 3.6
- PyTorch ≥ 1.8.
  Install them together at [pytorch.org](https://pytorch.org) to make sure of this. Note, please check
  PyTorch version matches that is required by Detectron2.
- Detectron2: follow [Detectron2 installation instructions](https://detectron2.readthedocs.io/tutorials/install.html).


### Example conda environment setup
```bash
conda create --name detic python=3.8 -y
conda activate detic
conda install pytorch torchvision torchaudio cudatoolkit=11.1 -c pytorch-lts -c nvidia

# under your working directory
git clone git@github.com:facebookresearch/detectron2.git
cd detectron2
pip install -e .

cd ..
git clone https://github.com/facebookresearch/Detic.git --recurse-submodules
cd Detic
pip install -r requirements.txt
```

Our project uses two submodules, [CenterNet2](https://github.com/xingyizhou/CenterNet2.git) and [Deformable-DETR](https://github.com/fundamentalvision/Deformable-DETR.git). If you forget to add `--recurse-submodules`, do `git submodule init` and then `git submodule update`. To train models with Deformable-DETR (optional), we need to compile it

```
cd third_party/Deformable-DETR/models/ops
./make.sh
```