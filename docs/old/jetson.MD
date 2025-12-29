# DimOS Jetson Setup Instructions
Tested on Jetpack 6.2, CUDA 12.6

## Required system dependencies
`sudo apt install portaudio19-dev python3-pyaudio`

## Installing cuSPARSELt
https://ninjalabo.ai/blogs/jetson_pytorch.html

```bash
wget https://developer.download.nvidia.com/compute/cusparselt/0.7.0/local_installers/cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
sudo dpkg -i cusparselt-local-tegra-repo-ubuntu2204-0.7.0_1.0-1_arm64.deb
sudo cp /var/cusparselt-local-tegra-repo-ubuntu2204-0.7.0/cusparselt-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get install libcusparselt0 libcusparselt-dev
ldconfig
```
## Install Torch and Torchvision wheels

Enter virtualenv
```bash
python3 -m venv venv
source venv/bin/activate
```

Wheels for jp6/cu126
https://pypi.jetson-ai-lab.io/jp6/cu126

Check compatibility:
https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html

### Working torch wheel tested on Jetpack 6.2, CUDA 12.6
`pip install --no-cache https://developer.download.nvidia.com/compute/redist/jp/v61/pytorch/torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl`

### Install torchvision from source:
```bash
# Set version by checking above torchvision<-->torch compatibility

# We use 0.20.0
export VERSION=20

sudo apt-get install libjpeg-dev zlib1g-dev libpython3-dev libopenblas-dev libavcodec-dev libavformat-dev libswscale-dev
git clone --branch release/0.$VERSION https://github.com/pytorch/vision torchvision
cd torchvision
export BUILD_VERSION=0.$VERSION.0
python3 setup.py install --user # remove --user if installing in virtualenv
```

### Verify success:
```bash
$ python3
import torch
print(torch.__version__)
print('CUDA available: ' + str(torch.cuda.is_available()))  # Should be True
print('cuDNN version: ' + str(torch.backends.cudnn.version()))
a = torch.cuda.FloatTensor(2).zero_()
print('Tensor a = ' + str(a))
b = torch.randn(2).cuda()
print('Tensor b = ' + str(b))
c = a + b
print('Tensor c = ' + str(c))

$ python3
import torchvision
print(torchvision.__version__)
```

## Install Onnxruntime-gpu

Find pre-build wheels here for your specific JP/CUDA version: https://pypi.jetson-ai-lab.io/jp6

`pip install https://pypi.jetson-ai-lab.io/jp6/cu126/+f/4eb/e6a8902dc7708/onnxruntime_gpu-1.23.0-cp310-cp310-linux_aarch64.whl#sha256=4ebe6a8902dc7708434b2e1541b3fe629ebf434e16ab5537d1d6a622b42c622b`
