# Deformable DETR

By [Xizhou Zhu](https://scholar.google.com/citations?user=02RXI00AAAAJ),  [Weijie Su](https://www.weijiesu.com/),  [Lewei Lu](https://www.linkedin.com/in/lewei-lu-94015977/), [Bin Li](http://staff.ustc.edu.cn/~binli/), [Xiaogang Wang](http://www.ee.cuhk.edu.hk/~xgwang/), [Jifeng Dai](https://jifengdai.org/).

This repository is an official implementation of the paper [Deformable DETR: Deformable Transformers for End-to-End Object Detection](https://arxiv.org/abs/2010.04159).


## Introduction

**TL; DR.** Deformable DETR is an efficient and fast-converging end-to-end object detector. It mitigates the high complexity and slow convergence issues of DETR via a novel sampling-based efficient attention mechanism.  

![deformable_detr](./figs/illustration.png)

![deformable_detr](./figs/convergence.png)

**Abstract.** DETR has been recently proposed to eliminate the need for many hand-designed components in object detection while demonstrating good performance. However, it suffers from slow convergence and limited feature spatial resolution, due to the limitation of Transformer attention modules in processing image feature maps. To mitigate these issues, we proposed Deformable DETR, whose attention modules only attend to a small set of key sampling points around a reference. Deformable DETR can achieve better performance than DETR (especially on small objects) with 10× less training epochs. Extensive experiments on the COCO benchmark demonstrate the effectiveness of our approach.

## License

This project is released under the [Apache 2.0 license](./LICENSE).

## Changelog

See [changelog.md](./docs/changelog.md) for detailed logs of major changes. 


## Citing Deformable DETR
If you find Deformable DETR useful in your research, please consider citing:
```bibtex
@article{zhu2020deformable,
  title={Deformable DETR: Deformable Transformers for End-to-End Object Detection},
  author={Zhu, Xizhou and Su, Weijie and Lu, Lewei and Li, Bin and Wang, Xiaogang and Dai, Jifeng},
  journal={arXiv preprint arXiv:2010.04159},
  year={2020}
}
```

## Main Results

| <sub><sub>Method</sub></sub>   | <sub><sub>Epochs</sub></sub> | <sub><sub>AP</sub></sub> | <sub><sub>AP<sub>S</sub></sub></sub> | <sub><sub>AP<sub>M</sub></sub></sub> | <sub><sub>AP<sub>L</sub></sub></sub> | <sub><sub>params<br>(M)</sub></sub> | <sub><sub>FLOPs<br>(G)</sub></sub> | <sub><sub>Total<br>Train<br>Time<br>(GPU<br/>hours)</sub></sub> | <sub><sub>Train<br/>Speed<br>(GPU<br/>hours<br/>/epoch)</sub></sub> | <sub><sub>Infer<br/>Speed<br/>(FPS)</sub></sub> | <sub><sub>Batch<br/>Infer<br/>Speed<br>(FPS)</sub></sub> | <sub><sub>URL</sub></sub>                     |
| ----------------------------------- | :----: | :--: | :----: | :---: | :------------------------------: | :--------------------:| :----------------------------------------------------------: | :--: | :---: | :---: | ----- | ----- |
| <sub><sub>Faster R-CNN + FPN</sub></sub> | <sub>109</sub> | <sub>42.0</sub> | <sub>26.6</sub> | <sub>45.4</sub> | <sub>53.4</sub> | <sub>42</sub> | <sub>180</sub> | <sub>380</sub> | <sub>3.5</sub> | <sub>25.6</sub> | <sub>28.0</sub> | <sub>-</sub> |
| <sub><sub>DETR</sub></sub> | <sub>500</sub> | <sub>42.0</sub> | <sub>20.5</sub> | <sub>45.8</sub> | <sub>61.1</sub> | <sub>41</sub> | <sub>86</sub> | <sub>2000</sub> | <sub>4.0</sub> |     <sub>27.0</sub>       |         <sub>38.3</sub>           | <sub>-</sub> |
| <sub><sub>DETR-DC5</sub></sub>      | <sub>500</sub> | <sub>43.3</sub> | <sub>22.5</sub> | <sub>47.3</sub> | <sub>61.1</sub> | <sub>41</sub> |<sub>187</sub>|<sub>7000</sub>|<sub>14.0</sub>|<sub>11.4</sub>|<sub>12.4</sub>| <sub>-</sub> |
| <sub><sub>DETR-DC5</sub></sub>      | <sub>50</sub> | <sub>35.3</sub> | <sub>15.2</sub> | <sub>37.5</sub> | <sub>53.6</sub> | <sub>41</sub> |<sub>187</sub>|<sub>700</sub>|<sub>14.0</sub>|<sub>11.4</sub>|<sub>12.4</sub>| <sub>-</sub> |
| <sub><sub>DETR-DC5+</sub></sub>     | <sub>50</sub> | <sub>36.2</sub> | <sub>16.3</sub> | <sub>39.2</sub> | <sub>53.9</sub> | <sub>41</sub> |<sub>187</sub>|<sub>700</sub>|<sub>14.0</sub>|<sub>11.4</sub>|<sub>12.4</sub>| <sub>-</sub> |
| **<sub><sub>Deformable DETR<br>(single scale)</sub></sub>** | <sub>50</sub> | <sub>39.4</sub> | <sub>20.6</sub> | <sub>43.0</sub> | <sub>55.5</sub> | <sub>34</sub> |<sub>78</sub>|<sub>160</sub>|<sub>3.2</sub>|<sub>27.0</sub>|<sub>42.4</sub>| <sub>[config](./configs/r50_deformable_detr_single_scale.sh)<br/>[log](https://drive.google.com/file/d/1n3ZnZ-UAqmTUR4AZoM4qQntIDn6qCZx4/view?usp=sharing)<br/>[model](https://drive.google.com/file/d/1WEjQ9_FgfI5sw5OZZ4ix-OKk-IJ_-SDU/view?usp=sharing)</sub> |
| **<sub><sub>Deformable DETR<br>(single scale, DC5)</sub></sub>** | <sub>50</sub> | <sub>41.5</sub> | <sub>24.1</sub> | <sub>45.3</sub> | <sub>56.0</sub> | <sub>34</sub> |<sub>128</sub>|<sub>215</sub>|<sub>4.3</sub>|<sub>22.1</sub>|<sub>29.4</sub>| <sub>[config](./configs/r50_deformable_detr_single_scale_dc5.sh)<br/>[log](https://drive.google.com/file/d/1-UfTp2q4GIkJjsaMRIkQxa5k5vn8_n-B/view?usp=sharing)<br/>[model](https://drive.google.com/file/d/1m_TgMjzH7D44fbA-c_jiBZ-xf-odxGdk/view?usp=sharing)</sub> |
| **<sub><sub>Deformable DETR</sub></sub>** | <sub>50</sub> | <sub>44.5</sub> | <sub>27.1</sub> | <sub>47.6</sub> | <sub>59.6</sub> | <sub>40</sub> |<sub>173</sub>|<sub>325</sub>|<sub>6.5</sub>|<sub>15.0</sub>|<sub>19.4</sub>|<sub>[config](./configs/r50_deformable_detr.sh)<br/>[log](https://drive.google.com/file/d/18YSLshFjc_erOLfFC-hHu4MX4iyz1Dqr/view?usp=sharing)<br/>[model](https://drive.google.com/file/d/1nDWZWHuRwtwGden77NLM9JoWe-YisJnA/view?usp=sharing)</sub>                   |
| **<sub><sub>+ iterative bounding box refinement</sub></sub>** | <sub>50</sub> | <sub>46.2</sub> | <sub>28.3</sub> | <sub>49.2</sub> | <sub>61.5</sub> | <sub>41</sub> |<sub>173</sub>|<sub>325</sub>|<sub>6.5</sub>|<sub>15.0</sub>|<sub>19.4</sub>|<sub>[config](./configs/r50_deformable_detr_plus_iterative_bbox_refinement.sh)<br/>[log](https://drive.google.com/file/d/1DFNloITi1SFBWjYzvVEAI75ndwmGM1Uj/view?usp=sharing)<br/>[model](https://drive.google.com/file/d/1JYKyRYzUH7uo9eVfDaVCiaIGZb5YTCuI/view?usp=sharing)</sub> |
| **<sub><sub>++ two-stage Deformable DETR</sub></sub>** | <sub>50</sub> | <sub>46.9</sub> | <sub>29.6</sub> | <sub>50.1</sub> | <sub>61.6</sub> | <sub>41</sub> |<sub>173</sub>|<sub>340</sub>|<sub>6.8</sub>|<sub>14.5</sub>|<sub>18.8</sub>|<sub>[config](./configs/r50_deformable_detr_plus_iterative_bbox_refinement_plus_plus_two_stage.sh)<br/>[log](https://drive.google.com/file/d/1ozi0wbv5-Sc5TbWt1jAuXco72vEfEtbY/view?usp=sharing) <br/>[model](https://drive.google.com/file/d/15I03A7hNTpwuLNdfuEmW9_taZMNVssEp/view?usp=sharing)</sub> |

*Note:*

1. All models of Deformable DETR are trained with total batch size of 32. 
2. Training and inference speed are measured on NVIDIA Tesla V100 GPU.
3. "Deformable DETR (single scale)" means only using res5 feature map (of stride 32) as input feature maps for Deformable Transformer Encoder.
4. "DC5" means removing the stride in C5 stage of ResNet and add a dilation of 2 instead.
5. "DETR-DC5+" indicates DETR-DC5 with some modifications, including using Focal Loss for bounding box classification and increasing number of object queries to 300.
6. "Batch Infer Speed" refer to inference with batch size = 4  to maximize GPU utilization.
7. The original implementation is based on our internal codebase. There are slight differences in the final accuracy and running time due to the plenty details in platform switch.


## Installation

### Requirements

* Linux, CUDA>=9.2, GCC>=5.4
  
* Python>=3.7

    We recommend you to use Anaconda to create a conda environment:
    ```bash
    conda create -n deformable_detr python=3.7 pip
    ```
    Then, activate the environment:
    ```bash
    conda activate deformable_detr
    ```
  
* PyTorch>=1.5.1, torchvision>=0.6.1 (following instructions [here](https://pytorch.org/))

    For example, if your CUDA version is 9.2, you could install pytorch and torchvision as following:
    ```bash
    conda install pytorch=1.5.1 torchvision=0.6.1 cudatoolkit=9.2 -c pytorch
    ```
  
* Other requirements
    ```bash
    pip install -r requirements.txt
    ```

### Compiling CUDA operators
```bash
cd ./models/ops
sh ./make.sh
# unit test (should see all checking is True)
python test.py
```

## Usage

### Dataset preparation

Please download [COCO 2017 dataset](https://cocodataset.org/) and organize them as following:

```
code_root/
└── data/
    └── coco/
        ├── train2017/
        ├── val2017/
        └── annotations/
        	├── instances_train2017.json
        	└── instances_val2017.json
```

### Training

#### Training on single node

For example, the command for training Deformable DETR on 8 GPUs is as following:

```bash
GPUS_PER_NODE=8 ./tools/run_dist_launch.sh 8 ./configs/r50_deformable_detr.sh
```

#### Training on multiple nodes

For example, the command for training Deformable DETR on 2 nodes of each with 8 GPUs is as following:

On node 1:

```bash
MASTER_ADDR=<IP address of node 1> NODE_RANK=0 GPUS_PER_NODE=8 ./tools/run_dist_launch.sh 16 ./configs/r50_deformable_detr.sh
```

On node 2:

```bash
MASTER_ADDR=<IP address of node 1> NODE_RANK=1 GPUS_PER_NODE=8 ./tools/run_dist_launch.sh 16 ./configs/r50_deformable_detr.sh
```

#### Training on slurm cluster

If you are using slurm cluster, you can simply run the following command to train on 1 node with 8 GPUs:

```bash
GPUS_PER_NODE=8 ./tools/run_dist_slurm.sh <partition> deformable_detr 8 configs/r50_deformable_detr.sh
```

Or 2 nodes of  each with 8 GPUs:

```bash
GPUS_PER_NODE=8 ./tools/run_dist_slurm.sh <partition> deformable_detr 16 configs/r50_deformable_detr.sh
```
#### Some tips to speed-up training
* If your file system is slow to read images, you may consider enabling '--cache_mode' option to load whole dataset into memory at the beginning of training.
* You may increase the batch size to maximize the GPU utilization, according to GPU memory of yours, e.g., set '--batch_size 3' or '--batch_size 4'.

### Evaluation

You can get the config file and pretrained model of Deformable DETR (the link is in "Main Results" session), then run following command to evaluate it on COCO 2017 validation set:

```bash
<path to config file> --resume <path to pre-trained model> --eval
```

You can also run distributed evaluation by using ```./tools/run_dist_launch.sh``` or ```./tools/run_dist_slurm.sh```.
