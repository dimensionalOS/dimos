# Detic model zoo

## Introduction

This file documents a collection of models reported in our paper.
The training time was measured on [Big Basin](https://engineering.fb.com/data-center-engineering/introducing-big-basin-our-next-generation-ai-hardware/)
servers with 8 NVIDIA V100 GPUs & NVLink.

#### How to Read the Tables

The "Name" column contains a link to the config file. 
To train a model, run 

```
python train_net.py --num-gpus 8 --config-file /path/to/config/name.yaml
``` 

To evaluate a model with a trained/ pretrained model, run 

```
python train_net.py --num-gpus 8 --config-file /path/to/config/name.yaml --eval-only MODEL.WEIGHTS /path/to/weight.pth
``` 

#### Third-party ImageNet-21K Pretrained Models

Our paper uses ImageNet-21K pretrained models that are not part of Detectron2 (ResNet-50-21K from [MIIL](https://github.com/Alibaba-MIIL/ImageNet21K) and SwinB-21K from [Swin-Transformer](https://github.com/microsoft/Swin-Transformer)). Before training, 
please download the models and place them under `DETIC_ROOT/models/`, and following [this tool](../tools/convert-thirdparty-pretrained-model-to-d2.py) to convert the format.


## Open-vocabulary LVIS

|         Name          |Training time |  mask mAP | mask mAP_novel  | Download |
|-----------------------|------------------|-----------|-----------------|----------|
|[Box-Supervised_C2_R50_640_4x](../configs/BoxSup-C2_Lbase_CLIP_R5021k_640b64_4x.yaml)     | 17h | 30.2      |       16.4      | [model](https://dl.fbaipublicfiles.com/detic/BoxSup-C2_Lbase_CLIP_R5021k_640b64_4x.pth) |
|[Detic_C2_IN-L_R50_640_4x](../configs/Detic_LbaseI_CLIP_R5021k_640b64_4x_ft4x_max-size.yaml) | 22h | 32.4      |       24.9      | [model](https://dl.fbaipublicfiles.com/detic/Detic_LbaseI_CLIP_R5021k_640b64_4x_ft4x_max-size.pth) |
|[Detic_C2_CCimg_R50_640_4x](../configs/Detic_LbaseCCimg_CLIP_R5021k_640b64_4x_ft4x_max-size.yaml) | 22h | 31.0      |       19.8      | [model](https://dl.fbaipublicfiles.com/detic/Detic_LbaseCCimg_CLIP_R5021k_640b64_4x_ft4x_max-size.pth) |
|[Detic_C2_CCcapimg_R50_640_4x](../configs/Detic_LbaseCCcapimg_CLIP_R5021k_640b64_4x_ft4x_max-size.yaml) | 22h | 31.0      |       21.3      | [model](https://dl.fbaipublicfiles.com/detic/Detic_LbaseCCcapimg_CLIP_R5021k_640b64_4x_ft4x_max-size.pth) |
|[Box-Supervised_C2_SwinB_896_4x](../configs/BoxSup-C2_Lbase_CLIP_SwinB_896b32_4x.yaml)     | 43h | 38.4      |       21.9      | [model](https://dl.fbaipublicfiles.com/detic/BoxSup-C2_Lbase_CLIP_SwinB_896b32_4x.pth) |
|[Detic_C2_IN-L_SwinB_896_4x](../configs/Detic_LbaseI_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml) | 47h | 40.7      |       33.8      | [model](https://dl.fbaipublicfiles.com/detic/Detic_LbaseI_CLIP_SwinB_896b32_4x_ft4x_max-size.pth) |


#### Note

- The open-vocabulary LVIS setup is LVIS without rare class annotations in training. We evaluate rare classes as novel classes in testing.

- The models with `C2` are trained using our improved LVIS baseline (Appendix D of the paper), including CenterNet2 detector, Federated Loss, large-scale jittering, etc.

- All models use [CLIP](https://github.com/openai/CLIP) embeddings as classifiers. This makes the box-supervised models have non-zero mAP on novel classes.

- The models with `IN-L` use the overlap classes between ImageNet-21K and LVIS as image-labeled data.

-  The models with `CC` use Conception Captions. `CCimg` uses image labels extracted from the captions (using a naive text-match) as image-labeled data. `CCcapimg` additionally uses the row captions (Appendix C of the paper).

- The Detic models are finetuned on the corresponding Box-Supervised models above (indicated by MODEL.WEIGHTS in the config files). Please train or download the Box-Supervised model and place them under `DETIC_ROOT/models/` before training the Detic models.


## Standard LVIS

|         Name          |Training time |  mask mAP | mask mAP_rare  | Download |
|-----------------------|------------------|-----------|-----------------|----------|
|[Box-Supervised_C2_R50_640_4x](../configs/BoxSup-C2_L_CLIP_R5021k_640b64_4x.yaml)     | 17h | 31.5      |       25.6      | [model](https://dl.fbaipublicfiles.com/detic/BoxSup-C2_L_CLIP_R5021k_640b64_4x.pth) |
|[Detic_C2_R50_640_4x](../configs/Detic_LI_CLIP_R5021k_640b64_4x_ft4x_max-size.yaml) | 22h | 33.2      |       29.7      | [model](https://dl.fbaipublicfiles.com/detic/Detic_LI_CLIP_R5021k_640b64_4x_ft4x_max-size.pth) |
|[Box-Supervised_C2_SwinB_896_4x](../configs/BoxSup-C2_L_CLIP_SwinB_896b32_4x.yaml)     | 43h | 40.7      |       35.9      | [model](https://dl.fbaipublicfiles.com/detic/BoxSup-C2_L_CLIP_SwinB_896b32_4x.pth) |
|[Detic_C2_SwinB_896_4x](../configs/Detic_LI_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml) | 47h | 41.7      |       41.7      | [model](https://dl.fbaipublicfiles.com/detic/Detic_LI_CLIP_SwinB_896b32_4x_ft4x_max-size.pth) |


|         Name          |Training time |  box mAP | box mAP_rare  | Download |
|-----------------------|------------------|-----------|-----------------|----------|
|[Box-Supervised_DeformDETR_R50_4x](../configs/BoxSup-DeformDETR_L_R50_4x.yaml)   |  31h | 31.7 |  21.4     |  [model](https://dl.fbaipublicfiles.com/detic/BoxSup-DeformDETR_L_R50_4x.pth) |
|[Detic_DeformDETR_R50_4x](../configs/Detic_DeformDETR_LI_R50_4x_ft4x.yaml) | 47h | 32.5  | 26.2  | [model](https://dl.fbaipublicfiles.com/detic/Detic_DeformDETR_LI_R50_4x_ft4x.pth) |


#### Note

- All Detic models use the overlap classes between ImageNet-21K and LVIS as image-labeled data;

- The models with `C2` are trained using our improved LVIS baseline in the paper, including CenterNet2 detector, Federated loss, large-scale jittering, etc.

- The models with `DeformDETR` are Deformable DETR models. We train the models with Federated Loss.

## Open-vocabulary COCO

|         Name          |Training time |  box mAP50 | box mAP50_novel | Download |
|-----------------------|------------------|-----------|-----------------|----------|
|[BoxSup_CLIP_R50_1x](../configs/BoxSup_OVCOCO_CLIP_R50_1x.yaml)     | 12h | 39.3      |   1.3  | [model](https://dl.fbaipublicfiles.com/detic/BoxSup_OVCOCO_CLIP_R50_1x.pth) |
|[Detic_CLIP_R50_1x_image](../configs/Detic_OVCOCO_CLIP_R50_1x_max-size.yaml)     |  13h | 44.7      |   24.1  | [model](https://dl.fbaipublicfiles.com/detic/Detic_OVCOCO_CLIP_R50_1x_max-size.pth) |
|[Detic_CLIP_R50_1x_caption](../configs/Detic_OVCOCO_CLIP_R50_1x_caption.yaml)     | 16h | 43.8      |   21.0  | [model](https://dl.fbaipublicfiles.com/detic/Detic_OVCOCO_CLIP_R50_1x_caption.pth) |
|[Detic_CLIP_R50_1x_caption-image](../configs/Detic_OVCOCO_CLIP_R50_1x_max-size_caption.yaml)     | 16h | 45.0      |   27.8 | [model](https://dl.fbaipublicfiles.com/detic/Detic_OVCOCO_CLIP_R50_1x_max-size_caption.pth) |

#### Note

- All models are trained with ResNet50-C4 without multi-scale augmentation. All models use CLIP embeddings as the classifier.

- We extract class names from COCO-captions as image-labels. `Detic_CLIP_R50_1x_image` uses the max-size loss; `Detic_CLIP_R50_1x_caption` directly uses CLIP caption embedding within each mini-batch for classification; `Detic_CLIP_R50_1x_caption-image` uses both losses.

- We report box mAP50 under the "generalized" open-vocabulary setting.


## Cross-dataset evaluation 


|         Name          |Training time |  Objects365 box mAP  | OpenImages box mAP50   | Download |
|-----------------------|------------------|-----------|-----------------|----------|
|[Box-Supervised_C2_SwinB_896_4x](../configs/BoxSup-C2_L_CLIP_SwinB_896b32_4x.yaml)     | 43h |  19.1  |  46.2    | [model](https://dl.fbaipublicfiles.com/detic/BoxSup-C2_L_CLIP_SwinB_896b32_4x.pth) |
|[Detic_C2_SwinB_896_4x](../configs/Detic_LI_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml) | 47h |   21.2  |53.0        | [model](https://dl.fbaipublicfiles.com/detic/Detic_LI_CLIP_SwinB_896b32_4x_ft4x_max-size.pth) |
|[Detic_C2_SwinB_896_4x_IN-21K](../configs/Detic_LI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml) | 47h | 21.4    |   55.2     | [model](https://dl.fbaipublicfiles.com/detic/Detic_LI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth) |
|[Box-Supervised_C2_SwinB_896_4x+COCO](../configs/BoxSup-C2_LCOCO_CLIP_SwinB_896b32_4x.yaml)     | 43h |  19.7  |   46.4   | [model](https://dl.fbaipublicfiles.com/detic/BoxSup-C2_LCOCO_CLIP_SwinB_896b32_4x.pth) |
|[Detic_C2_SwinB_896_4x_IN-21K+COCO](../configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml) | 47h |  21.6   |    54.6    | [model](https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth) |



#### Note

- `Box-Supervised_C2_SwinB_896_4x` and `Detic_C2_SwinB_896_4x` are the same model in the [Standard LVIS](#standard-lvis) section, but evaluated with Objects365/ OpenImages vocabulary (i.e. CLIP embeddings of the corresponding class names as classifier). To run the evaluation on Objects365/ OpenImages, run 

  ```
  python train_net.py --num-gpus 8 --config-file configs/Detic_C2_SwinB_896_4x.yaml --eval-only DATASETS.TEST "('oid_val_expanded','objects365_v2_val',)" MODEL.RESET_CLS_TESTS True MODEL.TEST_CLASSIFIERS "('datasets/metadata/oid_clip_a+cname.npy','datasets/metadata/o365_clip_a+cnamefix.npy',)" MODEL.TEST_NUM_CLASSES "(500,365)" MODEL.MASK_ON False
  ```

- `Detic_C2_SwinB_896_4x_IN-21K` trains on the full ImageNet-22K. We additionally use a dynamic class sampling ("Modified Federated Loss" in Section 4.4) and use a larger data sampling ratio of ImageNet images (1:16 instead of 1:4).

- `Detic_C2_SwinB_896_4x_IN-21K-COCO` is a model trained on combined LVIS-COCO and ImageNet-21K for better demo purposes. LVIS models do not detect persons well due to its federated annotation protocol. LVIS+COCO models give better visual results.


## Real-time models

|         Name          | Run time (ms) |  LVIS box mAP  | Download |
|-----------------------|------------------|-----------|-----------------|
|[Detic_C2_SwinB_896_4x_IN-21K+COCO (800x1333, no threshold)](../configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml) | 115 |  44.4   | [model](https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth) |
|[Detic_C2_SwinB_896_4x_IN-21K+COCO](../configs/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.yaml) | 46 |   35.0   | [model](https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_SwinB_896b32_4x_ft4x_max-size.pth) |
|[Detic_C2_ConvNeXtT_896_4x_IN-21K+COCO](../configs/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.yaml) | 26 |  30.7   | [model](https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_CXT21k_640b32_4x_ft4x_max-size.pth) |
|[Detic_C2_R5021k_896_4x_IN-21K+COCO](../configs/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.yaml) | 23 |  29.0  | [model](https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.pth) |
|[Detic_C2_R18_896_4x_IN-21K+COCO](../configs/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.yaml) | 18 |  22.1  | [model](https://dl.fbaipublicfiles.com/detic/Detic_LCOCOI21k_CLIP_R18_640b32_4x_ft4x_max-size.pth) |

- `Detic_C2_SwinB_896_4x_IN-21K+COCO (800x1333, thresh 0.02)` is the entry on the [Cross-dataset evaluation](#Cross-dataset evaluation) section without the mask head. All other entries use a max-size of 640 and an output score threshold of 0.3 using the following command (e.g., with R50).

  ```
  python train_net.py --config-file configs/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.yaml --num-gpus 2 --eval-only DATASETS.TEST "('lvis_v1_val',)" MODEL.RESET_CLS_TESTS True MODEL.TEST_CLASSIFIERS "('datasets/metadata/lvis_v1_clip_a+cname.npy',)" MODEL.TEST_NUM_CLASSES "(1203,)" MODEL.MASK_ON False MODEL.WEIGHTS models/Detic_LCOCOI21k_CLIP_R5021k_640b32_4x_ft4x_max-size.pth INPUT.MIN_SIZE_TEST 640 INPUT.MAX_SIZE_TEST 640 MODEL.ROI_HEADS.SCORE_THRESH_TEST 0.3
  ```

- All models are trained using the same training recipe except for different backbones.
- The ConvNeXtT and Res50 models are initialized from their corresponding ImageNet-21K pretrained models. The Res18 model is initialized from its ImageNet-1K pretrained model.
- The runtimes are measured on a local workstation with a Titan RTX GPU.
