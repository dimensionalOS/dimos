## Changelog

**[2020.12.07]** Fix a bug of sampling offset normalization (see [this issue](https://github.com/fundamentalvision/Deformable-DETR/issues/6)) in the MSDeformAttn module. The final accuracy on COCO is slightly improved. Code and pre-trained models have been updated. This bug only occurs in this released version but not in the original implementation used in our paper.