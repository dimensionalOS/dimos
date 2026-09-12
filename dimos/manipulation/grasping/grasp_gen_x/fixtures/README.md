# Recorded GraspGenX object cloud

`object_cloud.npy` contains the 2,624 XYZ points from the `pc` field of
`assets/sample_data/object_pc/1741385877_42353.json` in NVlabs/GraspGenX,
revision `b9429097728cb1c430dd78b92edf17ba318aad03`.

Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. Apache-2.0; see LICENSE.
Modified for this fixture: extracted XYZ only and converted to float32 NumPy format.
The input is a segmented object cloud in metres. Historical model outputs and
colors were omitted. Inference is stochastic; tests validate candidate invariants
rather than comparing against stored predictions.
