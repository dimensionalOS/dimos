# Use deterministic kinematic conversion for SONIC retargeting

dimOS will derive the native-equivalent SONIC pose from WebXR joint positions and orientations through fixed joint mappings, coordinate-basis changes, parent-relative rotations, and bounded wrist decomposition. The live path will not fit an SMPL body model with an iterative optimizer: WebXR already supplies an oriented skeleton, and deterministic conversion provides predictable latency, explicit failure behavior, and fixtures that can pin every transform at SONIC's input rate.

## Consequences

The retargeter must own and test the WebXR-to-SMPL rest-basis table. Operator body-shape estimation and runtime optimization are outside the first simulator milestone.
