# ContactGraspNet PyTorch Module

This module provides a PyTorch implementation of ContactGraspNet for robotic grasping on dimOS.

## Setup Instructions

### 1. Install Required Dependencies

Install the manipulation extras from the main repository:

```bash
# From the root directory of the dimos repository
pip install -e ".[manipulation]"
```

This will install all the necessary dependencies for using the contact_graspnet_pytorch module, including:
- PyTorch
- Open3D
- Other manipulation-specific dependencies

### 2. Testing the Module

To test that the module is properly installed and functioning:

```bash
# From the root directory of the dimos repository
pytest -s dimos/models/manipulation/contact_graspnet_pytorch/test_contact_graspnet.py
```

The test will verify that:
- The model can be loaded
- Inference runs correctly
- Grasping outputs are generated as expected

### 3. Using in Your Code

Reference ```inference.py``` for usage example.

### Troubleshooting

If you encounter issues with imports or missing dependencies:

1. Verify that the manipulation extras are properly installed:
   ```python
   import contact_graspnet_pytorch
   print("Module loaded successfully!")
   ```

2. If LFS data files are missing, ensure Git LFS is installed and initialized:
   ```bash
   git lfs pull
   ```