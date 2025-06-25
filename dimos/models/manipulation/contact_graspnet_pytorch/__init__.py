"""
ContactGraspNet - A PyTorch implementation of the Contact-GraspNet model.
This package can be imported as `dimos.models.manipulation.contact_graspnet_pytorch`.
"""

import os
import sys
# Add necessary directories to Python path
package_dir = os.path.dirname(os.path.abspath(__file__))
contact_pytorch_dir = os.path.join(package_dir, 'contact_graspnet_pytorch')
if contact_pytorch_dir not in sys.path:
    sys.path.insert(0, contact_pytorch_dir)
if package_dir not in sys.path:
    sys.path.insert(0, package_dir)

# Re-export the core classes for convenient access
from contact_graspnet_pytorch.contact_graspnet import ContactGraspnet
from contact_graspnet_pytorch.contact_grasp_estimator import GraspEstimator
from contact_graspnet_pytorch.inference import inference
from contact_graspnet_pytorch.mesh_utils import PandaGripper, Object, create_gripper

# This makes it possible to access like dimos.models.manipulation.contact_graspnet_pytorch.GraspEstimator
__all__ = [
    'GraspEstimator', 
    'inference', 
    'ContactGraspnet', 
    'PandaGripper', 
    'Object', 
    'create_gripper'
]
