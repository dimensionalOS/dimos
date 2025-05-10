import math
import numpy as np
from typing import Tuple
from scipy.spatial.transform import Rotation as R
import logging

logger = logging.getLogger(__name__)

def normalize_angle(angle: float) -> float:
    """Normalize angle to [-pi, pi] range"""
    return np.arctan2(np.sin(angle), np.cos(angle))

def distance_angle_to_goal_xy(distance: float, angle: float) -> Tuple[float, float]:
    """Convert distance and angle to goal x, y in robot frame"""
    return distance * np.cos(angle), distance * np.sin(angle)

