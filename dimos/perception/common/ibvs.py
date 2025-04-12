import numpy as np

class PersonDistanceEstimator:
    def __init__(self, K, camera_pitch, camera_height):
        """
        Initialize the distance estimator using ground plane constraint.
        
        Args:
            K: 3x3 Camera intrinsic matrix in OpenCV format
               (Assumed to be already for an undistorted image)
            camera_pitch: Upward pitch of the camera (in radians), in the robot frame
                         Positive means looking up, negative means looking down
            camera_height: Height of the camera above the ground (in meters)
        """
        self.K = K
        self.camera_height = camera_height
        
        # Precompute the inverse intrinsic matrix
        self.K_inv = np.linalg.inv(K)
        
        # Transform from camera to robot frame (z-forward to x-forward)
        self.T = np.array([[0, 0, 1],
                          [-1, 0, 0],
                          [0, -1, 0]])
        
        # Pitch rotation matrix (positive is upward)
        theta = -camera_pitch  # Negative since positive pitch is negative rotation about robot Y
        self.R_pitch = np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [            0, 1,            0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        
        # Combined transform from camera to robot frame
        self.A = self.R_pitch @ self.T
        
        # Store focal length and principal point for angle calculation
        self.fx = K[0, 0]
        self.cx = K[0, 2]

    def estimate_distance_angle(self, bbox: tuple, robot_pitch: float = None):
        """
        Estimate distance and angle to person using ground plane constraint.
        
        Args:
            bbox: tuple (x_min, y_min, x_max, y_max)
                 where y_max represents the feet position
            robot_pitch: Current pitch of the robot body (in radians)
                        If provided, this will be combined with the camera's fixed pitch
        
        Returns:
            depth: distance to person along camera's z-axis (meters)
            angle: horizontal angle in camera frame (radians, positive right)
        """
        x_min, _, x_max, y_max = bbox
        
        # Get center point of feet
        u_c = (x_min + x_max) / 2.0
        v_feet = y_max
        
        # Create homogeneous feet point and get ray direction
        p_feet = np.array([u_c, v_feet, 1.0])
        d_feet_cam = self.K_inv @ p_feet
        
        # If robot_pitch is provided, recalculate the transformation matrix
        if robot_pitch is not None:
            # Combined pitch (fixed camera pitch + current robot pitch)
            total_pitch = -camera_pitch - robot_pitch  # Both negated for correct rotation direction
            R_total_pitch = np.array([
                [ np.cos(total_pitch), 0, np.sin(total_pitch)],
                [                   0, 1,                   0],
                [-np.sin(total_pitch), 0, np.cos(total_pitch)]
            ])
            # Use the updated transformation matrix
            A = R_total_pitch @ self.T
        else:
            # Use the precomputed transformation matrix
            A = self.A
        
        # Convert ray to robot frame using appropriate transformation
        d_feet_robot = A @ d_feet_cam
        
        # Ground plane intersection (z=0)
        # camera_height + t * d_feet_robot[2] = 0
        if abs(d_feet_robot[2]) < 1e-6:
            raise ValueError("Feet ray is parallel to ground plane")
            
        # Solve for scaling factor t
        t = -self.camera_height / d_feet_robot[2]
        
        # Get 3D feet position in robot frame
        p_feet_robot = t * d_feet_robot
        
        # Convert back to camera frame
        p_feet_cam = self.A.T @ p_feet_robot
        
        # Extract depth (z-coordinate in camera frame)
        depth = p_feet_cam[2]
        
        # Calculate horizontal angle from image center
        angle = np.arctan((u_c - self.cx) / self.fx)
        
        return depth, angle


class ObjectDistanceEstimator:

    """
    Estimate distance to an object using the ground plane constraint.
    This class assumes the camera is mounted on a robot and uses the
    camera's intrinsic parameters to estimate the distance to a detected object.
    """
    def __init__(self, K, camera_pitch, camera_height):
        """
        Initialize the distance estimator using ground plane constraint.
        
        Args:
            K: 3x3 Camera intrinsic matrix in OpenCV format
               (Assumed to be already for an undistorted image)
            camera_pitch: Upward pitch of the camera (in radians)
                         Positive means looking up, negative means looking down
            camera_height: Height of the camera above the ground (in meters)
        """
        self.K = K
        self.camera_height = camera_height
        
        # Precompute the inverse intrinsic matrix
        self.K_inv = np.linalg.inv(K)
        
        # Transform from camera to robot frame (z-forward to x-forward)
        self.T = np.array([[0, 0, 1],
                          [-1, 0, 0],
                          [0, -1, 0]])
        
        # Pitch rotation matrix (positive is upward)
        theta = -camera_pitch  # Negative since positive pitch is negative rotation about robot Y
        self.R_pitch = np.array([
            [ np.cos(theta), 0, np.sin(theta)],
            [            0, 1,            0],
            [-np.sin(theta), 0, np.cos(theta)]
        ])
        
        # Combined transform from camera to robot frame
        self.A = self.R_pitch @ self.T
        
        # Store focal length and principal point for angle calculation
        self.fx = K[0, 0]
        self.fy = K[1, 1]
        self.cx = K[0, 2]
        self.estimated_object_size = None

    def estimate_object_size(self, bbox: tuple, distance: float):
        """
        Estimate the physical size of an object based on its bbox and known distance.
        
        Args:
            bbox: tuple (x_min, y_min, x_max, y_max) bounding box in the image
            distance: Known distance to the object (in meters)
            robot_pitch: Current pitch of the robot body (in radians), if any
        
        Returns:
            estimated_size: Estimated physical height of the object (in meters)
        """
        x_min, y_min, x_max, y_max = bbox
        
        # Calculate object height in pixels
        object_height_px = y_max - y_min
        
        # Calculate the physical height using the known distance and focal length
        estimated_size = object_height_px * distance / self.fy
        self.estimated_object_size = estimated_size
        
        return estimated_size
    
    def set_estimated_object_size(self, size: float):
        """
        Set the estimated object size for future distance calculations.
        
        Args:
            size: Estimated physical size of the object (in meters)
        """
        self.estimated_object_size = size

    def estimate_distance_angle(self, bbox: tuple):
        """
        Estimate distance and angle to object using size-based estimation.
        
        Args:
            bbox: tuple (x_min, y_min, x_max, y_max)
                 where y_max represents the bottom of the object
            robot_pitch: Current pitch of the robot body (in radians)
                        If provided, this will be combined with the camera's fixed pitch
            initial_distance: Initial distance estimate for the object (in meters)
                             Used to calibrate object size if not previously known
        
        Returns:
            depth: distance to object along camera's z-axis (meters)
            angle: horizontal angle in camera frame (radians, positive right)
            or None, None if estimation not possible
        """
        # If we don't have estimated object size and no initial distance is provided,
        # we can't estimate the distance
        if self.estimated_object_size is None:
            return None, None
            
        x_min, y_min, x_max, y_max = bbox
        
        # Calculate center of the object for angle calculation
        u_c = (x_min + x_max) / 2.0
        
        # If we have an initial distance estimate and no object size yet,
        # calculate and store the object size using the initial distance
        object_height_px = y_max - y_min
        depth = self.estimated_object_size * self.fy / object_height_px
        
        # Calculate horizontal angle from image center
        angle = np.arctan((u_c - self.cx) / self.fx)
        
        return depth, angle


# Example usage:
if __name__ == "__main__":
    # Example camera calibration
    K = np.array([[600,   0, 320],
                  [  0, 600, 240],
                  [  0,   0,   1]], dtype=np.float32)
    
    # Camera mounted 1.2m high, pitched down 10 degrees
    camera_pitch = np.deg2rad(0)  # negative for downward pitch
    camera_height = 1.0  # meters
    
    estimator = PersonDistanceEstimator(K, camera_pitch, camera_height)
    object_estimator = ObjectDistanceEstimator(K, camera_pitch, camera_height)
    
    # Example detection
    bbox = (300, 100, 380, 400)  # x1, y1, x2, y2
    
    depth, angle = estimator.estimate_distance_angle(bbox)
    # Estimate object size based on the known distance
    object_size = object_estimator.estimate_object_size(bbox, depth)
    depth_obj, angle_obj = object_estimator.estimate_distance_angle(bbox)
    
    print(f"Estimated person depth: {depth:.2f} m")
    print(f"Estimated person angle: {np.rad2deg(angle):.1f}°")
    print(f"Estimated object depth: {depth_obj:.2f} m")
    print(f"Estimated object angle: {np.rad2deg(angle_obj):.1f}°")

    # Shrink the bbox by 30 pixels while keeping the same center
    x_min, y_min, x_max, y_max = bbox
    width = x_max - x_min
    height = y_max - y_min
    center_x = (x_min + x_max) // 2
    center_y = (y_min + y_max) // 2
    
    new_width = max(width - 20, 2)  # Ensure width is at least 2 pixels
    new_height = max(height - 20, 2)  # Ensure height is at least 2 pixels
    
    x_min = center_x - new_width // 2
    x_max = center_x + new_width // 2
    y_min = center_y - new_height // 2
    y_max = center_y + new_height // 2
    
    bbox = (x_min, y_min, x_max, y_max)

    # Re-estimate distance and angle with the new bbox
    depth, angle = estimator.estimate_distance_angle(bbox)
    depth_obj, angle_obj = object_estimator.estimate_distance_angle(bbox)

    print(f"New estimated person depth: {depth:.2f} m")
    print(f"New estimated person angle: {np.rad2deg(angle):.1f}°")
    print(f"New estimated object depth: {depth_obj:.2f} m")
    print(f"New estimated object angle: {np.rad2deg(angle_obj):.1f}°")