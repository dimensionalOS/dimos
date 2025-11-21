import numpy as np

def calculate_transformation(camera_points, real_points):
    """Calculate transformation from camera to robot coordinates"""
    
    camera_points = np.array(camera_points)
    real_points = np.array(real_points)
    
    # Step 1: Find centroids
    camera_centroid = np.mean(camera_points, axis=0)
    real_centroid = np.mean(real_points, axis=0)
    
    print(f"Camera centroid: {camera_centroid}")
    print(f"Real centroid: {real_centroid}")
    
    # Step 2: Center the points
    camera_centered = camera_points - camera_centroid
    real_centered = real_points - real_centroid
    
    # Step 3: Compute cross-covariance matrix
    H = camera_centered.T @ real_centered
    
    # Step 4: SVD to find rotation
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    
    # Ensure proper rotation (det = 1)
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T
    
    # Step 5: Find translation
    t = real_centroid - R @ camera_centroid
    
    # Build 4x4 transformation matrix
    transform = np.eye(4)
    transform[:3, :3] = R
    transform[:3, 3] = t
    
    return transform, R, t

# Your points
camera_points = [
    [0.06, -0.0818, 0.918],
    [0.16479, -0.087483, 0.898],
    [0.2171, -0.10613, 0.93699],
    [0.024725, -0.1304658, 0.999],
    [-0.0169, -0.136898, 0.9943],
    [-0.16, -0.0529, 0.849636],
    [0.188313, -0.128734, 0.95622]
]
'''camera_points = [
    [0.06, -0.0818, 0.918],
    [-0.16, -0.0529, 0.849636],
    [0.188313, -0.128734, 0.95622]
]'''
real_points = [
    [0.540, -0.26, 0.095],
    [0.34, -0.36, 0.095],
    [0.68, -0.15, 0.130],
    [0.64, -0.32, 0.22],
    [0.465, -0.23, 0.105],
    [0.48, -0.44, 0.12],
    [0.59, -0.11, 0.11]
]
'''real_points = [
    [0.540, -0.26, 0.095],
    [0.48, -0.44, 0.12],
    [0.59, -0.11, 0.11]
]'''
# Calculate transformation
transform, R, t = calculate_transformation(camera_points, real_points)
transform = np.array([
    [ 0.8549843,   0.37402971, -0.35930993,  0.73029989],
    [ 0.50525877, -0.44422239,  0.73985136, -0.94619703],
    [ 0.11711288, -0.81410579, -0.56878496,  0.54634202],
    [ 0.0,         0.0,         0.0,         1.0       ]
])

print("\n=== TRANSFORMATION MATRIX ===")
print(transform)

# Verify the transformation
print("\n=== VERIFICATION ===")
for i, (cam_pt, real_pt) in enumerate(zip(camera_points, real_points)):
    cam_homo = np.append(cam_pt, 1)
    predicted = transform @ cam_homo
    predicted = predicted[:3]
    error = np.linalg.norm(predicted - real_pt)
    print(f"Point {i+1}:")
    print(f"  Camera: {cam_pt}")
    print(f"  Real: {real_pt}")
    print(f"  Predicted: {predicted}")
    print(f"  Error: {error*1000:.1f}mm")

# Calculate average error
def calculate_rmse(transform, camera_points, real_points):
    errors = []
    for cam_pt, real_pt in zip(camera_points, real_points):
        cam_homo = np.append(cam_pt, 1)
        predicted = (transform @ cam_homo)[:3]
        error = np.linalg.norm(predicted - real_pt)
        errors.append(error)
    return np.sqrt(np.mean(np.square(errors)))

rmse = calculate_rmse(transform, camera_points, real_points)
print(f"\nRMS Error: {rmse*1000:.1f}mm")