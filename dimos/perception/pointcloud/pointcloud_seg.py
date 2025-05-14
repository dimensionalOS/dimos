import numpy as np
import cv2
import yaml
import os
import sys
from PIL import Image, ImageDraw
from dimos.perception.segmentation import Sam2DSegmenter
from dimos.perception.pointcloud.utils import (
    load_camera_matrix_from_yaml,
    create_masked_point_cloud,
    o3d_point_cloud_to_numpy,
    rotation_to_o3d
)
from dimos.perception.pointcloud.cuboid_fit import fit_cuboid, visualize_fit
import torch
import open3d as o3d

class PointcloudSegmentation:
    def __init__(
        self,
        model_path="FastSAM-s.pt",
        device="cuda",
        color_intrinsics=None,
        depth_intrinsics=None,
        enable_tracking=True,
        enable_analysis=True,
    ):
        """
        Initialize processor to segment objects in RGB images and extract their point clouds.
        
        Args:
            model_path: Path to the FastSAM model
            device: Computation device ("cuda" or "cpu")
            color_intrinsics: Path to YAML file or list with color camera intrinsics [fx, fy, cx, cy]
            depth_intrinsics: Path to YAML file or list with depth camera intrinsics [fx, fy, cx, cy]
            enable_tracking: Whether to enable object tracking
            enable_analysis: Whether to enable object analysis (labels, etc.)
            min_analysis_interval: Minimum interval between analysis runs in seconds
        """
        # Initialize segmenter
        self.segmenter = Sam2DSegmenter(
            model_path=model_path,
            device=device,
            use_tracker=enable_tracking,
            use_analyzer=enable_analysis,
        )
        
        # Store settings
        self.enable_tracking = enable_tracking
        self.enable_analysis = enable_analysis
        
        # Load camera matrices
        self.color_camera_matrix = load_camera_matrix_from_yaml(color_intrinsics)
        self.depth_camera_matrix = load_camera_matrix_from_yaml(depth_intrinsics)
    
    def generate_color_from_id(self, track_id):
        """Generate a consistent color for a given tracking ID."""
        np.random.seed(track_id)
        color = np.random.randint(0, 255, 3)
        np.random.seed(None)
        return color
    
    def process_images(self, color_img, depth_img, fit_3d_cuboids=True):
        """
        Process color and depth images to segment objects and extract point clouds.
        Uses Open3D for point cloud processing.
        
        Args:
            color_img: RGB image as numpy array (H, W, 3)
            depth_img: Depth image as numpy array (H, W) in meters
            fit_3d_cuboids: Whether to fit 3D cuboids to each object
        
        Returns:
            dict: Dictionary containing:
                - viz_image: Visualization image with detections
                - objects: List of dicts for each object with:
                    - mask: Segmentation mask (H, W, bool)
                    - bbox: Bounding box [x1, y1, x2, y2]
                    - target_id: Tracking ID
                    - confidence: Detection confidence
                    - name: Object name (if analyzer enabled)
                    - point_cloud: Open3D point cloud object
                    - point_cloud_numpy: Nx6 array of XYZRGB points (for compatibility)
                    - color: RGB color for visualization
                    - cuboid_params: Cuboid parameters (if fit_3d_cuboids=True)
        """
        if self.depth_camera_matrix is None:
            raise ValueError("Depth camera matrix must be provided to process images")
        
        # Run segmentation
        masks, bboxes, target_ids, probs, names = self.segmenter.process_image(color_img)
        print(f"Found {len(masks)} segmentation masks")
        
        # Run analysis if enabled
        if self.enable_analysis:
            self.segmenter.run_analysis(color_img, bboxes, target_ids)
            names = self.segmenter.get_object_names(target_ids, names)
        
        # Create visualization image
        viz_img = self.segmenter.visualize_results(
            color_img.copy(),
            masks,
            bboxes,
            target_ids,
            probs,
            names
        )
        
        # Process each object
        objects = []
        for i, (mask, bbox, target_id, prob, name) in enumerate(zip(masks, bboxes, target_ids, probs, names)):
            # Convert mask to numpy if it's a tensor
            if hasattr(mask, 'cpu'):
                mask = mask.cpu().numpy()
                
            # Ensure mask is proper boolean array with correct dimensions
            mask = mask.astype(bool)
            
            # Ensure mask has the same shape as the depth image
            if mask.shape != depth_img.shape[:2]:
                print(f"Warning: Mask shape {mask.shape} doesn't match depth image shape {depth_img.shape[:2]}")
                if len(mask.shape) > 2:
                    # If mask has extra dimensions, take the first channel
                    mask = mask[:,:,0] if mask.shape[2] > 0 else mask[:,:,0]
                
                # If shapes still don't match, try to resize the mask
                if mask.shape != depth_img.shape[:2]:
                    mask = cv2.resize(mask.astype(np.uint8), 
                                      (depth_img.shape[1], depth_img.shape[0]), 
                                      interpolation=cv2.INTER_NEAREST).astype(bool)
            
            try:
                # Create point cloud using Open3D
                pcd = create_masked_point_cloud(
                    color_img,
                    depth_img,
                    mask,
                    self.depth_camera_matrix,
                    depth_scale=1.0  # Assuming depth is already in meters
                )
                
                # Skip if no points
                if len(np.asarray(pcd.points)) == 0:
                    print(f"Skipping object {i+1}: No points in point cloud")
                    continue
                
                # Generate color for visualization
                rgb_color = self.generate_color_from_id(target_id)
                
                # Create object data
                obj_data = {
                    "mask": mask,
                    "bbox": bbox,
                    "target_id": target_id,
                    "confidence": float(prob),
                    "name": name if name else "",
                    "point_cloud": pcd,
                    "point_cloud_numpy": o3d_point_cloud_to_numpy(pcd),
                    "color": rgb_color
                }
                
                # Fit 3D cuboid if requested
                if fit_3d_cuboids:
                    points = np.asarray(pcd.points)
                    cuboid_params = fit_cuboid(points)
                    obj_data["cuboid_params"] = cuboid_params
                    
                    # Update visualization with cuboid if available
                    if cuboid_params is not None and self.color_camera_matrix is not None:
                        viz_img = visualize_fit(viz_img, cuboid_params, self.color_camera_matrix)
                
                objects.append(obj_data)
                
            except Exception as e:
                print(f"Error processing object {i+1}: {e}")
                continue
        
        # Clean up GPU memory if using CUDA
        if torch.cuda.is_available():
            torch.cuda.empty_cache()
        
        return {
            "viz_image": viz_img,
            "objects": objects
        }
    
    def cleanup(self):
        """Clean up resources."""
        if hasattr(self, 'segmenter'):
            self.segmenter.cleanup()

def main():
    """
    Main function to test the PointcloudSegmentation class with data from rgbd_data folder.
    """

    def find_first_image(directory):
        """Find the first image file in the given directory."""
        image_extensions = ['.jpg', '.jpeg', '.png', '.bmp']
        for filename in sorted(os.listdir(directory)):
            if any(filename.lower().endswith(ext) for ext in image_extensions):
                return os.path.join(directory, filename)
        return None

    # Define paths
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dimos_dir = os.path.abspath(os.path.join(script_dir, "../../../"))
    data_dir = os.path.join(dimos_dir, "assets/rgbd_data")
    
    color_info_path = os.path.join(data_dir, "color_camera_info.yaml")
    depth_info_path = os.path.join(data_dir, "depth_camera_info.yaml")
    
    color_dir = os.path.join(data_dir, "color")
    depth_dir = os.path.join(data_dir, "depth")
    
    # Find first color and depth images
    color_img_path = find_first_image(color_dir)
    depth_img_path = find_first_image(depth_dir)
    
    if not color_img_path or not depth_img_path:
        print(f"Error: Could not find color or depth images in {data_dir}")
        return
    
    print(f"Found color image: {color_img_path}")
    print(f"Found depth image: {depth_img_path}")
    
    # Load images
    color_img = cv2.imread(color_img_path)
    if color_img is None:
        print(f"Error: Could not load color image from {color_img_path}")
        return
        
    color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)  # Convert to RGB
    
    depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)
    if depth_img is None:
        print(f"Error: Could not load depth image from {depth_img_path}")
        return
    
    # Convert depth to meters if needed (adjust scale as needed for your data)
    if depth_img.dtype == np.uint16:
        # Convert from mm to meters for typical depth cameras
        depth_img = depth_img.astype(np.float32) / 1000.0
    
    # Verify image shapes for debugging
    print(f"Color image shape: {color_img.shape}")
    print(f"Depth image shape: {depth_img.shape}")
    
    # Initialize segmentation with direct camera matrices
    seg = PointcloudSegmentation(
        model_path="FastSAM-s.pt",  # Adjust path as needed
        device="cuda" if torch.cuda.is_available() else "cpu",
        color_intrinsics=color_info_path,
        depth_intrinsics=depth_info_path,
        enable_tracking=False,
        enable_analysis=True
    )
    
    # Process images
    print("Processing images...")
    try:
        results = seg.process_images(color_img, depth_img, fit_3d_cuboids=True)
        
        # Show segmentation results using PIL instead of OpenCV
        viz_img = results["viz_image"]
        
        # Convert OpenCV image (BGR) to PIL image (RGB)
        pil_img = Image.fromarray(cv2.cvtColor(viz_img, cv2.COLOR_BGR2RGB))
        
        # Display the image using PIL
        pil_img.show(title="Segmentation Results")
        
        # Add a short pause to ensure the image has time to display
        import time
        time.sleep(0.5)
        
        print(f"Found {len(results['objects'])} objects with valid point clouds")
        
        # Visualize all point clouds in a single window
        all_pcds = []
        for i, obj in enumerate(results['objects']):
            pcd = obj['point_cloud']
            
            # Optionally add axis-aligned bounding box visualization
            if 'cuboid_params' in obj and obj['cuboid_params'] is not None:
                cuboid = obj['cuboid_params']
                
                # Create oriented bounding box using the rotation matrix instead of axis-aligned box
                center = cuboid['center']
                dimensions = cuboid['dimensions']
                rotation = rotation_to_o3d(cuboid['rotation'])
                
                # Create oriented bounding box
                obb = o3d.geometry.OrientedBoundingBox(
                    center=center,
                    R=rotation,
                    extent=dimensions
                )
                obb.color = [1, 0, 0]  # Red bounding box
                all_pcds.append(obb)
                
                # Add a small coordinate frame at the center of each object to show orientation
                coord_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(
                    size=min(dimensions) * 0.5,
                    origin=center
                )
                all_pcds.append(coord_frame)
            
            # Add the point cloud
            all_pcds.append(pcd)
        
        # Add coordinate frame at origin
        coordinate_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=0.1)
        all_pcds.append(coordinate_frame)
        
        # Show point clouds
        if all_pcds:
            o3d.visualization.draw_geometries(all_pcds,
                                             window_name="Segmented Objects",
                                             width=1280,
                                             height=720,
                                             left=50,
                                             top=50)
        else:
            print("No objects with valid point clouds found.")
    
    except Exception as e:
        print(f"Error during processing: {str(e)}")
        import traceback
        traceback.print_exc()
    
    # Clean up resources
    seg.cleanup()
    print("Done!")


if __name__ == "__main__":
    main()
