from abc import ABC, abstractmethod
import numpy as np

class Environment(ABC):
    def __init__(self):
        self.environment_type = None
        self.graph = None

    @abstractmethod
    def label_objects(self) -> list[str]:
        """
        Label all objects in the environment.
        
        Returns:
            A list of string labels representing the objects in the environment.
        """
        pass

    @abstractmethod
    def get_visualization(self, format_type):
        """Return different visualization formats like images, NERFs, or other 3D file types."""
        pass
    
    @abstractmethod
    def generate_segmentations(self, model: str = None, objects: list[str] = None, *args, **kwargs) -> list[np.ndarray]:
        """
        Generate object segmentations of objects[] using neural methods.

        Args:
            model (str, optional): The string of the desired segmentation model (SegmentAnything, etc.)
            objects (list[str], optional): The list of strings of the specific objects to segment.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            list of numpy.ndarray: A list where each element is a numpy array
            representing a binary mask for a segmented area of an object in the environment.

        Note:
            The specific arguments and their usage will depend on the subclass implementation.
        """
        pass

    @abstractmethod
    def get_segmentations(self) -> list[np.ndarray]:
        """
        Get segmentations using a method like 'segment anything'.

        Returns:
            list of numpy.ndarray: A list where each element is a numpy array
            representing a binary mask for a segmented area of an object in the environment.
        """
        pass


    @abstractmethod
    def generate_point_cloud(self, object: str = None, *args, **kwargs) -> np.ndarray:
        """
        Generate a point cloud for the entire environment or a specific object.

        Args:
            object (str, optional): The string of the specific object to get the point cloud for.
                                    If None, returns the point cloud for the entire environment.
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            np.ndarray: A numpy array representing the generated point cloud.
                        Shape: (n, 3) where n is the number of points and each point is [x, y, z].

        Note:
            The specific arguments and their usage will depend on the subclass implementation.
        """
        pass

    @abstractmethod
    def get_point_cloud(self, object: str = None) -> np.ndarray:
        """
        Return point clouds of the entire environment or a specific object.

        Args:
            object (str, optional): The string of the specific object to get the point cloud for. If None, returns the point cloud for the entire environment.

        Returns:
            np.ndarray: A numpy array representing the point cloud.
                        Shape: (n, 3) where n is the number of points and each point is [x, y, z].
        """
        pass

    @abstractmethod
    def generate_depth_map(self, stereo: bool = None, monocular: bool = None, model: str = None, *args, **kwargs) -> np.ndarray:
        """
        Generate a depth map using monocular or stereo camera methods.

        Args:
            stereo (bool, optional): Whether to stereo camera is avaliable for ground truth depth map generation.
            monocular (bool, optional): Whether to use monocular camera for neural depth map generation.
            model (str, optional): The string of the desired monocular depth model (Metric3D, ZoeDepth, etc.)
            *args: Variable length argument list.
            **kwargs: Arbitrary keyword arguments.

        Returns:
            np.ndarray: A 2D numpy array representing the generated depth map.
                        Shape: (height, width) where each value represents the depth
                        at that pixel location.

        Note:
            The specific arguments and their usage will depend on the subclass implementation.
        """
        pass

    @abstractmethod
    def get_depth_map(self) -> np.ndarray:
        """
        Return a depth map of the environment.

        Returns:
            np.ndarray: A 2D numpy array representing the depth map.
                        Shape: (height, width) where each value represents the depth
                        at that pixel location. Typically, closer objects have smaller
                        values and farther objects have larger values.

        Note:
            The exact range and units of the depth values may vary depending on the
            specific implementation and the sensor or method used to generate the depth map.
        """
        pass

    def initialize_from_images(self, images):
        """Initialize the environment from a set of image frames or video."""
        raise NotImplementedError("This method is not implemented for this environment type.")

    def initialize_from_file(self, file_path):
        """Initialize the environment from a spatial file type.

        Supported file types include:
        - GLTF/GLB (GL Transmission Format)
        - FBX (Filmbox)
        - OBJ (Wavefront Object)
        - USD/USDA/USDC (Universal Scene Description)
        - STL (Stereolithography)
        - COLLADA (DAE)
        - Alembic (ABC)
        - PLY (Polygon File Format)
        - 3DS (3D Studio)
        - VRML/X3D (Virtual Reality Modeling Language)

        Args:
            file_path (str): Path to the spatial file.

        Raises:
            NotImplementedError: If the method is not implemented for this environment type.
        """
        raise NotImplementedError("This method is not implemented for this environment type.")


