from abc import ABC, abstractmethod

class AbstractSensor(ABC):
    def __init__(self, sensor_type=None):
        self.sensor_type = sensor_type

    @abstractmethod
    def get_sensor_type(self):
        """Return the type of sensor."""
        pass

    @abstractmethod
    def calculate_intrinsics(self):
        """Calculate the sensor's intrinsics."""
        pass

    @abstractmethod
    def get_intrinsics(self):
        """Return the sensor's intrinsics."""
        pass
