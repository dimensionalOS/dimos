from typing import Optional, Union, List, Dict
from abc import ABC, abstractmethod
from pathlib import Path

class SimulatorBase(ABC):
    """Base class for simulators."""
    
    @abstractmethod
    def __init__(
        self, 
        headless: bool = True,
        open_usd: Optional[str] = None,  # Keep for Isaac compatibility
        entities: Optional[List[Dict[str, Union[str, dict]]]] = None  # Add for Genesis
    ):
        """Initialize the simulator.
        
        Args:
            headless: Whether to run without visualization
            open_usd: Path to USD file (for Isaac)
            entities: List of entity configurations (for Genesis)
        """
        self.headless = headless
        self.open_usd = open_usd
        self.stage = None
        
    @abstractmethod
    def get_stage(self):
        """Get the current stage/scene."""
        pass
    
    @abstractmethod
    def close(self):
        """Close the simulation."""
        pass 