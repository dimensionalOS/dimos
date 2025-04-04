from typing import Optional, List, Dict, Union
from isaacsim import SimulationApp
from ..base.simulator_base import SimulatorBase

class IsaacSimulator(SimulatorBase):
    """Isaac Sim simulator implementation."""
    
    def __init__(
        self, 
        headless: bool = True, 
        open_usd: Optional[str] = None,
        entities: Optional[List[Dict[str, Union[str, dict]]]] = None  # Add but ignore
):
        """Initialize the Isaac Sim simulation."""
        super().__init__(headless, open_usd)
        self.app = SimulationApp({
            "headless": headless,
            "open_usd": open_usd
        })
        
    def get_stage(self):
        """Get the current USD stage."""
        import omni.usd
        self.stage = omni.usd.get_context().get_stage()
        return self.stage
    
    def close(self):
        """Close the simulation."""
        if hasattr(self, 'app'):
            self.app.close() 