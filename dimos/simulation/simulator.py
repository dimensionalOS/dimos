from isaacsim import SimulationApp
from typing import Optional

class Simulator:
    """Wrapper class for Isaac Sim simulation."""
    
    def __init__(self, headless: bool = True, open_usd: Optional[str] = None):
        """Initialize the Isaac Sim simulation.
        
        Args:
            headless (bool): Whether to run in headless mode. Defaults to True.
            open_usd (Optional[str]): USD file to open on startup. Defaults to None.
        """
        self.app = SimulationApp({
            "headless": headless,
            "open_usd": open_usd
        })
        self.stage = None
        
    def get_stage(self):
        """Get the current USD stage."""
        import omni.usd
        self.stage = omni.usd.get_context().get_stage()
        return self.stage
    
    def close(self):
        """Close the simulation."""
        if self.app:
            self.app.close() 