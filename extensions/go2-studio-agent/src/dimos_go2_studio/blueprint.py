"""External Blueprint loaded by ``dimos run dimos-go2-studio.go2``."""

from dimos.core.coordination.blueprints import autoconnect
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import (
    unitree_go2_agentic,
)

from .skills import Go2StudioSkills

go2_studio_agentic = autoconnect(
    unitree_go2_agentic,
    Go2StudioSkills.blueprint(),
)
