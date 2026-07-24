"""External Blueprint loaded by ``dimos run dimos-go2-studio.go2``."""

from dimos.agents.skills.person_follow import PersonFollowSkillContainer
from dimos.core.coordination.blueprints import autoconnect
from dimos.navigation.movement_manager.movement_manager import MovementManager
from dimos.robot.unitree.go2.blueprints.agentic.unitree_go2_agentic import (
    unitree_go2_agentic,
)
from dimos.robot.unitree.unitree_skill_container import UnitreeSkillContainer

from .skills import Go2StudioSkills

go2_studio_agentic = (
    autoconnect(
        unitree_go2_agentic,
        MovementManager.blueprint(
            max_nav_linear_speed=0.1,
            allow_nav_reverse=False,
        ),
        Go2StudioSkills.blueprint(),
    )
    .disabled_modules(UnitreeSkillContainer, PersonFollowSkillContainer)
    .global_config(nerf_speed=0.18)
)
