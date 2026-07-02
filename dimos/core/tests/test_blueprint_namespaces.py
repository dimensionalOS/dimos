from dimos.core.coordination.blueprints import autoconnect
from dimos.core.coordination.test_blueprints import ModuleA, ModuleB


def test_namespaces_allow_duplicates_and_scope(dynamic_coordinator):
    """
    verify that we can deploy the same module classes multiple times as long as they are isolated within different namespaces.
    """
    robot1_bp = autoconnect(ModuleA.blueprint(), ModuleB.blueprint()).namespace("robot1")
    robot2_bp = autoconnect(ModuleA.blueprint(), ModuleB.blueprint()).namespace("robot2")
    system_bp = autoconnect(robot1_bp, robot2_bp)
    assert len(system_bp.blueprints) == 4, "system blueprint should contain exactly 4 atoms"
    dynamic_coordinator.load_blueprint(system_bp)
    assert dynamic_coordinator.n_modules == 4, (
        "coordinator should be tracking exactly 4 active modules"
    )
