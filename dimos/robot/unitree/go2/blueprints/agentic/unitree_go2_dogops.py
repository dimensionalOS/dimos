try:  # pragma: no cover - exercised inside a full DimOS checkout.
    from dimos.agents.mcp.mcp_server import McpServer
    from dimos.core.coordination.blueprints import autoconnect
    from dimos.experimental.dogops.dashboard import DogOpsDashboardModule
    from dimos.experimental.dogops.nav_eval import DogOpsNavEvalModule
    from dimos.experimental.dogops.observation_module import DogOpsObservationModule
    from dimos.experimental.dogops.skills import DogOpsSkillContainer
    from dimos.robot.unitree.go2.blueprints.smart.unitree_go2 import unitree_go2_markers
except ModuleNotFoundError:  # pragma: no cover - covered by local pack fallback tests.
    class _FallbackBlueprint:
        def __init__(self, *modules: object) -> None:
            self.modules = modules

        def global_config(self, **_: object) -> "_FallbackBlueprint":
            return self

        def __repr__(self) -> str:
            return "unitree-go2-dogops " + " ".join(str(module) for module in self.modules)

    class _FallbackModule:
        @classmethod
        def blueprint(cls) -> str:
            return cls.__name__

    def autoconnect(*modules: object) -> _FallbackBlueprint:
        return _FallbackBlueprint(*modules)

    unitree_go2_markers = "unitree_go2_markers"
    DogOpsObservationModule = type("DogOpsObservationModule", (_FallbackModule,), {})
    DogOpsSkillContainer = type("DogOpsSkillContainer", (_FallbackModule,), {})
    DogOpsDashboardModule = type("DogOpsDashboardModule", (_FallbackModule,), {})
    DogOpsNavEvalModule = type("DogOpsNavEvalModule", (_FallbackModule,), {})
    McpServer = type("McpServer", (_FallbackModule,), {})


unitree_go2_dogops = autoconnect(
    unitree_go2_markers,
    DogOpsObservationModule.blueprint(),
    DogOpsSkillContainer.blueprint(runtime_module=True),
    DogOpsDashboardModule.blueprint(),
    DogOpsNavEvalModule.blueprint(),
    McpServer.blueprint(),
).global_config(n_workers=12, robot_model="unitree_go2")

__all__ = ["unitree_go2_dogops"]
