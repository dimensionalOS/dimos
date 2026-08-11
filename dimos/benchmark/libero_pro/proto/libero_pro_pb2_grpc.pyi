from typing import Any

class PolicyInterfaceStub:
    GetHealth: Any
    WatchState: Any
    SetJointTargets: Any
    def __init__(self, channel: Any) -> None: ...

class EvaluationControlStub:
    GetHealth: Any
    InitializeTrial: Any
    StartTrial: Any
    WaitForTerminal: Any
    CancelTrial: Any
    GetNativeResult: Any
    def __init__(self, channel: Any) -> None: ...

def add_PolicyInterfaceServicer_to_server(servicer: Any, server: Any) -> None: ...
def add_EvaluationControlServicer_to_server(servicer: Any, server: Any) -> None: ...
