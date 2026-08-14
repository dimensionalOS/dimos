# mypy: ignore-errors
"""gRPC bindings generated from libero_pro.proto."""

import grpc

from . import libero_pro_pb2 as pb2


class PolicyInterfaceStub:
    def __init__(self, channel: grpc.Channel) -> None:
        self.GetHealth = channel.unary_unary(
            "/dimos.benchmark.libero_pro.v1.PolicyInterface/GetHealth",
            request_serializer=pb2.Empty.SerializeToString,
            response_deserializer=pb2.Health.FromString,
        )
        self.WatchState = channel.unary_stream(
            "/dimos.benchmark.libero_pro.v1.PolicyInterface/WatchState",
            request_serializer=pb2.WatchRequest.SerializeToString,
            response_deserializer=pb2.RobotSnapshot.FromString,
        )
        self.SetJointTargets = channel.unary_unary(
            "/dimos.benchmark.libero_pro.v1.PolicyInterface/SetJointTargets",
            request_serializer=pb2.JointTargets.SerializeToString,
            response_deserializer=pb2.Ack.FromString,
        )


class EvaluationControlStub:
    def __init__(self, channel: grpc.Channel) -> None:
        prefix = "/dimos.benchmark.libero_pro.v1.EvaluationControl/"
        self.GetHealth = channel.unary_unary(
            prefix + "GetHealth",
            request_serializer=pb2.Empty.SerializeToString,
            response_deserializer=pb2.Health.FromString,
        )
        self.InitializeTrial = channel.unary_unary(
            prefix + "InitializeTrial",
            request_serializer=pb2.InitializeTrialRequest.SerializeToString,
            response_deserializer=pb2.TrialReady.FromString,
        )
        for name in ("StartTrial", "WaitForTerminal", "CancelTrial", "GetNativeResult"):
            response = pb2.Empty if name == "StartTrial" else pb2.TerminalResult
            setattr(
                self,
                name,
                channel.unary_unary(
                    prefix + name,
                    request_serializer=pb2.Empty.SerializeToString,
                    response_deserializer=response.FromString,
                ),
            )


def add_PolicyInterfaceServicer_to_server(servicer, server) -> None:
    handlers = {
        "GetHealth": grpc.unary_unary_rpc_method_handler(
            servicer.GetHealth,
            request_deserializer=pb2.Empty.FromString,
            response_serializer=pb2.Health.SerializeToString,
        ),
        "WatchState": grpc.unary_stream_rpc_method_handler(
            servicer.WatchState,
            request_deserializer=pb2.WatchRequest.FromString,
            response_serializer=pb2.RobotSnapshot.SerializeToString,
        ),
        "SetJointTargets": grpc.unary_unary_rpc_method_handler(
            servicer.SetJointTargets,
            request_deserializer=pb2.JointTargets.FromString,
            response_serializer=pb2.Ack.SerializeToString,
        ),
    }
    server.add_generic_rpc_handlers(
        (
            grpc.method_handlers_generic_handler(
                "dimos.benchmark.libero_pro.v1.PolicyInterface", handlers
            ),
        )
    )


def add_EvaluationControlServicer_to_server(servicer, server) -> None:
    handlers = {}
    for name, request, response in (
        ("GetHealth", pb2.Empty, pb2.Health),
        ("InitializeTrial", pb2.InitializeTrialRequest, pb2.TrialReady),
        ("StartTrial", pb2.Empty, pb2.Empty),
        ("WaitForTerminal", pb2.Empty, pb2.TerminalResult),
        ("CancelTrial", pb2.Empty, pb2.TerminalResult),
        ("GetNativeResult", pb2.Empty, pb2.TerminalResult),
    ):
        handlers[name] = grpc.unary_unary_rpc_method_handler(
            getattr(servicer, name),
            request_deserializer=request.FromString,
            response_serializer=response.SerializeToString,
        )
    server.add_generic_rpc_handlers(
        (
            grpc.method_handlers_generic_handler(
                "dimos.benchmark.libero_pro.v1.EvaluationControl", handlers
            ),
        )
    )
