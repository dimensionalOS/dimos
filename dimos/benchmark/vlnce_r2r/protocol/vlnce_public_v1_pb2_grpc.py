# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Small gRPC binding shared by the host and pinned Python 3.6 container."""

import grpc  # type: ignore[import-untyped]

from . import vlnce_public_v1_pb2 as messages

_SERVICE = "dimos.benchmark.vlnce_r2r.public.v1.VlncePublicGateway"


class VlncePublicGatewayStub:
    def __init__(self, channel):
        self.Start = _method(channel, "Start", messages.StartRequest, messages.Observation)
        self.StepPlanar = _method(
            channel, "StepPlanar", messages.PlanarControl, messages.Observation
        )
        self.SubmitRoute = _method(
            channel, "SubmitRoute", messages.SubmitRouteRequest, messages.Acknowledgement
        )
        self.Cancel = _method(channel, "Cancel", messages.CancelRequest, messages.Acknowledgement)


class VlncePublicGatewayServicer:
    def Start(self, request, context):
        raise NotImplementedError

    def StepPlanar(self, request, context):
        raise NotImplementedError

    def SubmitRoute(self, request, context):
        raise NotImplementedError

    def Cancel(self, request, context):
        raise NotImplementedError


def add_VlncePublicGatewayServicer_to_server(servicer, server):
    handlers = {
        name: grpc.unary_unary_rpc_method_handler(
            getattr(servicer, name),
            request_deserializer=request.FromString,
            response_serializer=response.SerializeToString,
        )
        for name, request, response in (
            ("Start", messages.StartRequest, messages.Observation),
            ("StepPlanar", messages.PlanarControl, messages.Observation),
            ("SubmitRoute", messages.SubmitRouteRequest, messages.Acknowledgement),
            ("Cancel", messages.CancelRequest, messages.Acknowledgement),
        )
    }
    server.add_generic_rpc_handlers((grpc.method_handlers_generic_handler(_SERVICE, handlers),))


def _method(channel, name, request, response):
    return channel.unary_unary(
        f"/{_SERVICE}/{name}",
        request_serializer=request.SerializeToString,
        response_deserializer=response.FromString,
    )
