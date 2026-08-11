# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""gRPC bindings for the public-only VLN-CE gateway."""

from typing import Iterable  # noqa: UP035 - generated binding also runs on Python 3.6

import grpc  # type: ignore[import-untyped]

from . import vlnce_public_v1_pb2 as messages

_METHOD = "/dimos.benchmark.vlnce_r2r.public.v1.VlncePublicGateway/Stream"


class VlncePublicGatewayStub:
    def __init__(self, channel: grpc.Channel) -> None:
        self.Stream = channel.stream_stream(
            _METHOD,
            request_serializer=messages.ClientMessage.SerializeToString,
            response_deserializer=messages.ServerMessage.FromString,
        )


class VlncePublicGatewayServicer:
    def Stream(
        self,
        request_iterator: Iterable[messages.ClientMessage],
        context: grpc.ServicerContext,
    ) -> Iterable[messages.ServerMessage]:
        raise NotImplementedError


def add_VlncePublicGatewayServicer_to_server(
    servicer: VlncePublicGatewayServicer, server: grpc.Server
) -> None:
    handler = grpc.stream_stream_rpc_method_handler(
        servicer.Stream,
        request_deserializer=messages.ClientMessage.FromString,
        response_serializer=messages.ServerMessage.SerializeToString,
    )
    server.add_generic_rpc_handlers(
        (
            grpc.method_handlers_generic_handler(
                "dimos.benchmark.vlnce_r2r.public.v1.VlncePublicGateway",
                {"Stream": handler},
            ),
        )
    )
