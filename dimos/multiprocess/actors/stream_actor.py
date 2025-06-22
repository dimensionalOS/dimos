# Copyright 2025 Dimensional Inc.
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

from typing import Dict

from reactivex import operators as ops
from reactivex.disposable import Disposable
from reactivex.observable import Observable
from reactivex.scheduler import ThreadPoolScheduler
from reactivex.subject import Subject

from dimos.utils.reactive import backpressure


class DimosModule: ...


class Nav(DimosModule):
    target_path_stream: Subject

    def __init__(self, lidar_stream: Observable, target_stream: Observable): ...


class StreamActor:
    subscriptions: Dict[str, Subject] = {}
    streams: Dict[str, Observable] = {}

    def subscribe(self, actor, channel, callback):
        subject = Subject()
        self.subscriptions[channel] = subject
        actor.register_remote_subscriber(self, channel)

        def unsubscribe():
            self.subscriptions.pop(channel, None)
            actor.unregister_remote_subscriber(self, channel)

        subject.on_dispose(unsubscribe)

        return subject

    def receive_message(self, channel, data):
        self.subscriptions[channel].emit(data)

    def register_remote_subscriber(self, actor, channel):
        # Logic to register this actor as a subscriber to the remote actor
        pass
