# Copyright 2025-2026 Dimensional Inc.
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


from dimos.core.blueprints import autoconnect
from dimos.core.introspection.blueprint.dot import render
from dimos.core.module import Module
from dimos.core.stream import In, Out


class MsgA:
    pass


class MsgB:
    pass


class ProducerModule(Module):
    output_a: Out[MsgA]
    output_b: Out[MsgB]


class ConsumerModule(Module):
    output_a: In[MsgA]


# output_a connects (same name+type), output_b is disconnected (no consumer)
_combined = autoconnect(ProducerModule.blueprint(), ConsumerModule.blueprint())


def test_render_without_disconnected() -> None:
    dot = render(_combined, ignored_streams=set(), ignored_modules=set(), show_disconnected=False)
    # Connected channel should be present
    assert "output_a:MsgA" in dot
    # Disconnected output_b should NOT appear
    assert "output_b:MsgB" not in dot


def test_render_with_disconnected() -> None:
    dot = render(_combined, ignored_streams=set(), ignored_modules=set(), show_disconnected=True)
    # Connected channel should be present
    assert "output_a:MsgA" in dot
    # Disconnected output_b SHOULD appear with dashed style
    assert "output_b:MsgB" in dot
    assert "style=dashed" in dot


def test_disconnected_default_is_false() -> None:
    dot = render(_combined, ignored_streams=set(), ignored_modules=set())
    assert "output_b:MsgB" not in dot
