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

"""Tests for `baked_host`: the port union and the nested stdin blob."""

from collections.abc import Mapping
import json
import pathlib
import pickle
from typing import get_args, get_origin, get_type_hints

import pytest

from dimos.core.baked_host import BakedHost, baked_host
from dimos.core.native_module import NativeModule, NativeModuleConfig
from dimos.core.stream import IO, In, Out
from dimos.msgs.geometry_msgs.PoseStamped import PoseStamped
from dimos.msgs.sensor_msgs.PointCloud2 import PointCloud2


class MapperConfig(NativeModuleConfig):
    executable: str = "unused"
    stdin_config: bool = True
    voxel_size: float = 0.1


class Mapper(NativeModule):
    config: MapperConfig

    lidar: In[PointCloud2]
    local_map: Out[PointCloud2]


class PlannerConfig(NativeModuleConfig):
    executable: str = "unused"
    stdin_config: bool = True
    world_frame: str = "map"


class Planner(NativeModule):
    config: PlannerConfig

    local_map: In[PointCloud2]
    goal: In[PoseStamped]
    path: Out[PoseStamped]


MEMBERS = {"mapper": Mapper, "planner": Planner}


def host_class(**kwargs: Mapping[tuple[str, str], str]) -> type[BakedHost]:
    return baked_host("GoNav", executable="dist/go2-nav", members=MEMBERS, **kwargs)


def ports(cls: type[NativeModule]) -> dict[str, object]:
    return {
        name: hint for name, hint in get_type_hints(cls).items() if get_origin(hint) in (In, Out)
    }


def test_ports_are_the_union_of_the_members() -> None:
    assert set(ports(host_class())) == {"lidar", "local_map", "goal", "path"}


def test_a_port_someone_produces_is_an_output_on_the_host() -> None:
    declared = ports(host_class())
    # local_map is In on the planner and Out on the mapper: producing wins.
    assert get_origin(declared["local_map"]) is Out
    assert get_args(declared["local_map"]) == (PointCloud2,)
    assert get_origin(declared["lidar"]) is In
    assert get_origin(declared["path"]) is Out


def test_remaps_rename_a_member_port_on_the_host() -> None:
    declared = ports(host_class(remaps={("planner", "local_map"): "surface"}))
    assert "surface" in declared
    assert get_origin(declared["surface"]) is In


def test_a_type_clash_between_members_is_refused() -> None:
    class Clash(NativeModule):
        config: PlannerConfig

        path: In[PointCloud2]

    with pytest.raises(ValueError, match="remap one of them"):
        baked_host("Bad", executable="x", members={"planner": Planner, "clash": Clash})


def test_member_configs_default_and_are_overridable() -> None:
    cls = host_class()
    host = cls(mapper_config=MapperConfig(voxel_size=0.25))
    assert host.config.mapper_config.voxel_size == 0.25
    assert host.config.planner_config.world_frame == "map"
    # Absolute by construction: the spawn runs from the binary's own directory.
    assert host.config.executable == str(pathlib.Path.cwd() / "dist/go2-nav")
    host.stop()


def test_a_duplex_member_port_keeps_both_directions() -> None:
    """Collapsing IO to Out would leave the host deaf on the channel."""

    class DuplexConfig(NativeModuleConfig):
        executable: str = "unused"
        stdin_config: bool = True

    class Duplex(NativeModule):
        config: DuplexConfig
        cmd: IO[PointCloud2]

    cls = baked_host("H", executable="h", members={"duplex": Duplex})
    assert get_origin(get_type_hints(cls)["cmd"]) is IO


def test_a_baked_host_spawns_its_binary_with_no_arguments() -> None:
    host = host_class()()
    # What the base class would have sent, and what the binary exits 2 on.
    assert "--mapper_config" in host.config.to_cli_args()
    assert host._argv({"lidar": "dimos/lidar", "path": "dimos/path"}) == [host.config.executable]
    host.stop()


def test_stdin_blob_nests_one_section_per_member() -> None:
    host = host_class()()
    topics = {
        "lidar": "dimos/lidar",
        "local_map": "dimos/local_map",
        "goal": "dimos/goal",
        "path": "dimos/path",
    }
    blob = json.loads(host._stdin_blob(topics))
    assert set(blob["modules"]) == {"mapper", "planner"}
    assert blob["modules"]["mapper"]["topics"] == {
        "lidar": "dimos/lidar",
        "local_map": "dimos/local_map",
    }
    assert blob["modules"]["planner"]["topics"] == {
        "local_map": "dimos/local_map",
        "goal": "dimos/goal",
        "path": "dimos/path",
    }
    assert blob["modules"]["mapper"]["config"] == {"voxel_size": 0.1}
    assert blob["modules"]["planner"]["config"] == {"world_frame": "map"}
    assert "suppress" not in blob
    host.stop()


def test_a_remapped_member_port_keeps_its_own_name_in_its_section() -> None:
    host = host_class(remaps={("planner", "local_map"): "surface"})()
    blob = json.loads(
        host._stdin_blob({"local_map": "dimos/local_map", "surface": "dimos/surface"})
    )
    assert blob["modules"]["planner"]["topics"]["local_map"] == "dimos/surface"
    assert blob["modules"]["mapper"]["topics"]["local_map"] == "dimos/local_map"
    host.stop()


def test_an_explicit_suppress_list_replaces_the_baked_one() -> None:
    host = host_class()(suppress=[])
    blob = json.loads(host._stdin_blob({}))
    assert blob["suppress"] == []
    host.stop()


def test_an_empty_member_map_is_refused() -> None:
    with pytest.raises(ValueError, match="at least one member"):
        baked_host("Empty", executable="x", members={})


Pickled = baked_host("Pickled", executable="dist/x", members=MEMBERS)


def test_the_generated_classes_pickle() -> None:
    """Deploying to a worker pickles both, so both must resolve by module path."""
    assert pickle.loads(pickle.dumps(Pickled)) is Pickled
    host = Pickled()
    assert pickle.loads(pickle.dumps(host.config)).mapper_config.voxel_size == 0.1
    host.stop()
