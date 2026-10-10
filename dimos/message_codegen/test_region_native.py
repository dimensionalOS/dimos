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

"""Compiled region contracts preserve IDs, cylinder bounds and independent edges."""

import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from dimos.message_codegen.generate import generate
from dimos.message_codegen.native_build import prepare_cpp, write_cmake_toolchain


def test_region_cdr_exchange_through_cpp_and_rust(tmp_path):
    if os.environ.get("DIMOS_NATIVE_ACCEPTANCE") != "1":
        pytest.skip("Set DIMOS_NATIVE_ACCEPTANCE=1 for compiled region acceptance")
    root = Path(__file__).resolve().parents[2]
    package = tmp_path / "package"
    generate(
        [],
        package,
        [
            "dimos_msgs/msg/RegionBounds",
            "dimos_msgs/msg/RegionPointCloud2",
            "dimos_msgs/msg/RegionLineSegments3D",
            "dimos_msgs/msg/Contacts",
        ],
        "region_messages",
        shared=True,
    )
    sources = {
        name: Path(path)
        for name, path in json.loads(os.environ.get("DIMOS_NATIVE_SOURCE_DIRS", "{}")).items()
    }
    cache = os.environ.get("DIMOS_NATIVE_TEST_CACHE")
    prefix = prepare_cpp(
        package,
        cache=Path(cache) if cache else None,
        offline=os.environ.get("DIMOS_OFFLINE") == "1",
        source_dirs=sources or None,
    )
    toolchain = write_cmake_toolchain(prefix, tmp_path / "toolchain.cmake")
    (tmp_path / "dimos_message_build").symlink_to(Path(__file__).parent, target_is_directory=True)
    env = {**os.environ, "PYTHONPATH": os.pathsep.join([str(tmp_path), str(package / "python")])}
    bootstrap = """
from pathlib import Path
from dimos_message_build import registry
registry.providers = lambda: ()
registry.initialize((Path('package'),), discover=False)
from region_messages.dimos_msgs.msg import RegionBounds, RegionPointCloud2, RegionLineSegments3D, LineSegments3D, LineSegment3D
from region_messages.geometry_msgs.msg import Point
from region_messages.sensor_msgs.msg import PointCloud2
from region_messages.std_msgs.msg import Header
from region_messages.builtin_interfaces.msg import Time
import numpy as np
header = Header(stamp=Time(sec=12, nanosec=34), frame_id='map')
values = [
 RegionBounds(header=header, region_id=-196603, center=Point(x=-3.,y=5.,z=0.), radius=2.5,z_min=-1.,z_max=4.),
 RegionPointCloud2(region_id=-196603,cloud=PointCloud2(header=header,height=1,width=0,fields=[],is_bigendian=False,point_step=12,row_step=0,data=np.empty(0,dtype=np.uint8),is_dense=True)),
 RegionLineSegments3D(region_id=-196603,lines=LineSegments3D(header=header,segments=[
  LineSegment3D(start=Point(x=1.,y=2.,z=3.),end=Point(x=4.,y=5.,z=6.),weight=0.5),
  LineSegment3D(start=Point(x=9.,y=8.,z=7.),end=Point(x=6.,y=5.,z=4.),weight=100.)]))]
"""
    subprocess.run(
        [
            sys.executable,
            "-c",
            bootstrap
            + "\nfor i,value in enumerate(values): Path(f'input{i}.cdr').write_bytes(registry.encode(value))",
        ],
        cwd=tmp_path,
        env=env,
        check=True,
    )
    consumer = tmp_path / "consumer"
    consumer.mkdir()
    (consumer / "CMakeLists.txt").write_text(f"""cmake_minimum_required(VERSION 3.20)
project(region_consumer LANGUAGES CXX)
find_package(region_messages CONFIG REQUIRED)
add_executable(consumer main.cpp)
target_compile_features(consumer PRIVATE cxx_std_17)
target_include_directories(consumer PRIVATE "{root}/native/cpp/include")
target_link_libraries(consumer PRIVATE region_messages::messages)
""")
    (consumer / "main.cpp").write_text("""#include <dimos_msgs/msg/region_bounds.hpp>
#include <dimos_msgs/msg/region_point_cloud2.hpp>
#include <dimos_msgs/msg/region_line_segments3_d.hpp>
#include <dimos/native/cdr_codec.hpp>
#include <fstream>
#include <iterator>
#include <stdexcept>
template<class T> T read(int i) {
 std::ifstream file("input"+std::to_string(i)+".cdr",std::ios::binary);
 std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(file)),{});
 auto value=dimos::native::cdr_decode<T>(bytes);
 if(value.region_id!=-196603) throw std::runtime_error("region ID lost");
 auto wire=dimos::native::cdr_encode(value);
 std::ofstream output("cpp"+std::to_string(i)+".cdr",std::ios::binary);
 output.write(reinterpret_cast<const char*>(wire.data()),wire.size());
 return value;
}
int main() {
 auto bounds=read<dimos_msgs::msg::RegionBounds>(0);
 if(bounds.center.x!=-3 || bounds.radius!=2.5 || bounds.z_min!=-1 || bounds.z_max!=4) return 1;
 auto cloud=read<dimos_msgs::msg::RegionPointCloud2>(1);
 if(cloud.cloud.width!=0 || cloud.cloud.header.frame_id!="map") return 2;
 auto edges=read<dimos_msgs::msg::RegionLineSegments3D>(2);
 if(edges.lines.segments.size()!=2 || edges.lines.segments[1].start.x!=9 || edges.lines.segments[1].weight!=100) return 3;
}
""")
    subprocess.run(
        [
            "cmake",
            "-S",
            str(consumer),
            "-B",
            str(tmp_path / "cpp"),
            f"-DCMAKE_TOOLCHAIN_FILE={toolchain}",
        ],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(tmp_path / "cpp"), "--parallel", "2"], check=True)
    subprocess.run([str(tmp_path / "cpp/consumer")], cwd=tmp_path, check=True)
    binary = package / "rust/src/bin"
    binary.mkdir()
    (
        binary / "exchange.rs"
    ).write_text("""use region_messages_messages::dimos_msgs::msg::{region_bounds::RegionBounds,region_point_cloud2::RegionPointCloud2,region_line_segments3_d::RegionLineSegments3D};
fn main() {
 for i in 0..3 {
  let wire=std::fs::read(format!("cpp{i}.cdr")).unwrap();
  let body=match i {
   0=>{let (v,n)=re_cdr::from_bytes::<RegionBounds,re_cdr::LittleEndian>(&wire[4..]).unwrap(); assert_eq!(n,wire.len()-4);assert_eq!((v.region_id,v.center.x,v.radius,v.z_min,v.z_max),(-196603,-3.,2.5,-1.,4.)); re_cdr::to_vec::<_,re_cdr::LittleEndian>(&v).unwrap()},
   1=>{let (v,n)=re_cdr::from_bytes::<RegionPointCloud2,re_cdr::LittleEndian>(&wire[4..]).unwrap();assert_eq!(n,wire.len()-4);assert_eq!(v.region_id,-196603);assert_eq!(v.cloud.header.frame_id,"map");assert_eq!(v.cloud.width,0); re_cdr::to_vec::<_,re_cdr::LittleEndian>(&v).unwrap()},
   _=>{let (v,n)=re_cdr::from_bytes::<RegionLineSegments3D,re_cdr::LittleEndian>(&wire[4..]).unwrap();assert_eq!(n,wire.len()-4);assert_eq!(v.region_id,-196603);assert_eq!(v.lines.segments.len(),2);assert_eq!((v.lines.segments[1].start.x,v.lines.segments[1].weight),(9.,100.));re_cdr::to_vec::<_,re_cdr::LittleEndian>(&v).unwrap()}
  };
  let mut output=vec![0,1,0,0];output.extend(body);std::fs::write(format!("rust{i}.cdr"),output).unwrap();
 }
}
""")
    subprocess.run(
        [
            "cargo",
            "run",
            *(["--offline"] if os.environ.get("DIMOS_OFFLINE") == "1" else []),
            "--manifest-path",
            str(package / "rust/Cargo.toml"),
            "--bin",
            "exchange",
        ],
        cwd=tmp_path,
        check=True,
    )
    subprocess.run(
        [
            sys.executable,
            "-c",
            bootstrap
            + "\nfor i,value in enumerate(values): assert registry.encode(registry.decode(Path(f'rust{i}.cdr').read_bytes(),type(value))) == registry.encode(value)",
        ],
        cwd=tmp_path,
        env=env,
        check=True,
    )
