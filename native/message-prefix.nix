# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Use the generator's own locked ROSIDL support closure, not a second ROS overlay.
{ pkgs }:
let
  generation = import ./message-source.nix { inherit pkgs; };
  lock = builtins.fromJSON (builtins.readFile (generation.generator + "/dimos_cdrgen/native_sources.json"));
  repositories = pkgs.lib.mapAttrs (_: spec: builtins.fetchGit {
    inherit (spec) url;
    rev = spec.revision;
    allRefs = true;
  }) lock.repositories;
  fastcdrArchive = pkgs.fetchurl { inherit (lock.fastcdr) url sha256; };
  fastcdr = pkgs.runCommand "fastcdr-${lock.fastcdr.version}-source" { } ''
    mkdir -p $out
    tar -xzf ${fastcdrArchive} --strip-components=1 -C $out
  '';
  sources = repositories // { inherit fastcdr; };
  python = pkgs.python3.withPackages (p: [
    p.pip p.setuptools p.wheel p.pydantic p.empy p.lark p.catkin-pkg p.pyyaml p.numpy p.rosbags
  ]);
in pkgs.stdenv.mkDerivation {
  pname = "dimos-generated-message-prefix";
  version = "0.0.14";
  dontUnpack = true;
  nativeBuildInputs = [ python pkgs.cmake pkgs.pkg-config ];
  buildPhase = ''
    export PYTHONPATH="${generation.generator}:$PYTHONPATH"
    export DIMOS_NATIVE_SOURCES='${builtins.toJSON (pkgs.lib.mapAttrs (_: value: toString value) sources)}'
    export DIMOS_GENERATED_PACKAGE="${generation.source}/python/dimos/messages_schemas/package"
    export DIMOS_NATIVE_OUTPUT="$out"
    mkdir -p "$out"
    python3 - <<'PYTHON'
import json
import os
from pathlib import Path
from dimos_cdrgen.native_build import prepare_cpp, write_cmake_toolchain
output = Path(os.environ["DIMOS_NATIVE_OUTPUT"])
prefix = prepare_cpp(
    Path(os.environ["DIMOS_GENERATED_PACKAGE"]),
    cache=output / "cache",
    offline=True,
    source_dirs={name: Path(path) for name, path in json.loads(os.environ["DIMOS_NATIVE_SOURCES"]).items()},
)
write_cmake_toolchain(prefix, output / "toolchain.cmake")
(output / "prefix").symlink_to(prefix, target_is_directory=True)
PYTHON
  '';
  installPhase = "test -f $out/toolchain.cmake";
}
