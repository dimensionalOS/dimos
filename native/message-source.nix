# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Generate the built-in definitions through the same configuration and core as pip.
# Nix fetches fixed-hash inputs; the generation builder itself is offline.
{ pkgs }:
let
  generatorWheel = pkgs.fetchurl {
    url = "https://files.pythonhosted.org/packages/29/9f/a4fe6f8ea4dcafd3001c868ebbc751bb0af83aecb1fe6523765c12cdedba/dimos_cdrgen-0.1.2-py3-none-any.whl";
    sha256 = "9857c8cc40de58b03f94cb81e7d27ae3caac98101c96cdaa2965413a332e685f";
  };
  generator = pkgs.runCommand "dimos-cdrgen-0.1.2-source" { nativeBuildInputs = [ pkgs.unzip ]; } ''
    mkdir -p $out
    unzip -q ${generatorWheel} -d $out
  '';
  python = pkgs.python3.withPackages (p: [
    p.setuptools p.wheel p.pydantic p.empy p.lark p.rosbags p.numpy p.requests
  ]);
  product = pkgs.lib.fileset.toSource {
    root = ../.;
    fileset = pkgs.lib.fileset.unions [
      ../pyproject.toml
      ../dimos/message_codegen/build.py
      ../packages/dimos-generated/schema-sources.json
      ../packages/dimos-generated/custom
    ];
  };
  schemaLock = builtins.fromJSON (builtins.readFile ../packages/dimos-generated/schema-sources.json);
  schemaArchives = map (source: {
    name = builtins.hashString "sha256" source.url + ".tar.gz";
    archive = pkgs.fetchurl { inherit (source) url sha256; };
  }) schemaLock.sources;
  source = pkgs.runCommand "dimos-generated-message-source" { nativeBuildInputs = [ python ]; } ''
    cp -r ${product} product
    chmod -R u+w product
    mkdir -p cache/archives
    ${pkgs.lib.concatMapStringsSep "\n" (item: "cp ${item.archive} cache/archives/${item.name}") schemaArchives}
    export PYTHONPATH="$PWD/product:${generator}:$PYTHONPATH"
    python3 - <<'PYTHON'
from pathlib import Path
from dimos.message_codegen.build import prepare_builtin_sources
from dimos_cdrgen.source_package import prepare
prepare_builtin_sources(Path("cache"), offline=True)
prepare(Path("product"))
PYTHON
    cp -r product/build/message-source $out
    test -f $out/cpp/CMakeLists.txt
    test -f $out/rust/Cargo.toml
  '';
in {
  inherit generator python source;
}
