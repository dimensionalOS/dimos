# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Generate the built-in definitions through the same configuration and core as pip.
# Nix fetches fixed-hash inputs; the generation builder itself is offline.
{ pkgs }:
let
  generatorWheel = pkgs.fetchurl {
    url = "https://files.pythonhosted.org/packages/1a/05/cb085e18549a5eced615840b2f607a9c342a8a49c69012964f0568aa309b/dimos_cdrgen-0.1.1-py3-none-any.whl";
    sha256 = "f85923440196a3028dbd974a913b0ef9320eb12b57770f722e96bc1645a351a2";
  };
  generator = pkgs.runCommand "dimos-cdrgen-0.1.1-source" { nativeBuildInputs = [ pkgs.unzip ]; } ''
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
