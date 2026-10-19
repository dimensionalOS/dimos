# Generate the product catalog from pinned inputs before native builds enter their sandbox.
{ pkgs }:
let
  generatorWheel = pkgs.fetchurl {
    url = "https://files.pythonhosted.org/packages/a3/9c/086701c3249ed2caabe83cea008c3e08bd6063c48a1df852fbca6aadff50/dimos_cdrgen-0.1.0-py3-none-any.whl";
    sha256 = "6731e8900f7e61af419b8c77599587f32677c9d64f27ac15f5e78be4b76bc011";
  };
  generator = pkgs.runCommand "dimos-cdrgen-0.1.0-source" { nativeBuildInputs = [ pkgs.unzip ]; } ''
    mkdir -p $out
    unzip -q ${generatorWheel} -d $out
  '';
  python = pkgs.python3.withPackages (p: [
    p.pip p.setuptools p.wheel p.empy p.lark p.catkin-pkg p.pyyaml
    p.rosbags p.numpy p.requests
  ]);
  product = pkgs.lib.fileset.toSource {
    root = ../packages/dimos-generated;
    fileset = pkgs.lib.fileset.unions [
      ../packages/dimos-generated/pyproject.toml
      ../packages/dimos-generated/catalog.json
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
    export PYTHONPATH="${generator}:$PYTHONPATH"
    python3 - <<'PYTHON'
from pathlib import Path
import shutil
from dimos_cdrgen.source_inputs import prepare_sources
from dimos_cdrgen.source_package import prepare
project = Path("product")
prepare_sources(project / "schema-sources.json", project / "interfaces", Path("cache/archives"), offline=True)
shutil.copytree(project / "custom/dimos_msgs/msg", project / "interfaces/dimos_msgs/msg")
shutil.copyfile(project / "custom/dimos_msgs/LICENSE", project / "interfaces/licenses/dimos_msgs-LICENSE")
prepare(project)
PYTHON
    cp -r product/build/message-source $out
    test -f $out/cpp/CMakeLists.txt
    test -f $out/rust/Cargo.toml
  '';
in {
  inherit generator python source;
}
