# Built-in message package generated from the same pinned inputs as Cargo/Python.
{ pkgs }:
let
  fastcdr = pkgs.stdenv.mkDerivation {
    pname = "fastcdr";
    version = "2.4.0";
    src = pkgs.fetchurl {
      url = "https://github.com/eProsima/Fast-CDR/archive/refs/tags/v2.4.0.tar.gz";
      sha256 = "79d8466107dd6b7d1defe961c4aa31735038937cf9dd1175cf6b0da0df2209ab";
    };
    nativeBuildInputs = [ pkgs.cmake ];
    cmakeFlags = [ "-DBUILD_TESTING=OFF" "-DBUILD_SHARED_LIBS=OFF" "-DCMAKE_POSITION_INDEPENDENT_CODE=ON" ];
  };
in pkgs.stdenv.mkDerivation {
  pname = "dimos-generated-messages";
  version = "0.1.0";
  src = pkgs.lib.fileset.toSource {
    root = ../..;
    fileset = pkgs.lib.fileset.unions [ ../../dimos/__init__.py ../../dimos/message_codegen ];
  };
  nativeBuildInputs = [ pkgs.cmake pkgs.python3 ];
  propagatedBuildInputs = [ fastcdr ];
  postPatch = "python3 -m dimos.message_codegen.generate --output generated";
  cmakeDir = "../generated/cpp";
  cmakeFlags = [ "-DDIMOS_BUILD_PYTHON=OFF" ];
}
