# Build product schemas using the published generator and pinned source archives.
# All downloads happen before entering the build sandbox.
{ pkgs }:
let
  messageSource = import ../message-source.nix { inherit pkgs; };
  inherit (messageSource) generator python;
  lock = builtins.fromJSON (builtins.readFile "${generator}/dimos_cdrgen/native_sources.json");
  archivePins = builtins.fromJSON (builtins.readFile ./source-archives.json);
  archives = builtins.mapAttrs (name: source:
    assert archivePins.${name}.revision == source.revision;
    pkgs.fetchurl {
      url = "https://codeload.github.com/${pkgs.lib.removePrefix "https://github.com/" (pkgs.lib.removeSuffix ".git" source.url)}/tar.gz/${source.revision}";
      sha256 = archivePins.${name}.sha256;
    }
  ) lock.repositories;
  fastcdr = pkgs.fetchurl {
    url = lock.fastcdr.url;
    sha256 = lock.fastcdr.sha256;
  };
  unpack = name: archive: ''
    mkdir -p upstream/${name}
    tar -xf ${archive} --strip-components=1 -C upstream/${name}
    chmod -R u+w upstream/${name}
  '';
  support = pkgs.stdenv.mkDerivation {
    pname = "dimos-native-message-support";
    version = "0.1.0";
    dontUnpack = true;
    nativeBuildInputs = [ pkgs.cmake python ];
    configurePhase = ''
      runHook preConfigure
      mkdir source
      cp ${generator}/dimos_cdrgen/templates/native-support.cmake source/CMakeLists.txt
      cp ${generator}/dimos_cdrgen/templates/native-toolchain.cmake source/native-toolchain.cmake
      cp ${generator}/dimos_cdrgen/native_sources.json source/native_sources.json
      ${pkgs.lib.concatStringsSep "\n" (pkgs.lib.mapAttrsToList unpack archives)}
      ${unpack "fastcdr" fastcdr}
      export PIP_NO_INDEX=1 PIP_DISABLE_PIP_VERSION_CHECK=1
      cmake -S source -B build \
        -DPython3_EXECUTABLE=${python}/bin/python3 \
        -DDIMOS_SUPPORT_PREFIX="$out" \
        -DFETCHCONTENT_FULLY_DISCONNECTED=ON \
        -DFETCHCONTENT_SOURCE_DIR_FASTCDR="$PWD/upstream/fastcdr" \
        ${pkgs.lib.concatStringsSep " " (pkgs.lib.mapAttrsToList (name: _: "-DFETCHCONTENT_SOURCE_DIR_${pkgs.lib.toUpper name}=\"$PWD/upstream/${name}\"") archives)}
      runHook postConfigure
    '';
    buildPhase = "cmake --build build --parallel $NIX_BUILD_CORES";
    # ExternalProject installs each prerequisite as part of its build.
    installPhase = "test -d $out/share/rosidl_generator_cpp";
  };
in pkgs.stdenv.mkDerivation {
  pname = "dimos-generated-messages";
  version = "0.1.0";
  src = "${messageSource.source}/cpp";
  nativeBuildInputs = [ pkgs.cmake python ];
  propagatedBuildInputs = [ support ];
  # Installed ament CMake exports require Python even for C++ consumers.
  propagatedNativeBuildInputs = [ python ];
  setupHook = pkgs.writeText "dimos-message-python-hook" ''
    addToSearchPath PYTHONPATH "${support}/${pkgs.python3.sitePackages}"
  '';
  preConfigure = ''
    export PYTHONPATH="${support}/${pkgs.python3.sitePackages}:$PYTHONPATH"
    export AMENT_PREFIX_PATH="${support}:$out"
    cmakeFlagsArray+=("-DCMAKE_PREFIX_PATH=${support};$out")
  '';
  cmakeFlags = [
    "-DPython3_EXECUTABLE=${python}/bin/python3"
    "-DDIMOS_RUNTIME_PATHS=${support}/lib"
  ];
}
