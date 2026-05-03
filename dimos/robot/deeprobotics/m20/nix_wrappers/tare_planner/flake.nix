{
  description = "Wrapper flake for dimensionalOS/dimos-module-tare-planner with M20 NOS kernel 5.10 unpackPhase workaround";

  # See sibling local_planner/flake.nix for full workaround rationale.

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    mod.url = "github:dimensionalOS/dimos-module-tare-planner/v0.1.0";
  };

  outputs = { self, nixpkgs, mod, ... }:
    let
      system = "aarch64-linux";
      pkgs = import nixpkgs { inherit system; };
      base = mod.packages.${system}.default;
      kernel510Hooks = old: {
        preUnpack = ''
          cp() { /usr/bin/cp "$@"; }
          chmod() { /usr/bin/chmod "$@"; }
          export -f cp chmod
          ${old.preUnpack or ""}
        '';
        preFixup = ''
          cp() { /usr/bin/cp "$@"; }
          chmod() { /usr/bin/chmod "$@"; }
          export -f cp chmod
          ${old.preFixup or ""}
        '';
      };
      kernel510WrapWithCmakeFlags = extraCmakeFlags: drv:
        drv.overrideAttrs (old: (kernel510Hooks old) // {
          cmakeFlags = (old.cmakeFlags or []) ++ extraCmakeFlags;
        });
      nameOf = drv: drv.pname or drv.name or "";
      needsKernel510Wrap = drv:
        builtins.any
          (needle: builtins.match ".*${needle}.*" (nameOf drv) != null)
          [ "lcm" "pcl" ];
      patchInput = drv:
        if !(builtins.isAttrs drv && drv ? overrideAttrs && needsKernel510Wrap drv)
        then drv
        else if builtins.match ".*lcm.*" (nameOf drv) != null
        then kernel510WrapWithCmakeFlags [
          "-DLCM_ENABLE_TESTS=OFF"
          "-DLCM_INSTALL_EXAMPLES=OFF"
        ] drv
        else kernel510WrapWithCmakeFlags [
          "-DWITH_OPENNI=OFF"
          "-DWITH_OPENNI2=OFF"
          "-DWITH_VTK=OFF"
          "-DBUILD_2d=OFF"
          "-DBUILD_apps=OFF"
          "-DBUILD_benchmarks=OFF"
          "-DBUILD_cuda=OFF"
          "-DBUILD_examples=OFF"
          "-DBUILD_features=OFF"
          "-DBUILD_gpu=OFF"
          "-DBUILD_io=OFF"
          "-DBUILD_keypoints=OFF"
          "-DBUILD_ml=OFF"
          "-DBUILD_outofcore=OFF"
          "-DBUILD_people=OFF"
          "-DBUILD_recognition=OFF"
          "-DBUILD_registration=OFF"
          "-DBUILD_segmentation=OFF"
          "-DBUILD_simulation=OFF"
          "-DBUILD_stereo=OFF"
          "-DBUILD_surface=OFF"
          "-DBUILD_tools=OFF"
          "-DBUILD_tracking=OFF"
          "-DBUILD_visualization=OFF"
        ] drv;
    in {
      packages.${system}.default = base.overrideAttrs (old: {
        buildInputs = builtins.map patchInput (old.buildInputs or []);
        unpackPhase = ''
          /usr/bin/cp -r $src source
          /usr/bin/chmod -R u+w source
          cd source
        '';
        fixupPhase = ''
          /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
          /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
        '';
        postPatch = ''
          ${old.postPatch or ""}
          /usr/bin/sed -i '/#include <pcl\/segmentation\/extract_clusters.h>/d' main.cpp
        '';
      });
    };
}
