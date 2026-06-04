{
  description = "Wrapper flake for nav_stack PGO with M20 NOS kernel 5.10 unpackPhase workaround";

  # See sibling local_planner/flake.nix for full workaround rationale.
  # build_all.sh copies dimos/navigation/nav_stack/modules/pgo/cpp into
  # ./source before invoking this wrapper.

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    gtsam-extended = {
      url = "github:jeff-hykin/gtsam-extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
    gtsam-src = {
      url = "github:borglab/gtsam/1a9792a7ede244850a413739557635b606f295c0";
      flake = false;
    };
  };

  outputs = {
    self,
    nixpkgs,
    lcm-extended,
    dimos-lcm,
    gtsam-extended,
    gtsam-src,
    ...
  }:
    let
      system = "aarch64-linux";
      pkgs = import nixpkgs { inherit system; };

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
      kernel510WrapGtsam = drv:
        drv.overrideAttrs (old: (kernel510Hooks old) // {
          src = gtsam-src;
          env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";
        });
      nameOf = drv: drv.pname or drv.name or "";
      needsKernel510Wrap = drv:
        builtins.any
          (needle: builtins.match ".*${needle}.*" (nameOf drv) != null)
          [ "gtsam" "lcm" "pcl" ];
      patchInput = drv:
        if !(builtins.isAttrs drv && drv ? overrideAttrs && needsKernel510Wrap drv)
        then drv
        else if builtins.match ".*lcm.*" (nameOf drv) != null
        then kernel510WrapWithCmakeFlags [
          "-DLCM_ENABLE_TESTS=OFF"
          "-DLCM_INSTALL_EXAMPLES=OFF"
        ] drv
        else if builtins.match ".*gtsam.*" (nameOf drv) != null
        then kernel510WrapGtsam drv
        else kernel510WrapWithCmakeFlags [
          "-DWITH_OPENNI=OFF"
          "-DWITH_OPENNI2=OFF"
          "-DWITH_VTK=OFF"
          "-DBUILD_2d=ON"
          "-DBUILD_apps=OFF"
          "-DBUILD_benchmarks=OFF"
          "-DBUILD_cuda=OFF"
          "-DBUILD_examples=OFF"
          "-DBUILD_features=ON"
          "-DBUILD_gpu=OFF"
          "-DBUILD_io=OFF"
          "-DBUILD_keypoints=OFF"
          "-DBUILD_ml=OFF"
          "-DBUILD_outofcore=OFF"
          "-DBUILD_people=OFF"
          "-DBUILD_recognition=OFF"
          "-DBUILD_registration=ON"
          "-DBUILD_segmentation=OFF"
          "-DBUILD_simulation=OFF"
          "-DBUILD_stereo=OFF"
          "-DBUILD_surface=OFF"
          "-DBUILD_tools=OFF"
          "-DBUILD_tracking=OFF"
          "-DBUILD_visualization=OFF"
        ] drv;

      lcm = patchInput lcm-extended.packages.${system}.lcm;
      gtsam = patchInput (gtsam-extended.packages.${system}.gtsam-cpp.overrideAttrs (_old: {
        src = gtsam-src;
        env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";
      }));
    in {
      packages.${system}.default = pkgs.stdenv.mkDerivation {
        pname = "smartnav-pgo";
        version = "0.1.0";
        src = ./source;

        nativeBuildInputs = [ pkgs.cmake pkgs.pkg-config ];
        buildInputs = builtins.map patchInput [
          lcm
          pkgs.glib
          pkgs.eigen
          pkgs.boost
          pkgs.pcl
          gtsam
        ];

        env.NIX_CFLAGS_COMPILE = "-Wno-error=array-bounds";

        cmakeFlags = [
          "-DCMAKE_POLICY_VERSION_MINIMUM=3.5"
          "-DFETCHCONTENT_SOURCE_DIR_DIMOS_LCM=${dimos-lcm}"
        ];

        unpackPhase = ''
          /usr/bin/cp -r $src source
          /usr/bin/chmod -R u+w source
          cd source
        '';
        fixupPhase = ''
          /usr/bin/find $out -type f -executable -exec /usr/bin/chmod 0755 {} \;
          /usr/bin/find $out -type d -exec /usr/bin/chmod 0755 {} \;
        '';
      };
    };
}
