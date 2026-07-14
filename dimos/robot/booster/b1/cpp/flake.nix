{
  description = "Booster B1 native locomotion module";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
    booster-sdk = {
      url = "github:BoosterRobotics/booster_robotics_sdk/d5d8f7ae76d3e9f8cc224e682216f4681003ca46";
      flake = false;
    };
    dimos-lcm = {
      url = "github:dimensionalOS/dimos-lcm/main";
      flake = false;
    };
    lcm-extended = {
      url = "github:jeff-hykin/lcm_extended";
      inputs.nixpkgs.follows = "nixpkgs";
      inputs.flake-utils.follows = "flake-utils";
    };
  };

  outputs = {
    self,
    nixpkgs,
    flake-utils,
    booster-sdk,
    dimos-lcm,
    lcm-extended,
    ...
  }:
    flake-utils.lib.eachSystem [ "x86_64-linux" "aarch64-linux" ] (system:
      let
        pkgs = import nixpkgs { inherit system; };
        lcm = lcm-extended.packages.${system}.lcm;
        dimos-native-common = ../../../../hardware/sensors/lidar/common;
        sdkArch =
          if pkgs.stdenv.hostPlatform.isx86_64 then
            "x86_64"
          else
            "aarch64";

        booster-sdk-package = pkgs.stdenvNoCC.mkDerivation {
          pname = "booster-robotics-sdk";
          version = "unstable-2026-07-08";
          dontUnpack = true;
          dontConfigure = true;
          dontBuild = true;

          installPhase = ''
            runHook preInstall

            mkdir -p $out/include $out/lib/${sdkArch}
            cp -r ${booster-sdk}/include/. $out/include/
            cp -r ${booster-sdk}/lib/${sdkArch}/. $out/lib/${sdkArch}/
            cp ${booster-sdk}/LICENSE $out/LICENSE

            runHook postInstall
          '';

          meta = {
            description = "Booster Robotics C++ SDK";
            homepage = "https://github.com/BoosterRobotics/booster_robotics_sdk";
            license = pkgs.lib.licenses.asl20;
            platforms = [ "x86_64-linux" "aarch64-linux" ];
          };
        };

        booster-b1-native = pkgs.stdenv.mkDerivation {
          pname = "booster-b1-native";
          version = "0.1.0";
          src = pkgs.lib.cleanSource ./.;

          nativeBuildInputs = [
            pkgs.autoPatchelfHook
            pkgs.cmake
            pkgs.pkg-config
          ];
          buildInputs = [
            booster-sdk-package
            lcm
            pkgs.glib
          ];

          cmakeFlags = [
            "-DBOOSTER_SDK_ROOT=${booster-sdk-package}"
            "-DDIMOS_LCM_SOURCE_DIR=${dimos-lcm}"
            "-DDIMOS_NATIVE_COMMON_DIR=${dimos-native-common}"
          ];
          meta = {
            description = "dimOS native locomotion driver for the Booster B1";
            license = pkgs.lib.licenses.asl20;
            platforms = [ "x86_64-linux" "aarch64-linux" ];
            mainProgram = "booster_b1_native";
          };
        };
      in {
        packages = {
          default = booster-b1-native;
          booster-sdk = booster-sdk-package;
          inherit booster-b1-native;
        };
      });
}
