{
  description = "DJI Mini 4 Pro -> MAVLink bridge (Android / DJI MSDK v5)";

  inputs = {
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    flake-utils.url = "github:numtide/flake-utils";
  };

  outputs = { self, nixpkgs, flake-utils, ... }:
    flake-utils.lib.eachDefaultSystem (system:
      let
        pkgs = import nixpkgs {
          inherit system;
          config = {
            allowUnfree = true;
            android_sdk.accept_license = true;
          };
        };

        # Matches the MSDK v5.18 sample: compileSdk/targetSdk 35, minSdk 24,
        # AGP 8.7.0, Gradle 8.12, Kotlin 2.1.0.
        buildToolsVersion = "35.0.0";

        # The NDK and the SDK-layout CMake exist for ONE thing: the vendored
        # `apriltag` C library under android/app/src/apriltag, which is compiled
        # into the app APK and measured against OpenCV's ArUco from the on-phone
        # profiler. Nothing in the shipped Kotlin path needs native code.
        #
        # Both versions are pinned rather than left to AGP. AGP 8.7 defaults to
        # NDK 26.1 and CMake 3.22.1 and would try to *download* them into the
        # read-only Nix store SDK, which fails the same way build-tools does —
        # so whatever versions the store provides must also be the ones the
        # module asks for (see android/app/build.gradle).
        ndkVersion = "27.0.12077973";
        cmakeVersion = "3.22.1";

        androidComposition = pkgs.androidenv.composeAndroidPackages {
          cmdLineToolsVersion = "13.0";
          platformToolsVersion = "35.0.2";
          buildToolsVersions = [ buildToolsVersion ];
          platformVersions = [ "35" ];
          includeEmulator = false;
          includeSystemImages = false;
          includeSources = false;
          includeNDK = true;
          ndkVersions = [ ndkVersion ];
          cmakeVersions = [ cmakeVersion ];
        };

        androidSdk = androidComposition.androidsdk;
        sdkRoot = "${androidSdk}/libexec/android-sdk";
      in
      {
        devShells.default = pkgs.mkShell {
          packages = [
            androidSdk
            pkgs.jdk17
            # Gradle itself comes from ./gradlew (pinned to 8.12 to match AGP 8.7),
            # and Kotlin from the Gradle plugin — neither belongs in the shell.
            #
            # NOTE: deliberately NOT pkgs.android-tools. adb ships in the SDK's
            # platform-tools, and having two adb builds available is a real hazard:
            # whichever one starts the server owns it, and a version-mismatched
            # client then reports a healthy device as "offline". Arch's
            # android-tools adb also lacks mDNS, which Wireless debugging pairing
            # requires. One adb only — see docs/dev-environment.md.
            # MAVLink-side tooling for testing the bridge from the laptop, and
            # the offline vision tooling under tools/ (lensfit, tagcorners).
            #
            # One interpreter with all three packages, not pkgs.python3 plus
            # loose pkgs.python3Packages.*: those put the libraries in the store
            # but not on the interpreter's path, so `import numpy` fails from a
            # shell that appears to contain numpy. withPackages builds the env.
            #
            # opencv4 rather than opencv4.override { enableContrib = true; }:
            # ArUco moved from contrib into the main objdetect module in OpenCV
            # 4.7, so cv2.aruco.DICT_APRILTAG_36h11 and CORNER_REFINE_APRILTAG
            # are both present in the plain build. Verified 2026-07-28 against
            # this pin (4.13.0) — docs/apriltag-landing-recording.md §6.1 had
            # this marked "read, not verified".
            (pkgs.python3.withPackages (ps: [
              ps.pymavlink
              ps.numpy
              ps.opencv4
            ]))
            pkgs.ffmpeg # inspect the RTSP/RTP video path; decode datasets for tools/lensfit
          ];

          JAVA_HOME = "${pkgs.jdk17}";
          ANDROID_HOME = sdkRoot;
          ANDROID_SDK_ROOT = sdkRoot;

          # Gradle otherwise tries to fetch its own aapt2 binary, which will not
          # run against the Nix store's glibc.
          GRADLE_OPTS = "-Dorg.gradle.project.android.aapt2FromMavenOverride=${sdkRoot}/build-tools/${buildToolsVersion}/aapt2";

          # Absolute path to the one true adb, so scripts need not rely on PATH
          # order winning against a host-installed adb.
          ADB = "${sdkRoot}/platform-tools/adb";

          # AGP finds the NDK by ndkVersion under $SDK/ndk/<version>, which is
          # exactly the layout androidenv produces, so these are belt and braces
          # for anything invoking cmake outside Gradle.
          ANDROID_NDK_ROOT = "${sdkRoot}/ndk/${ndkVersion}";
          ANDROID_NDK_HOME = "${sdkRoot}/ndk/${ndkVersion}";

          shellHook = ''
            # platform-tools first: this must beat any host adb (e.g. Arch's
            # android-tools), or the server/client version mismatch returns
            # spurious "device offline".
            export PATH="${sdkRoot}/platform-tools:$JAVA_HOME/bin:$PATH"
            echo "mini4pro dev shell — jdk17, android sdk 35 (build-tools ${buildToolsVersion}); build with android/gradlew"
            echo "adb: $(command -v adb) ($(adb version 2>/dev/null | sed -n 2p))"
          '';
        };
      });
}
