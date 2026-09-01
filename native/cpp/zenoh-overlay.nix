# nixpkgs' zenoh-c cannot be consumed as it ships, so every C++ native module
# flake applies this. Both fixes belong upstream.
final: prev: {
  zenoh-c = prev.zenoh-c.overrideAttrs (old: {
    # Upstream defaults this off in a source build, and nixpkgs does not set
    # it. The zenoh transport needs Reliability and the session link list,
    # which both sit behind it.
    cmakeFlags = (old.cmakeFlags or [ ]) ++ [ "-DZENOHC_BUILD_WITH_UNSTABLE_API=ON" ];

    postFixup = (old.postFixup or "") + ''
      # zenohcConfig.cmake resolves its prefix to the dev output, but the
      # libraries are only in the main one, so every find_package points at a
      # path that does not exist.
      ln -s "$out"/lib/libzenohc.* "$dev/lib/"
    '' + prev.lib.optionalString prev.stdenv.hostPlatform.isDarwin ''
      # The dylib still carries the build sandbox as its install name, which
      # anything linking against it would then record and fail to load.
      install_name_tool -id "$out/lib/libzenohc.dylib" "$out/lib/libzenohc.dylib"
    '';
  });
}
