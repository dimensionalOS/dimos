# nixpkgs' zenoh-c cannot be consumed as it ships. All three fixes belong upstream.
final: prev: {
  zenoh-c = prev.zenoh-c.overrideAttrs (old: {
    # Off by default in a source build. The transport needs Reliability and the
    # session link list, which both sit behind it.
    cmakeFlags = (old.cmakeFlags or [ ]) ++ [ "-DZENOHC_BUILD_WITH_UNSTABLE_API=ON" ];

    postFixup = (old.postFixup or "") + ''
      # zenohcConfig.cmake resolves its prefix to the dev output, but the
      # libraries are only in the main one.
      ln -s "$out"/lib/libzenohc.* "$dev/lib/"
    '' + prev.lib.optionalString prev.stdenv.hostPlatform.isDarwin ''
      # The dylib still carries the build sandbox as its install name.
      install_name_tool -id "$out/lib/libzenohc.dylib" "$out/lib/libzenohc.dylib"
    '';
  });
}
