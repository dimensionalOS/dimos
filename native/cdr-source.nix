# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Keep local Cargo dependencies at their repository-relative paths in the sandbox.
{ pkgs, paths }:
let
  root = ../.;
  shared = [
    "native/rust/dimos-module"
    "native/rust/dimos-module-macros"
    "native/rust/dimos-lcm-transport"
  ];
  included = paths ++ shared;
  source = builtins.path {
    name = "dimos-cdr-native-inputs";
    path = root;
    filter = path: type:
      let
        relative = pkgs.lib.removePrefix ((toString root) + "/") (toString path);
        components = pkgs.lib.splitString "/" relative;
        wanted = (toString path) == (toString root) || builtins.any (entry:
          relative == entry || pkgs.lib.hasPrefix (entry + "/") relative
          || (type == "directory" && pkgs.lib.hasPrefix (relative + "/") entry)
        ) included;
      in !(builtins.any (component: builtins.elem component
        [ ".git" "target" "build" "result" "__pycache__" ".venv" ]
      ) components) && wanted;
  };
  messages = import ./message-source.nix { inherit pkgs; };
in pkgs.runCommand "dimos-cdr-native-source" { } ''
  mkdir -p $out
  cp -r ${source}/. $out/
  chmod -R u+w $out
  mkdir -p $out/packages/dimos-generated/build/message-source
  cp -r ${messages.source}/rust $out/packages/dimos-generated/build/message-source/rust
''
