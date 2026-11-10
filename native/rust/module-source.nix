# Copyright 2026 Dimensional Inc.
# SPDX-License-Identifier: Apache-2.0

# Stage standalone module crates with repository-relative generated SDK inputs.
{ pkgs, projectRoot, modulePath, moduleSource
, messageSource ? (import (projectRoot + "/native/message-source.nix") { inherit pkgs; }).source
}:
assert builtins.match "[a-zA-Z0-9_./-]+" modulePath != null && modulePath != "" && !(pkgs.lib.hasPrefix "/" modulePath) && !(pkgs.lib.hasInfix ".." modulePath);
let
  sdk = pkgs.lib.cleanSourceWith {
    src = projectRoot + "/native/rust";
    filter = path: type: !(builtins.elem (baseNameOf path) [ "target" "result" ".git" "__pycache__" ]);
  };
in pkgs.runCommand "dimos-native-module-source" { } ''
  mkdir -p $out/native/rust $out/build/message-source
  cp -r ${sdk}/. $out/native/rust/
  cp -r ${messageSource}/rust $out/build/message-source/rust
  mkdir -p "$out/${modulePath}"
  cp -r ${moduleSource}/. "$out/${modulePath}/"
''
