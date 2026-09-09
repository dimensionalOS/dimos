# DIOS provenance

The workspace creator is derived from installation code in
`dimensionalOS/dios` at commit `f19eda9506352166404507f1a564c493ff154a26`.
The import is selective: the app manager, desktop/frontend examples, services,
self-updater, Docker/Nix packaging, publishing scripts, and demo captures are
excluded. There is no vendored copy of the complete DIOS repository.

The Rust workspace and lockfile are independent of DimOS native modules.
The bootstrap and release artifacts belong to the DimOS release process.
