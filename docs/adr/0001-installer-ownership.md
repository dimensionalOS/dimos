# DimOS setup and runtime ownership

`dimup` is the persistent Python bootstrap and application creator. Its installation
logic is maintained in the DimOS repository, derived from DIOS. `dimos` belongs to
each SDK application's Python environment and owns runtime commands and diagnostics.

uv owns Python and application lockfiles. apt/Homebrew install host libraries and
compilers. Cargo and Nix retain their native build responsibilities. There is no
Pixi environment or second robot-management frontend in this installation path.

SDK source installations build their required assets and carry native build
inputs. Generated applications are editable packages with registered blueprints;
committed activation files and a uv lockfile reproduce them after cloning.
