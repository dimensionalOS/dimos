# dimos-build-config

Independent source-only packaging defaults and dynamic blueprint metadata. No runtime
DimOS dependency or package-code import. See [the authoring guide](../../docs/usage/packages.md).
The distribution is not published; use local wheels for validation.

Run tests with scikit-build-core, build, pytest and uv available and this provider
installed: `python -m pytest packages/dimos-build-config/src/dimos_build_config/test_config.py`.

The upstream configuration group contains hyphens. Its registration therefore uses
setuptools' standard dynamic entry-point file rather than a PEP 621 entry-point table,
whose schema rejects that spelling. There is no custom backend.
