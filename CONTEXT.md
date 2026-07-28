# DimOS

DimOS composes and operates modules that communicate through streams and RPCs.

## Language

**Module handle**:
A host-side value representing one deployed module, whether the module is owned by the current coordinator or reached through a coordinator connection.
_Avoid_: Module object, remote module

**Remote module proxy**:
A names-only module handle used when the client can discover a deployed module but cannot import its Python class.
_Avoid_: Remote module
