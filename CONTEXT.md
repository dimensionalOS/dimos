# DimOS Module Runtime

This context names the parts that let DimOS compose modules while their work runs in different processes or environments.

## Language

**Module**:
A typed, independently managed capability that participates in a Blueprint through streams and RPCs.
_Avoid_: Service, node

**Native Module**:
A Module whose computation runs in a managed external process while DimOS retains its composition and lifecycle identity.
_Avoid_: Foreign module, binary wrapper

**Python-Native Module**:
A Native Module whose concrete implementation runs in an isolated Python environment.
_Avoid_: External Python module, external-python deployment

**Contract**:
The host-visible Module class that defines a Python-Native Module's streams, configuration, RPCs, and skills.
_Avoid_: Declaration, interface module

**Runtime Subclass**:
The concrete subclass that implements a Contract inside its isolated Python environment.
_Avoid_: Worker class, external implementation

**Internal RPC Endpoint**:
The private identity through which a Contract reaches its Runtime Subclass without changing the Module's public identity.
_Avoid_: Public child RPC, socket bridge
