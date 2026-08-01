# Resolve configurable Module IO at build/load time

DimOS will support configuration-resolved Module IO: a Module may derive its coordinator-visible typed stream inputs and outputs from its final validated config before blueprint wiring. The default `Module.io_contract(config)` remains annotation-derived, so existing annotated `In`/`Out` modules keep their behavior. A custom `io_contract` override is a complete replacement for annotation-derived IO.

Configuration-resolved streams are fixed for a running module. They are stored in the Module's input/output registries and exposed through `self.inputs` and `self.outputs`; static annotated streams keep attribute access for backward compatibility. Blueprint remappings apply externally to resolved stream names and do not rename module-local keys.

This avoids runtime mutable ports while allowing policy, backend, task, and hardware configuration to shape the graph honestly. Runtime hot-add/remove streams, stream groups, separate wire names, and coordinator/worker contract fingerprinting are deferred until a concrete need appears.
