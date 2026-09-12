# Share policy execution and isolate inference dependencies

ABC and LeRobot share one observation and rollout controller, with inference
adapters in independently locked Python environments. Adapters convert their
checkpoint conventions into absolute joint targets in configured hardware
order; the existing control coordinator owns trajectory execution and hardware
limits. This keeps model dependency conflicts and checkpoint conventions out of
robot control while avoiding separate lifecycle implementations per backend.

ABC's released inference code is retained as a pinned source snapshot with its
licenses. Deployment uses its model, preprocessing, and CUDA helpers without
installing the upstream training and simulation stack. Updates must be checked
against the reference sampler and the backend's input/action contract.
