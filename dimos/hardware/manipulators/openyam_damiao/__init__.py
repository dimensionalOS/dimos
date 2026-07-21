# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.

"""Damiao hardware adapter for the six-axis OpenYAM arm."""

__all__ = ["OpenYAMDamiaoAdapter", "OpenYamDamiaoAdapter"]


def __getattr__(name: str) -> object:
    if name in __all__:
        from dimos.hardware.manipulators.openyam_damiao.adapter import (
            OpenYAMDamiaoAdapter,
            OpenYamDamiaoAdapter,
        )

        return {"OpenYAMDamiaoAdapter": OpenYAMDamiaoAdapter,
                "OpenYamDamiaoAdapter": OpenYamDamiaoAdapter}[name]
    raise AttributeError(name)
