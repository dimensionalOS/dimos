# Configuration API

Field types and defaults come from Python annotations. Field descriptions explain
their meaning and units. Configuration models use `Attributes:` sections.

## Gripper control task

::: dimos.control.tasks.gripper_task.gripper_task.GripperControlTaskConfig
    options:
      merge_init_into_class: true
      members: false

## GraspGenX gripper sweep volume

::: dimos.manipulation.grasping.grasp_gen_x.module.SweepVolumeGripperConfig
    options:
      show_if_no_docstring: true
      members:
        - extents_open
        - offset_open
        - extents_half_open
        - offset_half_open
        - fingertip_depth
        - family
