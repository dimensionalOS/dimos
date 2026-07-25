# Manipulation

The manipulation context plans and carries out coordinated robot motion while keeping
direct actuator commands distinct from planned motion.

## Language

**Generated plan**:
The complete coordinated motion selected for one planned execution, including every
participating robot and its shared timing.
_Avoid_: Partial plan, per-robot plan

**Planned execution**:
The transactional dispatch and supervision of a generated motion plan across one or
more robot tasks.
_Avoid_: Execution, actuation

**Execution acceptance**:
Confirmation that every targeted robot task accepted its part of a generated motion
plan. It does not mean the physical motion completed.
_Avoid_: Execution completion, motion completion

**Fresh plan**:
A generated motion plan whose initial joint state still agrees with the latest
available robot state closely enough to dispatch safely.
_Avoid_: Valid plan, current plan

**Replacement execution**:
A planned execution that supersedes the previously accepted plan only after every
task from that plan is confirmed safe to replace.
_Avoid_: Concurrent execution, implicit preemption

**Direct actuation**:
A hardware command that is not part of a generated motion plan, such as opening or
closing a gripper.
_Avoid_: Planned execution
