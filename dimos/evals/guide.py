# Copyright 2026 Dimensional Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""The Python API guide an eval agent gets as DIMOS.md.

In an eval, dimOS is already running when the agent starts, so the guide covers
connecting to it. ``python_guide`` adds the sections for the robot's kinds. When the
API changes, update these and the matching docs together: docs/usage/python-api.md
and docs/capabilities/manipulation/python_api.md.
"""

from typing import Literal

RobotGuide = Literal["manipulation", "navigation"]

PYTHON_API_GUIDE = """\
# Driving the robot from Python

dimOS is already running and the robot is up. Connect to it from a Python script with
`Dimos.connect()`. Do not start your own dimOS, with `Dimos()` or `dimos run`: there can
be only one per machine.

```python
from dimos import Dimos

app = Dimos.connect()
try:
    print(app)             # the running modules
    print(app.skills)      # every skill, with its arguments
finally:
    app.stop()             # closes this connection; dimOS and the robot keep running
```

Never run `dimos stop`: it ends the whole system, the robot included.

Don't name a script after a standard-library module (`inspect.py`, `types.py`, `test.py`):
Python imports the script's folder first, so the script shadows the real module and
`import dimos` fails.

## Skills and RPCs

Call skills by name, with keyword arguments. Which skills exist depends on the robot;
`print(app.skills)` lists them. For example, with the `observe` skill:

```python
image = app.skills.observe()
```

Modules also have `@rpc` methods. Reach a module by its class name and call them, for
example on an arm:

```python
state = app.ManipulationModule.get_state()
```

## Finding modules and RPCs

```python
app.list_modules()                       # every running module instance
module = app.get_module("ManipulationModule")  # by instance or class name
app.list_rpcs()                          # every RPC; app.list_rpcs(module) for one module
app.describe(module)                     # its RPCs, arguments and docs
app.describe("ManipulationModule.move_linear")
help(module.move_linear)                 # signature and docstring
```

A class name works when only one instance has that class; otherwise the error lists
the exact instance names. Each call reads the live system, so modules loaded later
show up too.

To find the module that offers a typed capability, pass a Spec Protocol. Exactly one
module must match: no match raises `LookupError`, more than one raises `ValueError`.

```python
from dimos.manipulation.manipulation_spec import ManipulationSpec

motion = app.find_module_by_spec(ManipulationSpec)
```

## Streams

`app.list_streams()` lists every stream, with its message type and module.
`peek_stream(name, timeout)` returns the next message, or `None` if none arrives in time:

```python
image = app.peek_stream("color_image", 2.0)
if image is not None:
    image.save("view.jpg")
```

To get every message, subscribe through the module that publishes the stream:

```python
unsubscribe = app.get_module("Detection3DModule").get_stream("detections").subscribe(print)
...
unsubscribe()
```

The client must have the stream's message types installed.
"""

MANIPULATION_GUIDE = """\
## Robot arms

For arm motion, use the `Arm` SDK. It picks the arm's planning group and blocks until
each move finishes, raising `MotionError` when one fails:

```python
from dimos.manipulation.sdk import Arm

arm = Arm.from_app(app)
print(arm.info)        # joint names and capabilities
print(arm.pose())      # the tool pose in the world frame
help(arm)              # every method
```
"""

NAVIGATION_GUIDE = """\
## Mobile robots

Mobile robots take goals through skills. On the Go2, `move_to` blocks until the robot
arrives or gives up; `app.describe("UnitreeSkillContainer.move_to")` explains its world
and relative coordinates. Habitat's `go_to` returns at once, so poll the pose until the robot stops,
and `stop_navigation()` cancels the goal:

```python
app.skills.go_to(x=4.0, y=1.5)
pose = app.peek_stream("odometry", 2.0)  # repeat until x, y stop changing
```
"""


def python_guide(*robots: RobotGuide) -> str:
    """The general guide, followed by the section for each kind of robot, in order."""
    sections = {"manipulation": MANIPULATION_GUIDE, "navigation": NAVIGATION_GUIDE}
    return "\n".join([PYTHON_API_GUIDE, *(sections[robot] for robot in robots)])
