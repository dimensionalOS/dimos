# Navigation from Python

Mobile robots take goals through skills, so a script drives them with the same calls
an agent makes. Connect to a running blueprint as in the
[Python API guide](/docs/usage/python-api.md):

```python skip
from dimos import Dimos

app = Dimos.connect()
print(app.skills)   # the robot's skills, with their arguments
```

## Go2: `move_to`

On the Go2, `move_to` blocks until the robot arrives or gives up. Its docstring
explains the world and relative coordinates it takes:

```python skip
print(app.describe("UnitreeSkillContainer.move_to"))
```

## Habitat and the MLS planner: `go_to`

`go_to` returns at once and the robot keeps driving, so poll its pose until it stops.
`stop_navigation()` cancels the goal:

```python skip
app.skills.go_to(x=4.0, y=1.5)
pose = app.peek_stream("odometry", 2.0)  # repeat until x, y stop changing
app.skills.stop_navigation()
```

`app.list_streams()` lists each robot's pose and camera streams.
