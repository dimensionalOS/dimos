# R1 Pro mobile sim2 demo

`r1pro-sim2` runs a floating R1 Pro in MuJoCo through sim2, with a browser view
and controls for the mobile base, torso and both arms.

## Run

From the checkout containing this blueprint:

```bash
uv sync --frozen --extra sim --extra control --extra cpu --extra planning --group dev
source .venv/bin/activate
python -m dimos.robot.galaxea.r1pro.sim.model
dimos run r1pro-sim2 --daemon
```

The model preparation command downloads the existing pinned Galaxea asset on
first use and writes the simulation adaptation to the DimOS cache. Run it again
after changing the generator.

Open [the controls](http://127.0.0.1:8095). Forward/back, lateral motion and turns
are relative to the robot. Each drive click lasts at most three seconds; Stop
brakes sooner. Losing the controlling browser or command heartbeat also stops
driving. The torso and arm sliders command individual joint positions in radians.
Home arms and torso restores their zero pose. Use `dimos stop` to end the demo;
`dimos restart` restores the initial scene.

## Integration

The blueprint uses the existing shared robot model, SimModule, encoder sensor and
atomic ActionCommand interfaces. The controller consumes standard JointState
measurements at 50 Hz and emits position/velocity/gain commands for 24 motors.
Body-frame Twist commands become steering angles and rolling speeds for the three
wheel assemblies. The root pose comes from physics and is displayed from
WorldState. There is no root-pose override for driving.

The model generator preserves vendor frames, visuals and inertias. It replaces
wheel collision meshes with cylinders measured from those meshes and fixes the
four gripper joints because their vendor effort and travel limits are zero. Wheel
limits and servo gains are demo assumptions, not validated hardware parameters.
Existing R1 Pro structural collision exclusions are passed into the sim2 scene.

This first demo has a ground plane and Viser visualization. Gripper actuation,
Quest/VR retargeting, onboard camera streams and the Unreal office scene are not
wired into this blueprint. Extreme manual poses can still collide or tip the
robot; this is joint teleoperation, not collision-aware planning.

## Validation

On the development Mac, the deployed five-module blueprint advanced 15.056
simulation seconds in approximately 15.06 wall seconds (1.00x real time), with
zero late physics steps. Bounded live commands produced about 0.61 m forward
travel, 0.60 m lateral travel and 0.50 rad of turning; stopping held the base still.
Torso yaw reached 0.20 rad and both shoulder joints followed their targets.
This measures simulation progress, not browser rendering FPS.

The steering, command timeout, joint limits, collision exclusions and existing
physics tests pass. The blueprint registry, Ruff and targeted mypy checks pass.
SimModule can count rejected duplicate commands when its lockstep observation
retransmission overlaps the controller response; those retries did not interrupt
physics or cause deadline faults in the live check.
