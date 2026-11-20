# M20 deployment

DimOS runs on the M20's three boards: GOS (`10.21.31.104`, where `dimos run m20` starts and the navigation runs),
AOS (`10.21.31.103`: the body IMU, the cameras and the motor bus) and NOS (`10.21.31.106`: the lidars' network and
dimSLAM). Each board runs a `dimos host` that takes the modules the blueprint places on it.

A laptop on the robot hotspot reaches GOS through AOS:

```bash
sudo ip route add 10.21.31.0/24 via 10.21.41.1
```

## Setup

1. Robot setup (clocks, the vendor services DimOS replaces, the ecan shim): `m20tool setup`.
2. The dimos Hosts, from the dimos checkout on GOS (re-run after changing the checkout, so the boards run the same
   revision):

   ```bash
   bash dimos/robot/deeprobotics/m20/onboard/install_dimos_host.sh gos aos nos
   ```

## Run

```bash
dimos run m20
```

`--choice M20Mode=sim` runs the same navigation on the simulated M20 instead (no robot needed). The real mode's own
choices (body filter, joint source, body IMU, cameras, where dimSLAM runs) are in
`dimos/robot/deeprobotics/m20/blueprints/m20.py`. Nothing in the real mode commands the legs.
