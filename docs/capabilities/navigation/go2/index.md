# Go2 mapping

## Real time mapping

The default blueprint maps dynamic environments in real time.

`dimos --robot-ip ... run unitree-go2`

Loop closure corrects odometry drift, and simulators have none, so it is off under `--simulation`. `--pgovoxelmapper.pgo.loop-closure true` turns it back on there.

## Saving maps between runs

`dimos --robot-ip ... run unitree-go2-map`

The map blueprint saves the map between runs, relocalizes against it, and updates it as the robot explores more or the environment changes. The map is kept in `map_go2.map.db`.

To relocalize against a prerecorded map without updating it, give the default blueprint a map file:

`dimos --robot-ip ... run unitree-go2 --map-file=<map>` (loads `<map>.map.db`)
