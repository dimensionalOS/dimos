# Go2 mapping

## Real time mapping

The default blueprint maps dynamic environments in real time.

`dimos --robot-ip ... run unitree-go2`

## Relocalizing against a premap

To relocalize against a prerecorded map, give the blueprint a map file:

`dimos --robot-ip ... run unitree-go2 --localmaprelocalization.map-file=<map>` (loads `<map>.pc2.lcm`)
