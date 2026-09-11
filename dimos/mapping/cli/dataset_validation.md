Dataset Validation

```sh
dimos mem summary recording_go2_mid360_2026-05-29_4-45pm-PST.db
```

Prints a table of the streams in the database: per-stream item count, average
Hz, start time (UTC), duration, and stored size, plus a total row. You can
replay messages in rerun:

```sh
dimos map replay recording_go2_mid360_2026-05-29_4-45pm-PST.db --duration 60
```


```sh
dimos map global recording_go2_mid360_2026-05-29_4-45pm-PST_corrected --voxel 0.1 --lidar fastlio_lidar
```

validates that livox lidar observations have correct poses associated coming from livox odometry

![output](assets/fastlio_lidar.png)


```sh
dimos map global recording_go2_mid360_2026-05-29_4-45pm-PST_corrected --voxel 0.1 --lidar lidar
```

validates that go2 lidar observations have correct poses associated coming from go2 odometry

![output](assets/go2_lidar.png)


```
dimos map global recording_go2_mid360_2026-05-29_4-45pm-PST_corrected --voxel 0.1 --lidar lidar --markers --pgo
```

validates that camera image has correct pose associated, `--pgo` is not needed but you can

![output](assets/markers_go2.png)


```sh
dimos map global recording_go2_mid360_2026-05-29_4-45pm-PST_corrected --voxel 0.1 --lidar fastlio_lidar --markers --image-pose fastlio_odometry --duration 120
```

Validates that transform between fastlio_odom and camera image is correct (below it isn't, we detect markers at the angle - mabye fastlio_odometry pose should be flat representing base_link not lidar mounting style)

![output](assets/markers_fastlio.png)
