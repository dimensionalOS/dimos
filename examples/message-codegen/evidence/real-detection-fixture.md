# Real-scene detection fixture after CDR cutover

The replacement `unitree_go2_detection_cdr` fixture contains the five moments
used by the existing detection and object-history tests (seeks 10, 12, 14, 16,
and 18 seconds). It contains generated Image, PointCloud2 and PoseStamped CDR,
complete `ros2msg` schemas, and a manifest of source paths, source SHA-256 hashes,
recording times, source sec/nanosec stamps and encoded payload hashes.

Source: project LFS `data/.lfs/unitree_go2_lidar_corrected.tar.gz`, endpoint
`https://lfs.dimensionalos.com/dimensionalOS/dimos`, SHA-256
`51a817f2b5664c9e2f2856293db242e030f0edce276e21da0edc2821d947aad2`.
The original archive is 1,212,727,745 bytes; safe extraction produced
3,776,299,125 regular-file bytes across 3,490 archive entries. The source is
preserved. The replacement archive is 5,823,304 bytes, SHA-256
`966a7b4d542a58a7891b7c0fc4951e3ea3975c56f52cd99b0a5f0541198d2d40`.

Selection exactly follows the historical fixture: choose lidar by nearest
recording-envelope timestamp to first lidar plus seek; choose image and odometry
by nearest recording-envelope time to the selected lidar's source timestamp.
The archived odometry firmware timestamp is stale relative to its envelope;
retain it rather than silently repairing the recording. The test consumer maps
its odometry parent to `world`, matching the historical fixture and Go2's current
configured parent. It labels the image `camera_optical` as before.

The explicitly pinned archival exporter is
[`demo_export_detection_fixture.py`](/examples/message-codegen/demo_export_detection_fixture.py).
It is an offline fixture maintenance tool, not a runtime compatibility reader.
It accepts only the named archive hash and a small allowlist of archived state
records. It never installs old message classes. Runtime and pytest decode only
CDR. After safely extracting the verified archive, reproduce the payloads:

```bash
python examples/message-codegen/demo_export_detection_fixture.py \
  --archive data/.lfs/unitree_go2_lidar_corrected.tar.gz \
  --source /path/to/extracted/unitree_go2_lidar_corrected \
  --output /path/to/new/unitree_go2_detection_cdr
```

The exporter refuses an existing output directory. All 15 reproduced payloads
matched the replacement fixture byte for byte. Export checks preserve exact
image pixels and point coordinates within float32 wire precision. Source poses
and timestamps are copied explicitly to generated ROS-shaped fields.

Validation: **26 detection-type tests passed; one manual live-publish demo was
deselected**. This includes the original real-scene OBB/AABB extents, centers,
point-count thresholds, 2D round-trip identity/confidence, and five-frame object
history assertions, with their original numerical tolerances. CPU YOLO uses
repository-locked Ultralytics 8.4.14 and project `models_yolo/yolo11n.pt`.
The model archive SHA-256 is
`01796d5884cf29258820cf0e617bf834e9ffb63d8a4c7a54eea802e96fe6a818`.
No robot, external inference API, or hardware controller was run.

```bash
HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 YOLO_AUTOINSTALL=false \
CUDA_VISIBLE_DEVICES='' python -m pytest -o addopts='' --import-mode=importlib \
  dimos/perception/detection/type -k 'not test_guess_projection' -q
```

The same audit found G1 viewer callbacks still calling rich cloud and stamped
point methods. They now use the external XYZ helper and `PointStamped.point`;
three offline Rerun geometry tests verify coordinates, height colors, lifted
waypoints/goals and non-finite rejection. No live G1 stack was started.
