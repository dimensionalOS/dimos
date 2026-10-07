# Ray Tracing Mapping

Using ray tracing for mapping allows moving objects to be cleared while building up a global map.

The ray tracer takes point clouds from a lidar and uses the transform for the sensor to register the point cloud in the correct frame. For each point in a new point cloud, a ray is traced from the sensor origin to the point. Any voxel already in the global map that the ray passes through is a candidate to be cleared from the map.

## How to run

```bash
dimos run mid360-pointlio-ray-trace --lidar-ip=<mid360-ip>
```

This blueprint runs the Mid-360 driver, Point-LIO, and the ray tracer, and visualizes with Rerun.

## Input/Output

### Inputs

| Stream  | Description                             |
|---------|-----------------------------------------|
| `lidar` | Lidar scans in sensor frame.            |
| `tf`    | Transform tree to register lidar scans. |

### Outputs

| Stream       | Description                                                                                                                                                           |
|--------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `global_map` | Full point cloud of the global map.                                                                                                                                   |
| `local_map`  | Cylindrical subsection of the global map with height of the 5th to 95th percentile z coordinate and radius of the 95th percentile xy distance from the sensor origin. |

Most downstream consumers can be built to use the `local_map`, allowing for incremental work only on parts of the map that possibly have changed.

## Voxel representation

Each voxel contains the following data:

| Field          | Description                                                                                                                                                                                     |
|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `health`       | Occupancy score. Each lidar scan with a point in this voxel increments health and each scan that traces through it decrements health. Only voxels with positive health are considered occupied. |
| `support`      | Number of healthy neighbors.                                                                                                                                                                    |
| `num_pts`      | Count of actual lidar hits binned to this voxel.                                                                                                                                                |
| `sum`          | Sum of all lidar points landing in this voxel, relative to the center of the voxel.                                                                                                             |
| `m2`           | Sum of the outer products of the points. Used for point covariance and normal fitting.                                                                                                          |
| `normal`       | The latest normal vector fit from the points binned to this voxel and its neighbors.                                                                                                            |
| `next_fit_pts` | The number of points required before the normal vector will be refit on this voxel.                                                                                                             |
| `fine`         | A bitmask representing which micro-voxels are occupied.                                                                                                                                         |

## Configuration

Depending on your requirements, you may want to tune the behavior of the ray tracer. Responsiveness, surface quality, and false positives/negatives can all be adjusted through the configuration.

The following config values are available:

### Map resolution

| Field          | Description                                                                                    |
|----------------|------------------------------------------------------------------------------------------------|
| `voxel_size`   | The size of voxels, in m.                                                                      |
| `fine_divisor` | How many micro-voxels per edge in each voxel. Example: 3 results in 27 micro-voxels per voxel. |

### Clearing properties

| Field          | Description                                                                                                                                                 |
|----------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `min_health`   | Minimum health of a voxel before it is deleted. Voxels spawn one above this health.                                                                         |
| `max_health`   | Maximum health of a voxel before it stops accumulating health.                                                                                              |
| `shadow_depth` | How far to continue tracing rays past their points, in m. Helps clear objects following or moving toward the sensor.                                        |
| `grace_depth`  | Voxels within this distance from points, in m, are spared from clearing. Helps with voxel clipping on floors and surfaces.                                  |
| `graze_cos`    | Spare voxels when the absolute value of the ray dot normal is below this value. Higher values clear only direct hits, lower values also clear grazing hits. |

### Performance

| Field           | Description                                                                                              |
|-----------------|----------------------------------------------------------------------------------------------------------|
| `max_range`     | Maximum distance each ray will be traced, in m. Points beyond it are ignored. Used to limit computation. |
| `ray_subsample` | Trace every Nth ray. Used to limit computation.                                                          |
