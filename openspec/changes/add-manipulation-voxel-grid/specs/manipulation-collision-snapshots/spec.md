# Capture-aligned manipulation collision mapping

## ADDED Requirements

### Requirement: capture-aligned sensor pose

For every accepted camera cloud, the system SHALL publish at most one sensor
Odometry in the configured map frame with the same timestamp and child frame as
the cloud. Missing capture-time TF SHALL NOT fall back to a latest pose.

#### Scenario: cloud pose is available

- **WHEN** a camera cloud has a capture-time TF estimate within tolerance
- **THEN** one Odometry is published with the cloud timestamp and frame

#### Scenario: cloud pose is unavailable

- **WHEN** capture-time TF remains unavailable after the bounded wait
- **THEN** no Odometry is published for that cloud

### Requirement: bounded generic cloud-pose synchronization

The native mapper SHALL wait for out-of-order Odometry, match only compatible
parent/child frames within the configured tolerance, and bound pending clouds
by watermark expiry and capacity. Navigation SHALL continue to process clouds
without any clear-mask producer.

#### Scenario: Odometry arrives after its cloud

- **WHEN** a compatible same-stamp Odometry arrives after a pending cloud
- **THEN** the cloud is registered once using that Odometry

#### Scenario: navigation has no clear-mask producer

- **WHEN** a navigation cloud and compatible Odometry arrive without a mask
- **THEN** the cloud is processed normally

### Requirement: model-derived self exclusion

The filter SHALL exclude padded collision geometry for every robot and gripper
link at capture time. Missing required link TF SHALL drop the whole capture.
Nearby points outside the padded model SHALL be retained.

#### Scenario: robot and environment are both visible

- **WHEN** a cloud contains modeled robot points and external points
- **THEN** robot points are removed and external points are retained

#### Scenario: a required link pose is absent

- **WHEN** any modeled collision link lacks capture-time TF
- **THEN** neither the filtered cloud nor its clear mask is published

### Requirement: optional independent occupancy cleanup

The filter SHALL publish previous-plus-current robot volumes as metric samples
in the map frame. The mapper SHALL apply valid masks monotonically without
requiring them for cloud integration. Late masks MAY create temporary eventual
consistency but SHALL NOT permanently block mapping.

#### Scenario: the robot vacates a mapped volume

- **WHEN** a newer valid mask covers the previous and current robot volumes
- **THEN** the mapper clears those quantized voxels independently of clouds

#### Scenario: an older mask arrives late

- **WHEN** a mask timestamp is older than the last applied mask
- **THEN** the mapper rejects it without changing occupancy

### Requirement: stable typed planning obstacle

Complete world maps SHALL be submitted as typed OCTREE `Obstacle` values using
the existing add/update/remove RPC lifecycle and the stable name
`mapping/global-voxel-map`. Non-empty maps SHALL update first and add if absent;
empty maps SHALL remove idempotently. Only backend-accepted geometry SHALL be
visualized. Mapper silence SHALL retain the last accepted obstacle.

#### Scenario: a non-empty map is received

- **WHEN** the bridge receives a finite non-empty world-frame global map
- **THEN** it updates or adds one OCTREE named `mapping/global-voxel-map`

#### Scenario: an empty map is received

- **WHEN** the bridge receives an empty global map
- **THEN** it removes the stable obstacle idempotently

### Requirement: backend and payload validation

The bridge SHALL reject a non-RoboPlan backend, frame mismatches, non-finite
points, and non-positive resolution. Collision registration SHALL retain all
voxels; Viser MAY cap only its rendered copy.

#### Scenario: an unsupported backend is configured

- **WHEN** the bridge is configured for a non-RoboPlan world
- **THEN** configuration fails before runtime map processing

#### Scenario: visualization caps a large OCTREE

- **WHEN** an accepted OCTREE exceeds the Viser point cap
- **THEN** only rendering is sampled and collision geometry remains complete
