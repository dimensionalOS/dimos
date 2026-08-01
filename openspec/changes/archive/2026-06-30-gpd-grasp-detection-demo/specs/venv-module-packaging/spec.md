## ADDED Requirements

### Requirement: GPD grasp demo package exercises pointcloud-based grasp detection
The system SHALL include a package-local GPD grasp demo package that prepares the pinned GPD native binding and uses it for grasp detection from pointcloud inputs produced by existing DimOS perception modules, not only import probing.

#### Scenario: Demo package declares GPD grasp dependency closure
- **WHEN** the GPD grasp demo package is prepared as a Python project runtime
- **THEN** its project manifests declare the Python, DimOS, GPD, and native/Pixi dependency closure needed to run the GPD pointcloud-consuming grasp detector in the worker runtime

#### Scenario: GPD import remains lazy
- **WHEN** the coordinator imports the GPD grasp demo package for blueprint construction
- **THEN** package import succeeds without importing `gpd.core` until worker-side grasp generation or an explicit worker-side probe runs

#### Scenario: Demo package can run a real GPD generation path
- **WHEN** the GPD grasp demo package runtime is prepared and the generator receives a valid pointcloud
- **THEN** the worker process can import the pinned GPD binding and execute the adapter path used for grasp generation
