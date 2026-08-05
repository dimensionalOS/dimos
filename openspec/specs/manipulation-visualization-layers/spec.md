# Manipulation Visualization Layers

## Purpose

Define display-only manipulation visualization layers, their backend-neutral
geometry models, Viser lifecycle and rendering behavior, and the standalone
banana grasp-proposal visualization workflow.

## Requirements

### Requirement: Display-only visualization layer interface
The manipulation visualization interface SHALL accept complete `VisualizationLayer` replacements through `set_layer` and SHALL clear a layer's rendered contents through `clear_layer`. These operations MUST NOT mutate the planning world, create collision geometry, or affect collision and motion-planning results.

#### Scenario: Visual layer is published
- **WHEN** a caller submits a valid visualization layer
- **THEN** the visualization backend accepts the layer for display without invoking planning-world mutation

#### Scenario: Layer is cleared
- **WHEN** a caller clears a registered visualization layer
- **THEN** its visual elements disappear while its registration and visibility preference remain

#### Scenario: Visualization and collision representations share a source object
- **WHEN** a display-only element and a planning obstacle describe the same physical object
- **THEN** adding, replacing, clearing, or failing to render the visual element does not add, replace, remove, or otherwise modify the planning obstacle

### Requirement: Backend-neutral layer and element models
A `VisualizationLayer` SHALL have a nonempty stable hierarchical ID, one nonempty coordinate frame, a first-registration default visibility value, and elements with IDs unique within that layer. The initial supported element types SHALL be point clouds and line sets represented by owned NumPy array snapshots rather than backend handles or robotics-domain messages.

#### Scenario: Valid point cloud
- **WHEN** a point-cloud element contains finite `N x 3` points and either no colors or matching RGB colors
- **THEN** the model snapshots and accepts the element

#### Scenario: Valid line set
- **WHEN** a line-set element contains finite `N x 3` vertices and `M x 2` in-range vertex indices
- **THEN** the model snapshots and accepts the element

#### Scenario: Caller mutates source arrays
- **WHEN** a caller changes a source NumPy array after constructing or submitting an element
- **THEN** the accepted visual snapshot remains unchanged

#### Scenario: Invalid geometry
- **WHEN** an element has non-finite coordinates, inconsistent color counts, invalid edge indices, or a non-positive explicit size
- **THEN** the layer is rejected before any current rendered generation is modified

#### Scenario: Duplicate element identity
- **WHEN** one layer contains two elements with the same ID
- **THEN** the layer is rejected

### Requirement: Complete and atomic layer replacement
Publishing a layer with an existing ID SHALL replace its complete contents. Viser MUST expose either the previous valid generation or the complete new generation and MUST NOT expose a partial mixture when validation or rendering fails.

#### Scenario: Successful replacement
- **WHEN** every element in a replacement layer renders successfully
- **THEN** Viser displays the complete new generation and removes the previous generation

#### Scenario: Failed replacement
- **WHEN** any element in a replacement layer fails validation or rendering
- **THEN** Viser removes partial replacement handles, retains the complete previous valid generation, and reports a visualization warning

#### Scenario: Empty replacement
- **WHEN** a registered layer is cleared
- **THEN** Viser removes its element handles but retains the layer entry and viewer state

### Requirement: Best-effort latest-wins updates
Layer submission SHALL NOT wait for Viser scene rendering. Viser SHALL retain at most the newest pending operation for each layer, SHALL order replacement and clear operations for the same layer, and SHALL contain visualization failures without failing a caller's robotics operation.

#### Scenario: Replacements arrive faster than rendering
- **WHEN** multiple replacements for one layer arrive before its pending update renders
- **THEN** Viser may skip intermediate replacements but eventually renders the newest layer value

#### Scenario: Clear supersedes replacement
- **WHEN** a clear operation supersedes an older pending replacement for the same layer
- **THEN** the older replacement does not reappear after the clear

#### Scenario: Renderer is unavailable
- **WHEN** Viser is disconnected, closed, or raises while processing an update
- **THEN** the caller is not failed or blocked on scene rendering and the failure is logged as a visualization warning

### Requirement: Hierarchical layer selector
Viser SHALL derive a hierarchical selector from slash-separated layer IDs. Leaf layers SHALL be independently toggleable, group controls SHALL toggle all descendants, and replacing or clearing contents SHALL preserve viewer-owned visibility.

#### Scenario: Hierarchical registration
- **WHEN** layers `grasp/object-cloud` and `grasp/proposals` are first submitted
- **THEN** Viser displays them as independently controlled children of a `Grasp` group

#### Scenario: Hidden layer is replaced
- **WHEN** a user hides a layer and its producer publishes replacement contents
- **THEN** Viser updates the contents while leaving the layer hidden

#### Scenario: Parent group is toggled
- **WHEN** a user changes a group checkbox
- **THEN** Viser applies that visibility to every descendant leaf layer

#### Scenario: Multiple clients view the selector
- **WHEN** more than one Viser client is connected
- **THEN** they observe the same server-global layer visibility state

### Requirement: Viser point-cloud rendering
Viser SHALL render point-cloud elements with source RGB when present and cyan otherwise. It SHALL use a default point size of 5 mm unless overridden and SHALL cap display at 20,000 points per element without modifying the submitted element.

#### Scenario: Colored cloud under the cap
- **WHEN** a point-cloud element provides RGB colors and at most 20,000 points
- **THEN** Viser renders every point with its matching source color

#### Scenario: Uncolored cloud
- **WHEN** a point-cloud element provides no colors
- **THEN** Viser renders it using the cyan fallback

#### Scenario: Cloud exceeds the render cap
- **WHEN** a point-cloud element contains more than 20,000 points
- **THEN** Viser deterministically samples at most 20,000 points, applies the same sample to colors, and leaves the source element unchanged

### Requirement: Viser line-set rendering
Viser SHALL render generic line-set elements from vertices and indexed edges, preserving supplied uniform or per-line colors and explicit positive line width.

#### Scenario: Gripper wireframe line set
- **WHEN** a line set describes a parallel-jaw gripper proposal
- **THEN** Viser renders every indexed segment at the supplied pose and appearance

#### Scenario: Multiple proposal elements
- **WHEN** a layer contains line sets with unique IDs for multiple grasp ranks
- **THEN** Viser renders each proposal independently within the same layer lifecycle

### Requirement: Visualization-focused verification
Automated verification SHALL cover layer-model validation and ownership, Viser rendering and lifecycle, hierarchical visibility, atomic failure behavior, and latest-wins ordering without requiring a browser, robot, planner, or live perception system.

#### Scenario: Automated test environment
- **WHEN** the visualization test suite runs with fake Viser scene handles
- **THEN** it validates the specified behavior without launching a browser or executing robot motion
