# Flujo de modulos G1

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Mostrar como se encadenan los modulos mas importantes de G1 real.

## Explicacion conceptual desde cero

DimOS no tiene un monolito "robot". Tiene subgrafos. Un subgrafo produce imagenes, otro produce memoria, otro produce tools, otro produce navegacion. El problema de integracion es que algunos subgrafos no comparten la misma interfaz.

## Archivos, clases y funciones relevantes

- Base: `dimos/robot/unitree/g1/blueprints/primitive/uintree_g1_primitive_no_nav.py`
- Conexion: `dimos/robot/unitree/g1/connection.py`
- Percepcion/memoria: `dimos/robot/unitree/g1/blueprints/perceptive/_perception_and_memory.py`
- Agentic: `dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py`
- Nav onboard: `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- Detection: `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_detection.py`

## Flujo de datos

### Base real

```text
Webcam -> CameraModule -> color_image / camera_info / TF camera
LCM /lidar -> pointcloud
LCM /map -> global_pointcloud
VoxelGridMapper + CostMapper + WavefrontFrontierExplorer
G1Connection -> UnitreeWebRTCConnection -> robot
```

### Agentic

```text
/human_input -> McpClient -> GPT-4o -> tool call
tool call -> McpServer -> RPC -> skill module
```

### Navegacion onboard

```text
Livox Mid-360 -> FastLio2 -> registered_scan + odometry + global_map_fastlio
registered_scan + odometry -> PGO -> corrected_odometry + global_map_pgo
registered_scan + corrected_odometry -> TerrainAnalysis / TerrainMapExt
goal -> SimplePlanner/FarPlanner -> way_point
way_point + local terrain -> LocalPlanner -> path/effective_cmd_vel
path + corrected_odometry -> PathFollower -> nav_cmd_vel
nav_cmd_vel -> MovementManager -> cmd_vel
cmd_vel -> G1HighLevelDdsSdk -> Unitree SDK2/DDS -> G1
```

### Detection

```text
color_image -> YOLO person detector -> detections_2d
detections_2d + pointcloud + TF -> Detection3DModule -> object pointclouds
Detection3DModule -> ObjectDBModule -> object tracks temporal
detections_2d + image -> PersonTracker -> target pose
```

## Inputs y outputs

- Inputs sensores: webcam, lidar, odometry.
- Inputs semanticos: texto, queries, tags.
- Outputs accion: `cmd_vel`, Unitree API requests.
- Outputs memoria: ChromaDB, visual memory, tagged locations.
- Outputs debug: Rerun entities, LCM topics, logs.

## Como se conecta con otras partes

Los flujos base y agentic estan juntos en `unitree-g1-full`. Los flujos nav y detection existen, pero no estan unidos ahi.

## Que funciona hoy en G1 real

La modularidad permite correr stacks por separado y reutilizar topics como `/cmd_vel`, `/lidar`, `/map`, `/color_image`.

## Que parece incompleto o separado

Falta un puente formal entre:

- `navigate_with_text` y nav onboard.
- detecciones 3D y memoria espacial.
- STT y agente G1 real.
- seguridad de goals y control directo.

## Notas para mi proyecto/paper

Este flujo puede convertirse en el diagrama de arquitectura del paper, con colores por capa: humano, agente, percepcion, memoria, planificacion, control y safety.
