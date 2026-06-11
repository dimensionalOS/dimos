# Mapa de archivos clave

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Dar un indice rapido de rutas reales para estudiar el G1 real en DimOS.

## Explicacion conceptual desde cero

El codigo relevante no esta en una sola carpeta. G1 vive en `dimos/robot/unitree/g1`, pero usa piezas generales de `dimos/core`, `dimos/agents`, `dimos/perception`, `dimos/navigation`, `dimos/hardware` y `dimos/visualization`.

## Archivos, clases y funciones relevantes

### Blueprints G1

- `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic.py`
- `dimos/robot/unitree/g1/blueprints/primitive/uintree_g1_primitive_no_nav.py`
- `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1.py`
- `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_shm.py`
- `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_detection.py`
- `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_agentic.py`
- `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_full.py`
- `dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py`

### Agente y MCP

- `dimos/agents/mcp/mcp_server.py`
- `dimos/agents/mcp/mcp_client.py`
- `dimos/agents/mcp/mcp_adapter.py`
- `dimos/agents/skills/navigation.py`
- `dimos/agents/skills/speak_skill.py`
- `dimos/robot/unitree/g1/skill_container.py`
- `dimos/robot/unitree/g1/system_prompt.py`

### Control G1 real

- `dimos/robot/unitree/g1/connection.py`
- `dimos/robot/unitree/connection.py`
- `dimos/robot/unitree/g1/effectors/high_level/dds_sdk.py`
- `dimos/robot/unitree/g1/effectors/high_level/webrtc.py`
- `dimos/robot/unitree/g1/wholebody_connection.py`
- `dimos/robot/unitree/g1/effectors/high_level/commands.py`

### Vision y memoria

- `dimos/hardware/sensors/camera/module.py`
- `dimos/hardware/sensors/camera/webcam.py`
- `dimos/perception/spatial_perception.py`
- `dimos/perception/object_tracker.py`
- `dimos/perception/detection/module2D.py`
- `dimos/perception/detection/module3D.py`
- `dimos/perception/detection/moduleDB.py`
- `dimos/perception/detection/person_tracker.py`

### Lidar, SLAM y navegacion

- `dimos/hardware/sensors/lidar/fastlio2/module.py`
- `dimos/navigation/nav_stack/main.py`
- `dimos/navigation/nav_stack/modules/pgo/pgo.py`
- `dimos/navigation/nav_stack/modules/terrain_analysis/terrain_analysis.py`
- `dimos/navigation/nav_stack/modules/terrain_map_ext/terrain_map_ext.py`
- `dimos/navigation/nav_stack/modules/simple_planner/simple_planner.py`
- `dimos/navigation/nav_stack/modules/local_planner/local_planner.py`
- `dimos/navigation/nav_stack/modules/path_follower/path_follower.py`
- `dimos/navigation/movement_manager/movement_manager.py`
- `dimos/navigation/navigation_spec.py`

### CLI, logs y visualizacion

- `dimos/robot/cli/dimos.py`
- `dimos/core/run_registry.py`
- `dimos/core/log_viewer.py`
- `dimos/visualization/vis_module.py`
- `dimos/visualization/rerun/bridge.py`

## Flujo de datos

```text
Blueprint G1 -> core/coordination -> modules
Agente -> agents/mcp + skills
Vision -> hardware/camera + perception
Lidar/nav -> hardware/lidar + navigation/nav_stack
Control -> robot/unitree/g1/connection + effectors/high_level
Debug -> cli + visualization + logs
```

## Inputs y outputs

Este mapa no ejecuta nada. Sirve para navegar el codigo. Sus "outputs" son rutas para lectura y citas.

## Como se conecta con otras partes

Cada documento de esta carpeta cita subconjuntos de estas rutas. Cuando haya duda, volver a este mapa.

## Que funciona hoy en G1 real

La mayoria de las piezas existen, pero repartidas. El registro `dimos/robot/all_blueprints.py` confirma los nombres `unitree-g1`, `unitree-g1-agentic`, `unitree-g1-full`, `unitree-g1-nav-onboard` y `unitree-g1-detection`.

## Que parece incompleto o separado

No hay un unico archivo blueprint que combine agente + nav onboard + detection + memoria de objetos + STT.

## Notas para mi proyecto/paper

Este mapa permite justificar cualquier afirmacion con ruta de codigo. Para un paper, conviene crear una tabla "subsystem -> files -> current status -> proposed integration".
