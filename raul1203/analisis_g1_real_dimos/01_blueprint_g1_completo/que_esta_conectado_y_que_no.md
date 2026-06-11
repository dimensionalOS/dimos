# Que esta conectado y que no

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Separar hechos de codigo: que esta conectado hoy en blueprints G1 reales y que queda como pieza externa.

## Explicacion conceptual desde cero

En DimOS, "existe el modulo" no significa "esta en el blueprint". Y "esta en el blueprint" no significa "tiene todos sus inputs alimentados". Hay que mirar tres niveles:

1. El archivo existe.
2. El blueprint incluye el modulo.
3. Los streams/specs que necesita el modulo tienen proveedor.

## Archivos, clases y funciones relevantes

- `_connect_module_refs` en `dimos/core/coordination/module_coordinator.py`
- `NavigationInterfaceSpec` en `dimos/navigation/navigation_spec.py`
- `NavigationSkillContainer` en `dimos/agents/skills/navigation.py`
- `ReplanningAStarPlanner` en `dimos/navigation/replanning_a_star/module.py`
- `unitree_g1_nav_onboard` en `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`

## Flujo de datos

```text
Si modulo pide _navigation: NavigationInterfaceSpec
  -> ModuleCoordinator busca otro modulo con set_goal/get_state/is_goal_reached/cancel_goal
  -> si no hay candidato y no es Optional
  -> falla la conexion de module refs
```

## Inputs y outputs

`NavigationSkillContainer` necesita:

- `color_image: In[Image]`
- `odom: In[PoseStamped]`
- `_spatial_memory: SpatialMemorySpec`
- `_navigation: NavigationInterfaceSpec`
- `_object_tracking: ObjectTrackingSpec | None`

En `unitree-g1-full`:

- `SpatialMemory` existe.
- `ObjectTracking` existe, aunque con limitaciones de depth.
- No se ve un proveedor de `NavigationInterfaceSpec`.

## Como se conecta con otras partes

El nav onboard no implementa `NavigationInterfaceSpec`. Sus modulos consumen `goal: In[PointStamped]` y `stop_movement: In[Bool]` por streams. Eso es otra interfaz.

## Que funciona hoy en G1 real

Conectado en `unitree-g1-full`:

- MCP server/client.
- Skills G1 de movimiento directo, gestos y modos.
- Speak skill.
- SpatialMemory.
- ObjectTracking.
- G1Connection WebRTC.
- Keyboard teleop.

Conectado en `unitree-g1-nav-onboard`:

- FastLIO2 Mid-360.
- Nav stack.
- MovementManager.
- G1HighLevelDdsSdk.
- Rerun nav visualization.

Conectado en `unitree-g1-detection`:

- YOLO/person detection.
- Detection3DModule.
- ObjectDBModule.
- PersonTracker.

## Que parece incompleto o separado

- Agente + nav onboard: separado.
- Agente + detection 3D: separado.
- Nav skill + onboard planner: interfaz incompatible.
- Webcam monocular + ObjectTracking 3D: falta depth.
- STT + G1 agentic: no incluido.

## Notas para mi proyecto/paper

Este archivo da la lista de brechas tecnicas concretas. Es mejor que decir "falta integracion" de forma general.
