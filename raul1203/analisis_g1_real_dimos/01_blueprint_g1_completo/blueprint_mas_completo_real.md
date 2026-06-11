# Blueprint mas completo real

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Responder claramente: cual es el blueprint G1 real mas completo y que significa "completo" en este repo.

## Explicacion conceptual desde cero

Hay dos formas de medir "completo":

1. Por amplitud de capacidades agenticas en un solo blueprint.
2. Por completitud robotica fisica: lidar, SLAM, planificacion, control, deteccion y agente juntos.

Con criterio 1, el ganador es `unitree-g1-full`. Con criterio 2, todavia no hay ganador unico: el sistema esta separado.

## Archivos, clases y funciones relevantes

- `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_full.py`
- `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_shm.py`
- `dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py`
- `dimos/robot/unitree/keyboard_teleop.py`

`unitree_g1_full` hace:

```python
unitree_g1_full = autoconnect(
    unitree_g1_shm,
    _agentic_skills,
    KeyboardTeleop.blueprint(),
)
```

## Flujo de datos

```text
unitree_g1_full
  |
  +-- unitree_g1_shm
  |     +-- unitree_g1
  |     |     +-- unitree_g1_basic
  |     |     |     +-- camera / voxel map / costmap / frontier explorer / G1Connection
  |     |     +-- SpatialMemory / ObjectTracking
  |     +-- pSHM color_image / vis
  |
  +-- _agentic_skills
  |     +-- McpServer / McpClient / NavigationSkillContainer / SpeakSkill / UnitreeG1SkillContainer
  |
  +-- KeyboardTeleop
```

## Inputs y outputs

Inputs:

- Texto humano por `/human_input`.
- Imagen por `color_image`.
- Odometry/TF si hay publicadores externos.
- Teclado por `KeyboardTeleop`.

Outputs:

- `cmd_vel` hacia `G1Connection`.
- Voz por `SpeakSkill`.
- Tool calls y mensajes del agente.
- Imagen y visualizaciones.

## Como se conecta con otras partes

`unitree-g1-full` se conecta con control WebRTC y agente. No se conecta con `FastLio2`, `create_nav_stack`, `G1HighLevelDdsSdk` ni los modulos de `unitree-g1-detection`.

## Que funciona hoy en G1 real

Como intencion de blueprint, es el mas amplio para interaccion agentica: agente, MCP, skills, camara, memoria, tracking, habla, control directo y teleop.

## Que parece incompleto o separado

Hay una incompatibilidad importante: `NavigationSkillContainer` declara `_navigation: NavigationInterfaceSpec`, pero `unitree-g1-full` no incluye un modulo que implemente ese spec. El modulo `ObjectTracking` si puede satisfacer `ObjectTrackingSpec`, y `SpatialMemory` puede satisfacer `SpatialMemorySpec`, pero navegacion queda sin proveedor RPC.

## Notas para mi proyecto/paper

Para un paper, `unitree-g1-full` se puede presentar como "baseline agentico no integrado con nav/detection avanzada". La contribucion seria convertirlo en un stack completo con navegacion real y memoria semantica de objetos.
