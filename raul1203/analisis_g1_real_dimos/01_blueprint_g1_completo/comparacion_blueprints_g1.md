# Comparacion de blueprints G1 reales

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Comparar rapidamente los blueprints G1 reales pedidos y elegir cual profundizar.

## Explicacion conceptual desde cero

Un blueprint no es un modo de robot magico: es solo una composicion concreta de modulos. El nombre puede decir "full", "agentic" o "nav", pero la verdad esta en los modulos que contiene.

## Archivos, clases y funciones relevantes

- Registro: `dimos/robot/all_blueprints.py`
- `unitree-g1-basic`: `dimos/robot/unitree/g1/blueprints/basic/unitree_g1_basic.py`
- `unitree-g1`: `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1.py`
- `unitree-g1-agentic`: `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_agentic.py`
- `unitree-g1-full`: `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_full.py`
- `unitree-g1-nav-onboard`: `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- `unitree-g1-detection`: `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_detection.py`

## Tabla de comparacion

| Blueprint | Real/sim | Componentes principales | Agente | Nav real avanzada | Deteccion 3D | Memoria | Control |
|---|---|---|---|---|---|---|---|
| `unitree-g1-basic` | real | primitive no nav + `G1Connection` | no | no | no | no | WebRTC |
| `unitree-g1` | real | basic + `SpatialMemory` + `ObjectTracking` | no | no | tracking incompleto si no hay depth | si, frames/tags | WebRTC |
| `unitree-g1-agentic` | real | `unitree_g1` + `_agentic_skills` | si | no integrado | no blueprint detection | si | WebRTC skills |
| `unitree-g1-full` | real | `unitree_g1_shm` + `_agentic_skills` + `KeyboardTeleop` | si | no integrado | no blueprint detection | si | WebRTC + teleop |
| `unitree-g1-nav-onboard` | real | FastLIO2 + nav stack + MovementManager + DDS SDK | no | si | no | no | DDS SDK |
| `unitree-g1-detection` | real | basic + YOLO person + Detection3D + ObjectDB + PersonTracker | no | no | si | ObjectDB temporal | WebRTC base |

## Flujo de datos

```text
unitree-g1-full
  -> unitree_g1_shm
       -> unitree_g1
            -> unitree_g1_basic
                 -> uintree_g1_primitive_no_nav + G1Connection
            -> SpatialMemory + ObjectTracking
       -> pSHM color_image + extra vis
  -> McpServer + McpClient + NavigationSkillContainer + SpeakSkill + UnitreeG1SkillContainer
  -> KeyboardTeleop
```

```text
unitree-g1-nav-onboard
  -> FastLio2
  -> create_nav_stack(planner="simple")
  -> MovementManager
  -> G1HighLevelDdsSdk
  -> vis_module
```

```text
unitree-g1-detection
  -> unitree_g1_basic
  -> Detection3DModule
  -> ObjectDBModule
  -> PersonTracker
```

## Inputs y outputs

- `unitree-g1-full`: recibe texto/agente, camara, teleop; emite voz, WebRTC move, skills.
- `unitree-g1-nav-onboard`: recibe lidar y goals por streams; emite `cmd_vel` hacia DDS.
- `unitree-g1-detection`: recibe imagen y pointcloud; emite detecciones, crops, pointclouds recortadas y target.

## Como se conecta con otras partes

`unitree-g1-full` es la mejor base agentica. `unitree-g1-nav-onboard` es la mejor base de navegacion real. `unitree-g1-detection` es la mejor base de deteccion 3D/persona. Un G1 agentico total deberia combinar esas tres familias.

## Que funciona hoy en G1 real

Cada bloque tiene funcionalidad real plausible por separado. El registro confirma que los nombres existen.

## Que parece incompleto o separado

No hay blueprint unico con agente + nav onboard + deteccion 3D. Ademas, el agente espera `NavigationInterfaceSpec`, y el nav stack onboard no lo implementa.

## Notas para mi proyecto/paper

Esta comparacion es la evidencia principal para motivar una contribucion: integracion agentica completa sobre hardware real.
