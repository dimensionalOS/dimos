# Diagnostico y roadmap

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Responder que ya funciona, que esta separado, que falta integrar y cual es el roadmap recomendado.

## Explicacion conceptual desde cero

El G1 agentico total no es solo "poner mas modulos en un blueprint". Requiere compatibilidad de interfaces, frames, safety y verificacion de tareas.

## Archivos, clases y funciones relevantes

- `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_full.py`
- `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_detection.py`
- `dimos/navigation/navigation_spec.py`
- `dimos/core/coordination/module_coordinator.py`

## Flujo de datos

```text
Estado actual:
agentic/full     detection       nav-onboard
    |               |                |
    +--- separados--+--- separados---+

Estado deseado:
agent + perception + memory + nav + control + safety
    -> un solo blueprint real
    -> interfaces compatibles
    -> TF coherente
    -> logs y evaluacion
```

## Inputs y outputs

Inputs para roadmap:

- Codigo existente.
- Requisitos de hardware real.
- Preguntas del proyecto/paper.

Outputs:

- Lista priorizada de trabajo.

## Como se conecta con otras partes

Este diagnostico resume todos los documentos anteriores.

## Que funciona hoy en G1 real

- Control directo WebRTC.
- DDS nav onboard.
- FastLIO2 y mapas.
- MCP agent.
- Speak skill.
- SpatialMemory de frames/tags.
- Deteccion 3D/persona en blueprint separado.

## Que parece incompleto o separado

Brechas principales:

1. `NavigationSkillContainer` no tiene proveedor `NavigationInterfaceSpec` en `unitree-g1-full`.
2. Nav onboard no esta unido al agente.
3. Detection 3D no esta unida al agente/memoria.
4. STT no esta unido al blueprint G1 real.
5. Frames `world/base_link` y `map/body` no estan normalizados.
6. No hay safety arbiter central.
7. Memoria actual no es memoria de objetos persistente.

## Roadmap recomendado

1. Crear adaptador `NavStackNavigationAdapter` que implemente `NavigationInterfaceSpec`.
2. Crear blueprint nuevo que combine `unitree_g1_full` + `unitree_g1_nav_onboard` de forma controlada.
3. Remapear/aislar `cmd_vel` para que haya un solo authority.
4. Normalizar TF (`map`, `odom`, `body`, `base_link`, `world`).
5. Integrar `unitree-g1-detection` y exponer tools de consulta de objetos.
6. Crear memoria de objetos persistente con detecciones 3D.
7. Agregar STT como opt-in.
8. Agregar safety arbiter.
9. Crear experimentos reproducibles y datasets.

## Notas para mi proyecto/paper

El roadmap mismo puede convertirse en metodologia incremental: baseline, integracion nav, integracion vision, memoria semantica, safety y evaluacion.
