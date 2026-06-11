# MovementManager y seguridad

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar el rol de `MovementManager`, diferencias entre navegacion y movimiento directo, y riesgos de seguridad.

## Explicacion conceptual desde cero

Hay dos maneras de mover el G1:

- Movimiento directo: una skill publica velocidad por un tiempo.
- Navegacion: un planner calcula camino y produce `cmd_vel` continuamente.

`MovementManager` mezcla teleop y navegacion. Si hay teleop, cancela goal y suprime comandos de navegacion por un cooldown. Esto evita que el planner pelee contra el operador.

## Archivos, clases y funciones relevantes

- `dimos/navigation/movement_manager/movement_manager.py`
- `dimos/robot/unitree/g1/skill_container.py`
- `dimos/robot/unitree/g1/effectors/high_level/dds_sdk.py`
- `dimos/robot/unitree/g1/connection.py`
- `dimos/robot/unitree/keyboard_teleop.py`

## Flujo de datos

```text
nav_cmd_vel -> MovementManager -> cmd_vel
tele_cmd_vel -> MovementManager -> cancel goal + cmd_vel
clicked_point -> MovementManager -> goal + way_point
```

En nav onboard:

```text
PathFollower.cmd_vel remapped -> nav_cmd_vel
MovementManager.cmd_vel -> G1HighLevelDdsSdk.cmd_vel
```

En agentic/full:

```text
UnitreeG1SkillContainer.move
  -> G1Connection.move
  -> UnitreeWebRTCConnection.move
```

## Inputs y outputs

Inputs:

- `nav_cmd_vel`
- `tele_cmd_vel`
- `clicked_point`

Outputs:

- `goal`
- `way_point`
- `cmd_vel`
- `stop_movement`

## Como se conecta con otras partes

`MovementManager` es parte de `unitree-g1-nav-onboard`, no de `unitree-g1-full`. La skill `move` en G1 agentic evita el planner y manda velocidades directas.

## Que funciona hoy en G1 real

El nav onboard tiene mux teleop/nav. El control directo WebRTC tiene auto-stop por timeout en `UnitreeWebRTCConnection`. DDS SDK tambien tiene `cmd_vel_timeout`.

## Que parece incompleto o separado

No hay una politica de seguridad global que valide todos los caminos:

- skill directa WebRTC.
- DDS nav stack.
- teleop.
- gestos.
- mode changes.

No hay geofencing, validacion semantica de distancia, confirmacion para acciones riesgosas ni detector de humanos/obstaculos integrado al agente.

## Notas para mi proyecto/paper

Para hardware humanoide, safety debe ser una capa de sistema, no una nota del prompt. Propuesta: safety arbiter que intercepta goals, velocities y Unitree API requests.
