# Gestos, modos y riesgos

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar gestos de brazos, modos de movimiento y riesgos de control real.

## Explicacion conceptual desde cero

Ademas de caminar, el G1 puede ejecutar comandos predefinidos de brazos y modos. En el codigo, estos se envian como `api_id` y `parameter` a topics Unitree.

## Archivos, clases y funciones relevantes

- `dimos/robot/unitree/g1/skill_container.py`
- `dimos/robot/unitree/g1/effectors/high_level/commands.py`
- `dimos/robot/unitree/g1/effectors/high_level/dds_sdk.py`
- `dimos/robot/unitree/g1/effectors/high_level/webrtc.py`

## Flujo de datos

```text
execute_arm_command("ArmHeart")
  -> api_id 7106
  -> topic "rt/api/arm/request"
  -> parameter {"data": command_id}

execute_mode_command("RunMode")
  -> api_id 7101
  -> topic "rt/api/sport/request"
  -> parameter {"data": command_id}
```

## Inputs y outputs

Inputs:

- Arm command names: `Handshake`, `HighFive`, `Hug`, `HighWave`, `Clap`, `FaceWave`, `LeftKiss`, `ArmHeart`, `RightHeart`, `HandsUp`, `XRay`, `RightHandUp`, `Reject`, `CancelAction`.
- Mode command names: `WalkMode`, `WalkControlWaist`, `RunMode`.

Outputs:

- Request a Unitree API.
- String de exito/error para el agente.

## Como se conecta con otras partes

El system prompt G1 lista estos comandos. El LLM debe elegir un `command_name` exacto. Si no coincide, el codigo sugiere nombres cercanos con `difflib`.

## Que funciona hoy en G1 real

Las skills estan implementadas en `UnitreeG1SkillContainer`. Tambien hay versiones high-level en DDS/WebRTC effectors.

## Que parece incompleto o separado

El movimiento de brazos y modos puede ser riesgoso cerca de personas. El prompt dice priorizar seguridad, pero no hay una capa que valide distancia a humanos, espacio libre, estado del robot o necesidad de confirmacion.

## Notas para mi proyecto/paper

Los gestos son utiles para interaccion humano-robot, pero tambien son una prueba de safety. Propuesta: gesture safety envelope basado en percepcion 3D y proximidad humana.
