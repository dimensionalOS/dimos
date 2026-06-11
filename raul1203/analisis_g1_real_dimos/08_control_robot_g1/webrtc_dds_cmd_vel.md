# WebRTC, DDS y cmd_vel

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Responder que comandos controlan al G1 real y si usa WebRTC, DDS o ambos.

## Explicacion conceptual desde cero

El repo tiene dos familias de control G1 real:

- WebRTC: usado por `G1Connection` y las skills agenticas actuales.
- DDS/Unitree SDK2: usado por `G1HighLevelDdsSdk` en nav onboard.

Ambos pueden mover el robot, pero estan en blueprints distintos.

## Archivos, clases y funciones relevantes

- WebRTC generic: `dimos/robot/unitree/connection.py`: `UnitreeWebRTCConnection`
- G1 WebRTC module: `dimos/robot/unitree/g1/connection.py`: `G1Connection`
- G1 agent skill: `dimos/robot/unitree/g1/skill_container.py`
- DDS high-level: `dimos/robot/unitree/g1/effectors/high_level/dds_sdk.py`
- DDS spec: `dimos/robot/unitree/g1/effectors/high_level/high_level_spec.py`
- WebRTC high-level alternative: `dimos/robot/unitree/g1/effectors/high_level/webrtc.py`
- Whole body low-level DDS: `dimos/robot/unitree/g1/wholebody_connection.py`

## Flujo de datos

### Agentic/full WebRTC

```text
move(x,y,yaw,duration)
  -> Twist
  -> G1Connection.move
  -> UnitreeWebRTCConnection.move
  -> RTC_TOPIC["WIRELESS_CONTROLLER"]
```

### Nav onboard DDS

```text
nav_cmd_vel -> MovementManager -> cmd_vel
  -> G1HighLevelDdsSdk.move
  -> LocoClient.Move / SetVelocity
  -> Unitree SDK2 DDS
```

### Low-level whole body DDS

```text
motor_command: MotorCommandArray
  -> G1WholeBodyConnection
  -> rt/lowcmd

rt/lowstate
  -> motor_states + imu
```

## Inputs y outputs

Inputs:

- `Twist` para movimiento.
- `command_name` para gestos/modos.
- DDS network interface para SDK2.
- Robot IP para WebRTC.

Outputs:

- Comandos joystick WebRTC.
- LocoClient velocity DDS.
- Unitree API requests.
- Motor states/IMU para low-level.

## Como se conecta con otras partes

`unitree-g1-full` usa `G1Connection` por WebRTC. `unitree-g1-nav-onboard` usa `G1HighLevelDdsSdk` por DDS. `G1HighLevelWebRtc` existe como alternativa de high-level spec, pero no es el modulo usado por `unitree-g1-full`.

## Que funciona hoy en G1 real

Ambas vias estan implementadas. WebRTC tiene auto-stop con `cmd_vel_timeout`. DDS SDK tambien para movimientos continuos sin duration.

## Que parece incompleto o separado

No hay una decision unica de arquitectura. Si se integran agentic y nav, hay que decidir si el robot recibe todos los `cmd_vel` por DDS, o si se conserva WebRTC para skills directas. Mezclar ambos sin arbitro puede ser peligroso.

## Notas para mi proyecto/paper

Para hardware real, conviene proponer un unico "motion authority": todos los comandos pasan por un mux/safety arbiter y luego por una sola interfaz controlada.
