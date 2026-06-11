# Comandos G1 real y peligrosos

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Separar comandos posibles para G1 real de comandos que no se deben correr sin robot, espacio y supervision.

## Explicacion conceptual desde cero

En robotica, "arrancar un blueprint" puede activar sensores, abrir conexiones de red y subscribirse a comandos de movimiento. Un comando que parece inofensivo puede publicar `cmd_vel` o requests Unitree.

## Archivos, clases y funciones relevantes

- `dimos/robot/cli/dimos.py`: `run`, `mcp call`, `agent-send`, `topic send`
- `dimos/robot/unitree/g1/connection.py`
- `dimos/robot/unitree/g1/effectors/high_level/dds_sdk.py`
- `dimos/robot/unitree/g1/skill_container.py`

## Flujo de datos

```text
dimos run unitree-g1-full
  -> G1Connection WebRTC
  -> possible cmd_vel / API requests

dimos run unitree-g1-nav-onboard
  -> FastLIO2 network bind
  -> DDS SDK
  -> planner cmd_vel

dimos mcp call move ...
  -> skill
  -> physical movement
```

## Inputs y outputs

Inputs:

- Robot IP.
- Network interface.
- MCP args.
- Topics.

Outputs:

- Movimiento fisico.
- Gestos.
- Cambio de modo.
- Lidar/network activity.

## Como se conecta con otras partes

Los comandos aqui activan los modulos documentados en control, navegacion y agente.

## Que funciona hoy en G1 real

Comandos posibles, solo con robot preparado y supervision:

```bash
dimos run unitree-g1-full --robot-ip 192.168.123.161
dimos run unitree-g1-agentic --robot-ip 192.168.123.161
dimos run unitree-g1-nav-onboard
dimos run unitree-g1-detection --robot-ip 192.168.123.161
dimos agent-send "say hello"
dimos mcp call speak --arg text='"hello"'
```

Comandos que pueden mover o cambiar estado:

```bash
dimos mcp call move --arg x=0.3 --arg duration=1.0
dimos mcp call execute_arm_command --arg command_name='"HighFive"'
dimos mcp call execute_mode_command --arg command_name='"RunMode"'
dimos topic send /cmd_vel 'Twist(...)'
```

## Que parece incompleto o separado

No hay un comando unico "safe dry-run" que valide tool calls sin ejecutarlos en hardware. Tampoco hay prompt de confirmacion obligatorio antes de movimientos.

## Notas para mi proyecto/paper

Para experimentos reales, definir protocolo:

- area despejada.
- e-stop fisico.
- operador listo.
- velocidad limitada.
- logs activos.
- una sola fuente de `cmd_vel`.
- checklist de TF y topic rates antes de permitir accion.
