# Instruccion a accion

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Describir el flujo completo ideal y compararlo con el flujo real actual.

## Explicacion conceptual desde cero

Un robot agentico completo deberia cerrar este ciclo:

```text
entender instruccion -> percibir mundo -> recuperar memoria -> elegir accion -> planificar -> ejecutar -> verificar -> comunicar
```

DimOS tiene piezas para cada paso, pero no todas estan conectadas para G1 real.

## Archivos, clases y funciones relevantes

- Agente: `dimos/agents/mcp/mcp_client.py`
- Skills: `dimos/agents/skills/navigation.py`, `dimos/robot/unitree/g1/skill_container.py`
- Memoria: `dimos/perception/spatial_perception.py`
- Vision: `dimos/perception/detection/module3D.py`
- Nav: `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- Control: `dimos/robot/unitree/g1/effectors/high_level/dds_sdk.py`, `dimos/robot/unitree/g1/connection.py`

## Flujo de datos

### Flujo real directo, mas cerrado

```text
"camina adelante"
  -> /human_input
  -> McpClient
  -> LLM llama move
  -> UnitreeG1SkillContainer.move
  -> G1Connection.move
  -> UnitreeWebRTCConnection
  -> G1
```

### Flujo semantico deseado

```text
"ve a la mesa roja"
  -> LLM
  -> buscar objeto visible con Qwen/YOLO
  -> si no visible, consultar memoria espacial/objetos
  -> obtener pose segura en map
  -> set_goal / stream goal
  -> nav stack onboard
  -> DDS cmd_vel
  -> verificar llegada
  -> speak resultado
```

## Inputs y outputs

Inputs:

- Texto.
- Imagen.
- Lidar/mapa.
- Memoria.

Outputs:

- Tool calls.
- Goals.
- Movimiento.
- Voz.
- Logs/visualizacion.

## Como se conecta con otras partes

El flujo directo usa WebRTC y no necesita nav. El flujo semantico necesita unir agente, memoria, detection, nav onboard y safety.

## Que funciona hoy en G1 real

Movimiento directo y habla estan mas cerca de funcionar end-to-end. Navegacion semantica completa esta incompleta por la interfaz nav.

## Que parece incompleto o separado

No hay verificacion final de "llegue al objeto" basada en vision o lidar. Tampoco hay recuperacion si falla el goal, ni decision multi-step robusta.

## Notas para mi proyecto/paper

El ciclo verificar-comunicar es importante para evaluar agentes embodied. Un paper deberia medir no solo si llama tools, sino si completa tareas y detecta fallos.
