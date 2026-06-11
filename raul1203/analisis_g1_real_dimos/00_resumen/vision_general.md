# Vision general

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Dar una vista desde cero de DimOS aplicado al Unitree G1 real y responder cual es la historia grande: que piezas existen, como se comunican y por que todavia no forman un unico "G1 agentico completo".

## Explicacion conceptual desde cero

DimOS organiza un robot como un conjunto de `Modules`. Un modulo es un proceso logico que publica y consume datos. Los datos viajan por streams tipados `In[T]` y `Out[T]`. Un `Blueprint` es la receta que junta modulos, les asigna transports y conecta streams con el mismo nombre y tipo.

Para el G1 real, DimOS tiene varios blueprints. Algunos se concentran en control basico, otros en percepcion, otros en navegacion y otros en agente. El agente no controla el robot directamente en Python: recibe texto, razona con un LLM, descubre tools por MCP y llama skills. Las skills son metodos Python marcados con `@skill`; DimOS las expone como tools.

## Archivos, clases y funciones relevantes

- `dimos/core/module.py`: base `Module`, streams y RPC.
- `dimos/core/stream.py`: `In`, `Out`, `Transport`.
- `dimos/core/coordination/blueprints.py`: `Blueprint`, `autoconnect`.
- `dimos/core/coordination/module_coordinator.py`: despliegue, conexion de streams e inyeccion de specs.
- `dimos/robot/all_blueprints.py`: nombres disponibles, incluido `unitree-g1-full`.
- `dimos/robot/unitree/g1/blueprints/...`: blueprints especificos de G1.
- `dimos/agents/mcp/mcp_server.py` y `dimos/agents/mcp/mcp_client.py`: MCP local.

## Flujo de datos

```text
Blueprint
  -> ModuleCoordinator
  -> workers/procesos
  -> streams In/Out
  -> transports LCM, pLCM, pSHM, DDS, WebRTC, native modules
  -> agente/tools/control/percepcion/navegacion
```

Para el G1 agentico:

```text
Texto humano -> /human_input -> McpClient -> LLM -> tool call -> McpServer -> @skill -> modulo real
```

Para navegacion onboard:

```text
Mid-360 -> FastLIO2 -> PGO/corrected_odometry -> terrain maps -> planner -> cmd_vel -> DDS SDK -> G1
```

## Inputs y outputs

- Inputs humanos: texto por `dimos agent-send`, `humancli` o MCP `agent_send`.
- Inputs sensores: webcam, lidar Mid-360, odometria, pointclouds.
- Outputs de accion: `cmd_vel`, requests Unitree WebRTC/DDS, voz TTS.
- Outputs de observabilidad: logs JSONL, Rerun, LCM topics.

## Como se conecta con otras partes

El sistema usa dos tipos de conexion:

- Streams para datos continuos: imagenes, pointclouds, odometria, goals, velocidades.
- RPC/specs para llamadas de metodo: skills, control directo, memoria, navegacion semantica.

El punto critico es que `NavigationSkillContainer` espera una referencia RPC que cumpla `NavigationInterfaceSpec`. El nav stack onboard actual esta orientado a streams, no a ese spec.

## Que funciona hoy en G1 real

Funciona como base separada: control WebRTC, agente MCP, habla, memoria visual, nav onboard, deteccion. No todo esta unido en un solo blueprint real operativo.

## Que parece incompleto o separado

La navegacion real mas avanzada vive separada del agente. La deteccion 3D/persona vive separada del agente. La escucha por microfono existe como componentes, pero no esta en los blueprints G1 agentic/full.

## Notas para mi proyecto/paper

La oportunidad fuerte para un paper es proponer una arquitectura integrada de "humanoid embodied semantic agent" que cierre el ciclo texto - percepcion - memoria - navegacion - accion segura en hardware real.
