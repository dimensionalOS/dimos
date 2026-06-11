# Agente, MCP y skills

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar como funciona el agente LLM de G1: como recibe instrucciones, como descubre tools, que skills tiene y como llama acciones reales.

## Explicacion conceptual desde cero

El agente no sabe mover el robot por si mismo. El agente es un `McpClient` que usa un modelo LLM y un conjunto de tools. Esas tools vienen de un `McpServer` local. El server inspecciona los modulos desplegados y expone como tools todos los metodos marcados con `@skill`.

MCP aqui funciona como un protocolo JSON-RPC sobre HTTP:

- `POST /mcp` para `initialize`, `tools/list`, `tools/call`.
- `GET /mcp` para eventos SSE de progreso.

El puerto por defecto es `http://localhost:9990/mcp`.

## Archivos, clases y funciones relevantes

- `dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py`
- `dimos/agents/mcp/mcp_server.py`: `McpServer`
- `dimos/agents/mcp/mcp_client.py`: `McpClient`
- `dimos/agents/annotation.py`: `@skill`
- `dimos/robot/unitree/g1/skill_container.py`: skills G1 directas.
- `dimos/agents/skills/navigation.py`: `NavigationSkillContainer`
- `dimos/agents/skills/speak_skill.py`: `SpeakSkill`
- `dimos/robot/unitree/g1/system_prompt.py`: `G1_SYSTEM_PROMPT`

## Flujo de datos

```text
Usuario -> /human_input
  -> McpClient.human_input
  -> HumanMessage
  -> LangGraph create_agent(model="gpt-4o", tools=...)
  -> AIMessage/tool_calls
  -> McpClient llama POST /mcp tools/call
  -> McpServer busca RpcCall por nombre
  -> RPC al modulo propietario de la skill
  -> resultado texto vuelve al LLM
```

## Inputs y outputs

Inputs:

- Texto humano en `/human_input`.
- Tools de MCP descubiertas con `tools/list`.
- System prompt G1.

Outputs:

- Mensajes del agente en `/agent`.
- Estado idle en `/agent_idle`.
- Tool calls a skills.
- Acciones: WebRTC move, TTS, tags, navegacion, etc.

## Como se conecta con otras partes

`_agentic_skills` incluye:

```text
McpServer
McpClient(system_prompt=G1_SYSTEM_PROMPT)
NavigationSkillContainer
SpeakSkill
UnitreeG1SkillContainer
```

`McpServer.on_system_modules()` recopila skills de todos los modulos. `McpClient.on_system_modules()` llama `tools/list`, crea tools LangChain y arranca el loop del agente.

## Que funciona hoy en G1 real

Las skills G1 directas existen:

- `move(x, y, yaw, duration)` en `UnitreeG1SkillContainer`.
- `execute_arm_command(command_name)`.
- `execute_mode_command(command_name)`.
- `speak(text, blocking=True)`.
- `tag_location(location_name)`.
- `navigate_with_text(query)`.
- `stop_navigation()`.
- Tools internas del server: `server_status`, `list_modules`, `agent_send`.

## Que parece incompleto o separado

`navigate_with_text` depende de `_navigation: NavigationInterfaceSpec`. En los blueprints agentic/full G1 no se ve un modulo que satisfaga ese spec. Por tanto, la navegacion semantica no esta cerrada contra el nav stack real onboard.

Ademas, el modelo por defecto del cliente es `gpt-4o`, y Qwen se usa en la skill de navegacion visual para obtener bboxes de una imagen, no como agente principal.

## Notas para mi proyecto/paper

La arquitectura agentica es valiosa porque convierte capacidades de robot en tools. La brecha cientifica es grounding: que el tool `navigate_with_text` no solo responda, sino que se conecte a percepcion, memoria y navegacion real verificable.
