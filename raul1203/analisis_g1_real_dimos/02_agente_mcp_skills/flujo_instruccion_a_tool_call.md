# Flujo instruccion a tool call

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Describir paso a paso que ocurre cuando alguien escribe una instruccion para el G1 agentico.

## Explicacion conceptual desde cero

Una instruccion humana primero se vuelve mensaje. El agente la agrega a su historial. Luego el LLM decide si responder con texto o llamar una tool. Si llama tool, `McpClient` ejecuta una llamada MCP, `McpServer` la traduce a RPC y el modulo con la skill ejecuta codigo real.

## Archivos, clases y funciones relevantes

- `dimos/agents/mcp/mcp_client.py`
  - `McpClient.start`
  - `_fetch_tools`
  - `_mcp_tool_to_langchain`
  - `_process_message`
- `dimos/agents/mcp/mcp_server.py`
  - `_handle_tools_list`
  - `_handle_tools_call`
  - `McpServer.on_system_modules`
- `dimos/robot/cli/dimos.py`
  - `agent_send_cmd`
  - `mcp call`

## Flujo de datos

```text
1. Usuario:
   dimos agent-send "walk forward 2 meters"

2. CLI:
   llama tool MCP "agent_send"

3. McpServer.agent_send:
   publica "walk forward 2 meters" en pLCM "/human_input"

4. McpClient:
   recibe HumanMessage y lo mete en _message_queue

5. LangGraph:
   procesa historial + system prompt + tools

6. LLM:
   decide tool: move(x=0.5, duration=4.0)

7. McpClient:
   POST /mcp tools/call {"name": "move", "arguments": ...}

8. McpServer:
   ejecuta RpcCall hacia UnitreeG1SkillContainer.move

9. UnitreeG1SkillContainer:
   crea Twist y llama G1Connection.move

10. G1Connection:
    envia comando por UnitreeWebRTCConnection
```

## Inputs y outputs

Input principal:

- Texto natural.

Outputs intermedios:

- `HumanMessage`, `AIMessage`, `ToolMessage`.
- JSON-RPC MCP.
- RPC DimOS.

Output final:

- Comando real, habla o memoria/navegacion segun la skill.

## Como se conecta con otras partes

El flujo de tool call sirve para todas las skills. Lo que cambia es el modulo final:

- `SpeakSkill` para voz.
- `UnitreeG1SkillContainer` para movimiento/gestos/modos.
- `NavigationSkillContainer` para memoria/navegacion.
- `McpServer` para introspeccion y `agent_send`.

## Que funciona hoy en G1 real

El camino texto -> MCP -> skill directa de movimiento/gestos/habla esta implementado. Depende de que el blueprint arranque correctamente y de que el MCP server este disponible.

## Que parece incompleto o separado

El camino texto -> `navigate_with_text` -> nav real falla conceptualmente si no hay proveedor de `NavigationInterfaceSpec`. Tambien falta una politica de seguridad fuerte que valide tool calls antes de mover hardware.

## Notas para mi proyecto/paper

Este flujo puede presentarse como "language-to-tool-to-actuator pipeline". La parte original puede estar en validacion semantica, recuperacion de memoria y safety gating antes del actuator.
