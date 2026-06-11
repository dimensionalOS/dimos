# Texto, voz y humancli

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Responder como recibe instrucciones el agente, como habla y si escucha en el G1 real.

## Explicacion conceptual desde cero

El canal humano principal del agente es texto por LCM pickled en `/human_input`. Hay varias formas de publicar ahi:

- `dimos agent-send "mensaje"` usa MCP `agent_send`.
- `dimos humancli` abre una interfaz TUI que publica a `/human_input` y escucha `/agent`.
- El propio `McpServer.agent_send` publica al mismo topic.

La voz de salida es una skill (`speak`) que genera audio con OpenAI TTS y lo reproduce con `sounddevice`.

La voz de entrada existe como componentes STT (`WhisperNode`, `WebInput`), pero no esta incluida en `unitree-g1-agentic` ni `unitree-g1-full`.

## Archivos, clases y funciones relevantes

- `dimos/robot/cli/dimos.py`: `agent_send_cmd`, `humancli`, `mcp call`.
- `dimos/utils/cli/human/humancli.py`: `HumanCLIApp`.
- `dimos/agents/mcp/mcp_server.py`: `agent_send`.
- `dimos/agents/mcp/mcp_client.py`: `human_input: In[str]`.
- `dimos/agents/skills/speak_skill.py`: `SpeakSkill.speak`.
- `dimos/stream/audio/tts/node_openai.py`: `OpenAITTSNode`.
- `dimos/stream/audio/stt/node_whisper.py`: `WhisperNode`.
- `dimos/agents/web_human_input.py`: `WebInput`.

## Flujo de datos

### Texto

```text
dimos agent-send "hola"
  -> MCP tool agent_send
  -> pLCMTransport("/human_input")
  -> McpClient.human_input
  -> LLM
```

### humancli

```text
Input textual -> /human_input
/agent -> pantalla humancli
/agent_idle -> indicador thinking
```

### Habla

```text
LLM decide speak("...")
  -> McpServer -> SpeakSkill.speak
  -> OpenAITTSNode(model="tts-1", voice=ONYX, speed=1.2)
  -> SounddeviceAudioOutput(sample_rate=24000)
```

### Escucha potencial, no integrada en G1 full

```text
Microfono/browser -> AudioNormalizer -> WhisperNode -> texto -> /human_input
```

## Inputs y outputs

Inputs:

- Texto CLI/TUI/MCP.
- Audio de microfono solo si se usa pipeline STT o `WebInput`.

Outputs:

- Voz por speaker local.
- Mensajes del agente por `/agent`.
- Tool calls mostrables en `humancli`.

## Como se conecta con otras partes

`McpClient` es el puente hacia el LLM. `SpeakSkill` es solo una skill mas; el agente debe decidir llamarla.

## Que funciona hoy en G1 real

El canal de texto esta claro y se puede usar con un blueprint que tenga MCP. La salida de voz esta integrada en `_agentic_skills`.

## Que parece incompleto o separado

STT no esta incluido en el blueprint G1 real agentic/full. `WebInput` existe y usa Whisper, pero aparece integrado en otros stacks, no en G1 agentic. Tampoco hay wake word, cancelacion de eco ni politica de escucha continua.

## Notas para mi proyecto/paper

Para un robot social/humanoide, la diferencia entre "habla" y "escucha" es critica. Un paper puede proponer dialogo multimodal robusto: STT con wake word, TTS, estado de agente y confirmaciones de seguridad antes de accion fisica.
