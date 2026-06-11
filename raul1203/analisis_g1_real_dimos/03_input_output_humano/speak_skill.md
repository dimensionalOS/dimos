# Speak skill

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar la skill de habla y sus limites practicos.

## Explicacion conceptual desde cero

`SpeakSkill` convierte texto a audio. No es parte del LLM internamente: es una tool. El prompt le dice al agente que use `speak` porque los humanos cerca del robot no ven el texto.

## Archivos, clases y funciones relevantes

- `dimos/agents/skills/speak_skill.py`
- `dimos/stream/audio/tts/node_openai.py`
- `dimos/stream/audio/node_output.py`

## Flujo de datos

```text
speak(text, blocking=True)
  -> OpenAITTSNode.consume_text
  -> OpenAI audio.speech.create
  -> AudioEvent
  -> SounddeviceAudioOutput.write
```

## Inputs y outputs

Input:

- `text: str`
- `blocking: bool`

Output:

- Audio reproducido.
- String de retorno: `Spoke: ...`, error o warning.

## Como se conecta con otras partes

`SpeakSkill` se despliega en `_agentic_skills`. El MCP server lo expone como tool. El LLM lo llama cuando quiere comunicarse.

## Que funciona hoy en G1 real

La skill esta incluida en `unitree-g1-agentic` y `unitree-g1-full` mediante `_agentic_skills`.

## Que parece incompleto o separado

Depende del dispositivo de audio local y de OpenAI API. Si no hay salida de audio, `SounddeviceAudioOutput` consume y descarta eventos. No hay sincronizacion con escucha para evitar que STT transcriba la propia voz del robot.

## Notas para mi proyecto/paper

La voz puede usarse como canal de transparencia: "voy a caminar", "no puedo hacer eso por seguridad", "perdi el objetivo". Esto mejora confianza y reduce riesgo en interaccion humano-robot.
