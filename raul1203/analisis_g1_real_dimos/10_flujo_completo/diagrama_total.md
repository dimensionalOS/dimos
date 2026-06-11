# Diagrama total

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Ofrecer un diagrama textual compacto del sistema G1 agentico completo deseado.

## Explicacion conceptual desde cero

El stack completo debe verse como capas. Cada capa transforma datos: humano a texto, texto a tools, sensores a memoria/mapa, memoria a goals, goals a movimiento.

## Archivos, clases y funciones relevantes

Ver tambien `00_resumen/mapa_de_archivos_clave.md`.

## Flujo de datos

```text
                     +----------------------+
                     | Usuario humano       |
                     | texto / voz / web    |
                     +----------+-----------+
                                |
                                v
                        /human_input (pLCM)
                                |
                                v
                     +----------------------+
                     | McpClient + LLM      |
                     | GPT-4o + prompt G1   |
                     +----------+-----------+
                                |
                 tools/list + tools/call over MCP
                                |
                                v
                     +----------------------+
                     | McpServer            |
                     | @skill -> RPC        |
                     +----+------+-----+----+
                          |      |     |
                          |      |     |
          +---------------+      |     +------------------+
          v                      v                        v
  +---------------+      +---------------+        +----------------+
  | SpeakSkill    |      | G1 skills     |        | Nav skill      |
  | OpenAI TTS    |      | move/gestos   |        | text->goal     |
  +---------------+      +-------+-------+        +--------+-------+
                                  |                         |
                                  v                         v
                           WebRTC control          SpatialMemory/ObjectTracking
                                                            |
                                                            v
                                                     NavigationInterface
                                                            |
                                                            v
  +--------------------------------------------------+------------------+
  | Nav onboard real                                                    |
  | FastLIO2 -> PGO -> terrain maps -> planner -> MovementManager       |
  +---------------------------------------+-----------------------------+
                                          |
                                          v
                                    DDS cmd_vel
                                          |
                                          v
                                     Unitree G1
```

## Inputs y outputs

Inputs:

- Texto/voz.
- Camara.
- Lidar.
- Odometria/TF.

Outputs:

- Habla.
- Movimiento.
- Gestos.
- Goals.
- Mapas.
- Memoria.

## Como se conecta con otras partes

Las flechas continuas ya existen parcialmente. Las flechas que faltan son:

- Nav skill -> nav onboard.
- Detection/ObjectDB -> SpatialMemory.
- STT/WebInput -> G1 full.
- Safety arbiter -> todos los comandos fisicos.

## Que funciona hoy en G1 real

El diagrama existe como arquitectura potencial, no como unico blueprint listo.

## Que parece incompleto o separado

Faltan adaptadores y arbitraje. Sin eso, el sistema es un conjunto de subsistemas potentes pero desacoplados.

## Notas para mi proyecto/paper

Este diagrama puede ser la figura principal de propuesta. Marcar en color "implemented", "separate", "missing" seria muy claro.
