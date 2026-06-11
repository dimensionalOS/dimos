# Analisis G1 real en DimOS

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo

Esta carpeta explica, desde cero y con rutas reales del codigo, como funciona DimOS para un Unitree G1 real: blueprints, agente, MCP/tools, skills, voz, vision, lidar, SLAM, navegacion, memoria espacial, control, visualizacion, logs, seguridad y brechas de integracion.

La lectura se enfoca en robot real. La simulacion solo aparece cuando ayuda a separar lo que existe en sim de lo que existe para hardware.

## Conclusion ejecutiva

El blueprint G1 real mas completo por nombre y composicion es `unitree-g1-full`, definido en `dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_full.py`. Combina `unitree_g1_shm`, `_agentic_skills` y `KeyboardTeleop`.

Pero "mas completo" no significa "G1 agentico total". En el codigo actual, el stack esta partido:

- `unitree-g1-full`: agente + MCP + skills + TTS + memoria espacial + tracking basico + camara webcam + conexion G1 WebRTC + teleop.
- `unitree-g1-nav-onboard`: lidar Mid-360 + FastLIO2 + PGO + terrain analysis + planners + DDS SDK control.
- `unitree-g1-detection`: deteccion YOLO/persona + deteccion 3D con pointcloud + ObjectDB + PersonTracker.

La brecha principal es que el agente de G1 incluye `NavigationSkillContainer`, pero no incluye en ese blueprint un modulo que implemente `NavigationInterfaceSpec`. El nav stack onboard recibe goals por streams, no por esa interfaz RPC. Por eso el camino "texto del usuario -> navigate_with_text -> set_goal -> nav stack real" no queda cerrado en un unico blueprint real.

## Diagrama general

```text
Usuario
  |
  | dimos agent-send / humancli / MCP agent_send
  v
/human_input (pLCM)
  |
  v
McpClient (LangGraph + GPT-4o)
  |
  | descubre tools en http://localhost:9990/mcp
  v
McpServer
  |
  | llama @skill por RPC
  v
Skills G1:
  - speak -> OpenAI TTS -> sounddevice
  - move / gestos / modos -> G1Connection -> WebRTC
  - tag_location / navigate_with_text -> SpatialMemory + NavigationInterfaceSpec

Piezas separadas para sistema total:

Camara/webcam -> SpatialMemory / ObjectTracking
Lidar Mid-360 -> FastLIO2 -> PGO -> terrain maps -> planners -> MovementManager -> cmd_vel -> G1HighLevelDdsSdk
YOLO + pointcloud -> Detection3DModule / ObjectDBModule / PersonTracker
Rerun/logs/LCM -> inspeccion y debug
```

## Orden recomendado de lectura

1. `00_resumen/vision_general.md`
2. `01_blueprint_g1_completo/comparacion_blueprints_g1.md`
3. `01_blueprint_g1_completo/blueprint_mas_completo_real.md`
4. `02_agente_mcp_skills/agente_mcp_skills.md`
5. `06_navegacion/nav_stack_y_texto_a_goal.md`
6. `07_memoria_semantica_espacial/spatial_memory.md`
7. `04_percepcion_vision/camaras_deteccion_tracking.md`
8. `05_lidar_slam_mapeo/lidar_slam_nav_mapas.md`
9. `08_control_robot_g1/webrtc_dds_cmd_vel.md`
10. `10_flujo_completo/instruccion_a_accion.md`
11. `11_brechas_para_g1_agentico_total/diagnostico_y_roadmap.md`

## Carpetas

- `00_resumen`: conceptos base y mapa de archivos.
- `01_blueprint_g1_completo`: seleccion del stack mas completo, comparacion y conexiones.
- `02_agente_mcp_skills`: LLM, MCP server/client, skills y prompt G1.
- `03_input_output_humano`: texto, humancli, agent-send, TTS y STT.
- `04_percepcion_vision`: camaras, deteccion 2D/3D, tracking, modelos y limites.
- `05_lidar_slam_mapeo`: Mid-360, FastLIO2, TF, mapas y costmaps.
- `06_navegacion`: nav stack, MovementManager, texto a goal y seguridad.
- `07_memoria_semantica_espacial`: SpatialMemory, ChromaDB, CLIP, tags y memoria de objetos.
- `08_control_robot_g1`: WebRTC, DDS SDK, cmd_vel, gestos y riesgos.
- `09_visualizacion_logs_debug`: Rerun, logs, LCM topics y observabilidad.
- `10_flujo_completo`: flujos de instruccion a accion y ejemplos.
- `11_brechas_para_g1_agentico_total`: que funciona, que falta y roadmap.
- `12_comandos`: comandos seguros, comandos de robot real y comandos que no conviene correr sin hardware.

## Que funciona hoy en G1 real

- Control directo por WebRTC desde `G1Connection` y `UnitreeG1SkillContainer.move`.
- Gestos y modos por requests a topics Unitree como `rt/api/arm/request` y `rt/api/sport/request`.
- Agente MCP con `McpServer` y `McpClient`, siempre que el blueprint construya correctamente.
- Habla con `SpeakSkill`, OpenAI TTS y salida local `sounddevice`.
- Camara webcam monocular via `CameraModule` y `Webcam`.
- Memoria espacial de frames visuales con metadata de pose y ChromaDB/CLIP.
- Nav stack real onboard por DDS en `unitree-g1-nav-onboard`, pero separado del blueprint agentic.
- Deteccion 3D/persona en `unitree-g1-detection`, tambien separada.

## Que falta integrar

- Unificar `unitree-g1-full`, `unitree-g1-nav-onboard` y `unitree-g1-detection`.
- Darle al nav stack onboard una interfaz `NavigationInterfaceSpec` o adaptar `NavigationSkillContainer` a goals por streams.
- Resolver frames/TF entre `world/base_link`, `map/body`, `camera_link` y `camera_optical`.
- Conectar memoria espacial y deteccion de objetos persistente.
- Incluir STT o `WebInput` en el blueprint G1 real si se quiere que el robot escuche.
- Definir seguridad operacional para comandos directos, teleop, DDS, WebRTC, stop y zonas prohibidas.

## Roadmap recomendado

1. Crear un blueprint experimental `unitree-g1-agentic-nav-detection` que combine agente, nav onboard y deteccion.
2. Implementar un adaptador `NavigationInterfaceSpec` para el nav stack onboard.
3. Integrar `Detection3DModule/ObjectDBModule` con `SpatialMemory` para memoria de objetos.
4. Alinear TF y nombres de frames de todo el stack real.
5. Agregar STT como modulo opt-in, no siempre activo.
6. Agregar capa de seguridad: limites de velocidad, e-stop, zonas prohibidas, validacion de goals y estado del robot.
7. Registrar datasets y experimentos reproducibles para paper: texto a goal, memoria visual, deteccion 3D, navegacion segura y recuperacion ante fallos.
