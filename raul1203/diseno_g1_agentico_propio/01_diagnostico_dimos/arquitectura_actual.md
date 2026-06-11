# Arquitectura actual de DimOS para G1

Ultima modificacion: 2026-06-11 11:46:53 -05 -0500

## Resultado principal

**Hecho confirmado.** No existe en el commit analizado un unico blueprint de G1 real
que conecte voz de entrada, agente, deteccion 3D, memoria de entidades, lidar, SLAM,
navegacion y una autoridad segura de control.

Hay tres familias:

| Familia | Blueprint real principal | Capacidades |
|---|---|---|
| Agentic/perceptiva | `unitree-g1-full` | webcam, memoria visual, tracking basico, MCP, skills, TTS, WebRTC, teleop |
| Navegacion onboard | `unitree-g1-nav-onboard` | Mid-360, FAST-LIO2, PGO, terreno, A*, local planner, DDS |
| Deteccion | `unitree-g1-detection` | YOLO persona, proyeccion 3D, DB experimental y PersonTracker |

## Composicion de blueprints

### `unitree-g1-full`

Ruta:
`dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_full.py`

```text
unitree_g1_full
  + unitree_g1_shm
      + unitree_g1
          + unitree_g1_basic
              + uintree_g1_primitive_no_nav
              + G1Connection
          + SpatialMemory
          + ObjectTracking
      + vis_module
  + McpServer
  + McpClient
  + NavigationSkillContainer
  + SpeakSkill
  + UnitreeG1SkillContainer
  + KeyboardTeleop
```

**Hecho confirmado.** `NavigationSkillContainer` declara `_navigation:
NavigationInterfaceSpec`, pero este blueprint no incluye una implementacion visible
de `set_goal`, `get_state`, `is_goal_reached` y `cancel_goal`. La navegacion semantica
no queda cerrada solo por incluir el skill.

### `unitree-g1-nav-onboard`

Ruta:
`dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`

```text
Livox Mid-360 + IMU
  -> FastLio2
  -> registered_scan + odometry + global_map
  -> PGO
  -> corrected_odometry + map->odom
  -> TerrainAnalysis + TerrainMapExt
  -> SimplePlanner (A*)
  -> LocalPlanner
  -> PathFollower
  -> nav_cmd_vel
  -> MovementManager
  -> cmd_vel
  -> G1HighLevelDdsSdk
```

El blueprint usa hasta 12 workers y fija:

- velocidad maxima de `0.6 m/s`;
- celda del planner de `0.2 m`;
- inflacion de `0.5 m`;
- replanning de `5 Hz`;
- Mid-360 en `192.168.123.120` por defecto;
- host lidar en `192.168.123.164` por defecto.

**Hecho confirmado.** FAST-LIO2 publica odometria local en `odom`, PGO publica la
correccion `map -> odom`, y los consumidores usan `corrected_odometry`.

### `unitree-g1-detection`

Ruta:
`dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_detection.py`

El flujo pretendido es:

```text
color_image + pointcloud + TF
  -> YoloPersonDetector
  -> Detection3DModule
  -> ObjectDBModule
  -> PersonTracker
```

**Hecho confirmado.** Usa una calibracion `zed.CameraInfo.SingleWebcam` aunque la
base real crea `Webcam(camera_index=0)`. La validez metrica depende de que esa
calibracion coincida con la camara fisica y su montaje.

## Sensores y streams

### Camara

`uintree_g1_primitive_no_nav.py` crea un `CameraModule` con una webcam a 15 FPS,
slice izquierdo y un transform fijo:

```text
sensor -> camera_link
translation = (0.05, 0.0, 0.6)
pitch = 0.2 rad
```

Publica `color_image` y `camera_info`. No es la camara de profundidad nativa del G1
segun el codigo inspeccionado; es una webcam del host.

### Lidar e IMU

`FastLio2` integra Livox SDK2 directamente con el binario FAST-LIO-NON-ROS. Publica:

- `lidar: PointCloud2`;
- `odometry: Odometry`;
- `global_map: PointCloud2`;
- TF `odom -> body`.

La configuracion G1 coloca el Mid-360 a `z=1.2 m`.

### Control

Hay tres rutas:

| Ruta | Clase | Uso |
|---|---|---|
| WebRTC high-level | `G1Connection` / `G1HighLevelWebRtc` | velocidad, stand/lie y requests |
| DDS high-level | `G1HighLevelDdsSdk` | locomocion por `LocoClient`, timeout de `cmd_vel` |
| DDS low-level | `G1WholeBodyConnection` | 29 motores a 500 Hz, IMU y estados |

`unitree-g1-full` usa WebRTC. `unitree-g1-nav-onboard` usa DDS high-level.
`unitree-g1-coordinator` usa low-level DDS y un `ControlCoordinator`.

**Inferencia tecnica.** Combinar estas rutas sin arbitraje puede producir conflicto
de autoridad, de modo o de joints. Debe existir una maquina de estados exclusiva.

## Agente, MCP y skills

`McpClient`:

- usa `create_agent` de LangChain/LangGraph;
- configura `gpt-4o` por defecto;
- descubre tools por HTTP en `http://localhost:9990/mcp`;
- mantiene `_history` en memoria;
- procesa una cola en un thread;
- no muestra persistencia durable del grafo en esta composicion.

`McpServer`:

- descubre metodos `@skill`;
- genera schemas;
- despacha la llamada por RPC;
- soporta notificaciones de progreso;
- expone CORS `*`;
- no muestra, en la porcion inspeccionada, una politica por skill ligada a estado
  fisico, rol o zona.

Skills G1:

- `move`: publica velocidad directa por RPC a la conexion;
- `execute_arm_command`;
- `execute_mode_command`;
- `navigate_with_text`;
- `tag_location`;
- `stop_navigation`;
- `speak`.

**Limitacion critica.** El prompt indica al LLM que estime duracion y velocidad. Esto
mezcla intencion semantica con control temporal de movimiento.

## Voz

**Hecho confirmado.**

- `SpeakSkill` usa `OpenAITTSNode` y `SounddeviceAudioOutput`.
- Existe `WhisperNode`, que prefiere `openai-whisper` y cae a `faster-whisper`.
- El valor por defecto es modelo `base`, idioma `en`.
- `stream/audio/pipelines.py` conecta microfono, normalizacion, `KeyRecorder`,
  Whisper y salida, pero no esta incluido en los blueprints G1 agentic reales.
- La captura usa pulsar Enter, no VAD ni wake word.

Por tanto, G1 puede hablar desde el skill, pero "escuchar siempre y conversar por
turnos" no es una capacidad integrada.

## Percepcion visual

### Memoria visual

`SpatialMemory`:

- toma `color_image`;
- busca TF `world -> base_link`;
- almacena frames y pose;
- calcula embeddings CLIP;
- usa ChromaDB y `VisualMemory`;
- puede borrar la DB al iniciar porque `new_memory=True` por defecto;
- guarda ubicaciones nombradas en una lista en memoria.

Es una memoria de lugares visuales, no un registro normalizado de entidades.

### Tracking

`ObjectTracking` y `ObjectTracker2D` usan tracking iniciado por bbox, CSRT y
geometria/TF. El detector YOLO base llama a `model.track(..., persist=True)`.
`YoloPersonDetector` usa BoT-SORT.

### Detectores disponibles

- `Yolo2DDetector`: YOLO11n, closed-set.
- `YoloPersonDetector`: YOLO11n-pose, persona.
- `Yoloe2DDetector`: YOLOE con modo prompt o vocabulario interno.
- `Detection3DModule`: fusiona bbox 2D con pointcloud y transform.

**Hecho confirmado.** YOLOE existe en el repositorio, pero el blueprint
`unitree-g1-detection` selecciona `YoloPersonDetector`.

### Bases de objetos

Hay dos caminos diferentes:

- `moduleDB.py` contiene `ObjectDBModule`, con acumulacion simple y metodos
  incompletos como `lookup()` que retorna una lista vacia.
- `objectDB.py` contiene una clase `ObjectDB` mas reciente, con objetos pendientes,
  promocion por numero de detecciones, TTL y matching por track/distancia.

**Hecho confirmado.** El blueprint G1 importa `ObjectDBModule`, no la clase
`ObjectDB` mas estructurada. La mejora existe en codigo, pero no esta conectada aqui.

## Navegacion y personas dinamicas

El stack nativo no es solo un A* estatico:

- `TerrainAnalysis` acepta parametros de decay, clearing y borrado de obstaculos
  dinamicos;
- `TerrainMapExt` acumula con decay;
- `SimplePlanner` reconstruye el costmap desde el mapa extendido, evitando acumular
  indefinidamente;
- el mapa local rapido agrega obstaculos antes de la siguiente reconstruccion;
- hay deteccion de atasco que reduce inflacion.

Esto corrige una simplificacion importante: DimOS ya intenta manejar dinamica
geometrica.

**Limitacion.** No hay en el blueprint nav una capa semantica explicita de personas,
velocidad de tracks, prediccion ni zonas proxemicas. Un conjunto de puntos de una
persona sigue siendo geometria anonima hasta que decae.

## Simulacion frente a real

| Capacidad | Real | Simulacion |
|---|---|---|
| G1 WebRTC | si | no |
| G1 DDS high-level | si | no |
| FAST-LIO2 Mid-360 | si | no en `unitree_g1_nav_sim` |
| Unity nav | no | si |
| MuJoCo `G1SimConnection` | no | si |
| `ReplanningAStarPlanner` que satisface interfaz nav | no en basic real | si en basic sim |
| camara webcam del host | si | omitida por `global_config.simulation` |
| agente/MCP | blueprint real y sim | ambos |

## Dependencias y acoplamientos

- modelos por LFS y rutas de datos DimOS;
- binarios nativos compilados con Nix;
- GTSAM/PCL para PGO;
- Open3D para puntos;
- Ultralytics para YOLO/YOLOE y tracking;
- ChromaDB/CLIP legado para memoria;
- OpenAI para agente y TTS por defecto;
- Unitree WebRTC no oficial y SDK2/CycloneDDS;
- Rerun actualmente requerido desde core segun comentario en `pyproject.toml`;
- autoconnect por nombre y tipo de stream, sensible a remappings.

## Diagnostico operacional

| Fallo probable | Causa |
|---|---|
| skill nav no construye o no actua | falta implementacion de `NavigationInterfaceSpec` |
| objeto 3D desplazado | calibracion de camara o TF incorrecto |
| "pared" de persona | puntos dinamicos llegan a mapa persistente o decay excesivo |
| movimiento inesperado | WebRTC, DDS, teleop y agente sin arbitro comun |
| memoria perdida/borrada | `new_memory=True` o lista de tags no persistida |
| agente bloqueado | llamada remota o tool sin timeout operacional adecuado |
| voz entiende ingles | `WhisperNode` usa `language="en"` por defecto |
| mapa salta | loop closure incorrecto o consumidores que usan `map` como frame continuo |
| planner atraviesa/evita mal | altura, inflacion o montaje G1 mal calibrados |

## Conclusion

DimOS proporciona un conjunto serio de baselines y adaptadores, pero el sistema G1
agentico real es una coleccion de capacidades parcialmente conectadas. La propuesta
propia debe preservar los algoritmos que ya resuelven bien problemas concretos y
redisenar contratos, autoridad, estado, memoria y evaluacion.
