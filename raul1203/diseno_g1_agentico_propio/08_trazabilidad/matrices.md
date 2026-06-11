# Matrices de trazabilidad y reutilizacion

Ultima modificacion: 2026-06-11 11:46:53 -05 -0500

## Matriz central

Las mejoras son **hipotesis de diseño** hasta que las metricas indicadas se
midan en el mismo hardware y escenario.

| Subsistema | DimOS actual | Limitacion | Mantener | Reemplazar | Extender | Tecnologia candidata | Mejora esperada | Metrica | Riesgo |
|---|---|---|---|---|---|---|---|---|---|
| Blueprint G1 | Blueprints separados de full, nav y detection | La composicion agentica no integra nav real | Autoconnect y modulos | Ninguno inicialmente | Blueprint integrado por contratos | DimOS | Historia extremo a extremo | Build, arranque, salud | Colisiones de wiring |
| Agente | McpClient + LangGraph, GPT-4o | Estado en memoria y movimiento directo | MCP para tools de alto nivel | Prompt de duraciones | Orquestador persistente | LangGraph/checkpoints o FSM propia | Reanudacion y control | Task success, unsafe rate | Complejidad |
| Skills | `@skill` con schema | Resultado principalmente textual | Decorador y RPC | Tool `move` para LLM | Resultado, cancelacion, timeout | Contratos propios | Efectos verificables | Calls validas, cancelacion | Ruptura API |
| MCP | HTTP local, CORS amplio | Sin politica fisica evidente | Descubrimiento | Exposicion directa | Auth, scopes y allowlist | MCP + gateway propio | Menor superficie | Accesos rechazados | Configuracion |
| Voz entrada | WhisperNode/KeyRecorder no integrado | Push por Enter, sin VAD/AEC | faster-whisper disponible | Delimitacion manual final | VAD, AEC, barge-in | Silero VAD | Uso natural | WER, p95, stop recall | Ruido |
| Voz salida | SpeakSkill con TTS remoto | Red y falta de duplex | Skill de habla | Dependencia unica remota | Fallback local/pregrabado | Piper sujeto a licencia | Operacion degradada | Latencia, MOS | GPL/voz |
| Camara | Webcam host | Sin profundidad y montaje real | Interfaces de imagen | Webcam como sensor principal | RGB-D calibrada | D455/D455f o ZED 2i | Localizacion 3D | Error profundidad | Iluminacion |
| Deteccion cerrada | YOLO11n | Vocabulario limitado | Baseline de seguridad | Ninguno | Clases y benchmark propio | RT-DETR alterno | Recall estable | mAP, p95 | Licencias/pesos |
| Deteccion abierta | Adaptador YOLOE existe | No integrado al G1 | Adaptador si supera tests | Ninguno | Ejecucion bajo demanda | YOLOE, YOLO-World, Grounding DINO, OWLv2 | Consultas abiertas | Recall abierto, VRAM | Latencia |
| Segmentacion | No central en blueprint | Caja mezcla fondo/objeto | Caja como baseline | Ninguno | Mascara selectiva | SAM 2 | Mejor pose 3D | Error 3D vs coste | GPU |
| Tracking 2D | BoT-SORT en persona | ID efimero | BoT-SORT baseline | Ninguno | Comparativa | ByteTrack | Robustez medida | HOTA, IDF1 | Oclusion |
| Tracking 3D | Fusion puntual | Persistencia global insuficiente | Detection3D | ID 2D como entidad | Filtro 3D con covarianza | Implementacion propia | Dinamicos coherentes | Error pose/velocidad | Asociacion |
| LIO | FAST-LIO2 + Mid-360 | Rendimiento real no caracterizado | Baseline completo | Ninguno | Calidad y metricas | FAST-LIVO2, Point-LIO | Menor deriva en casos limite | ATE/RPE, p95 | Calibracion |
| Cierre de lazo | PGO GTSAM + ICP | Falsos cierres no medidos | PGO | Ninguno | Validacion y relocalizacion | RTAB-Map candidato | Persistencia global | Precision de cierres | Saltos |
| Reconstruccion | Nube/voxel + Open3D disponible | Sin producto semantico denso | Mapa voxel para MVP | Ninguno | TSDF por zonas | Open3D, Voxblox, nvblox | Superficies utiles | Error, memoria, W | Sobrecarga |
| Objetos dinamicos | Parametros de decay/clearing | Modelo de entidad no unificado | Mapa local | Fusion en mapa estatico | Tracks y TTL | Capa propia/Nav2 | Menos contaminacion | Obstaculos fantasma | Falsos negativos |
| Memoria visual | SpatialMemory + Chroma | Lugares en RAM y procedencia limitada | Embeddings/frames como evidencia | Vector store como verdad unica | Entidades espaciales versionadas | PostGIS + pgvector | Consulta persistente | Recall@k, conflictos | Migracion |
| DB de objetos | ObjectDBModule incompleto | Lookup vacio y flujo fragil | Ideas de TTL de implementacion alterna | Modulo activo | Contrato transaccional | PostgreSQL/SQLite | Integridad | Tests CRUD/fusion | Carga |
| Navegacion agentica | NavigationSkillContainer | Falta proveedor real en full G1 | Skill conceptual | Acoplamiento directo | NavigationExecutive | Propio | Cierra interfaz | Build y task success | Semantica |
| Plan global | SimplePlanner/FarPlanner | Dinamicos y escala por medir | SimplePlanner baseline | Ninguno | Benchmarks | Nav2 Smac candidato | Rutas mas robustas | SPL, tiempo plan | Integracion ROS |
| Plan local | PathFollower DimOS | Conducta social no medida | Baseline | Ninguno | Criticos/capas | Nav2 MPPI, DWB, TEB | Menos oscilacion | SCT, separacion | Modelo G1 |
| Arbitraje | MovementManager teleop/nav | No es supervisor integral | Cancelacion por teleop | Ninguno | Prioridad, TTL, salud | Supervisor propio | Autoridad unica | Violaciones, stop p95 | Falla comun |
| Control G1 | WebRTC, DDS high-level, low-level | Varias rutas posibles | SDK2/DDS high-level | Multiples rutas activas | Adaptador seguro | Unitree SDK2 | Menor ambiguedad | Comandos rechazados | Firmware |
| Seguridad | Timeouts parciales | No hay frontera agentica integral | Timeouts existentes | Confianza en LLM | Watchdog e invariantes | Supervisor propio | Fallo a estado seguro | Matriz de fallos | Afirmacion excesiva |
| Observabilidad | Logs, run registry, Rerun | Correlacion multimodal limitada | Rerun y logs | Ninguno | IDs, MCAP y trazas | MCAP + OpenTelemetry | Reproduccion | Completitud, drops | Disco |
| Evaluacion | Tests de software | Sin benchmark integrado G1 | Pytest/replay | Demo como evidencia | Escenarios/ablaciones | evo, SPL, SCT | Afirmaciones comparables | IC, effect size | Ground truth |

## Matriz de reutilizacion

| Componente reutilizado | Proyecto/licencia | Uso previsto | Modificacion propia | Aporte academico |
|---|---|---|---|---|
| Modulos, streams y blueprints | DimOS, licencia del repo | Composicion y transporte | Contratos y blueprint integrado | Arquitectura evaluable |
| McpServer/McpClient | DimOS + MCP | Tools cognitivas | Gateway, scopes y persistencia | Frontera de autoridad |
| `@skill` | DimOS | Descubrimiento | Resultado estructurado y cancelacion | Semantica verificable |
| FAST-LIO2 wrapper | DimOS/FAST-LIO2 | Odometria baseline | Calidad, versionado y benchmark | Estudio integrado |
| PGO GTSAM/PCL | DimOS | Correccion global | Validacion de cierres | Efecto sobre memoria |
| Terrain/map/nav stack | DimOS | Navegacion baseline | NavigationExecutive y capas dinamicas | Comparativa reproducible |
| MovementManager | DimOS | Entrada de nav/teleop | Integracion bajo supervisor | Arbitraje medible |
| Unitree SDK2/DDS | Unitree, BSD-3-Clause | Control alto nivel | Adaptador y politica | No es aporte propio |
| YOLOE adapter | DimOS/YOLOE | Consulta abierta | Scheduling y benchmark | Estrategia por niveles |
| BoT-SORT | MIT | Tracking 2D | Fusion 3D y TTL | Modelo mundo dinamico |
| Open3D | MIT | TSDF/procesamiento | Reconstruccion selectiva | Ablacion coste-beneficio |
| Rerun | Apache-2.0 | Visualizacion | Layout y entidades del sistema | Reproducibilidad |
| Whisper/faster-whisper | MIT | ASR | Pipeline, confianza y stop | Interaccion evaluada |
| Silero VAD | MIT | Deteccion de voz | Umbrales y barge-in | Corpus G1 |
| PostgreSQL/PostGIS | PostgreSQL/GPL-2.0-or-later | Memoria espacial | Esquema y fusion | Memoria con procedencia |
| pgvector | PostgreSQL License | Embeddings | Ranking espacio-vector | Ablaciones de memoria |
| MCAP | Apache-2.0 | Episodios | Manifiesto e IDs | Paquete de replicacion |
| OpenTelemetry | Apache-2.0 | Trazas/metricas | Taxonomia robotica | Diagnostico causal |

Las licencias se basan en los repositorios oficiales consultados. Antes de
distribuir una implementacion se debe revisar la combinacion exacta de codigo,
pesos y modelos.

## Hecho, inferencia y propuesta

| Afirmacion | Tipo | Evidencia/validacion |
|---|---|---|
| G1 full no incluye el stack nav onboard | Hecho observado | Blueprints locales |
| La skill de navegacion no tiene proveedor evidente en G1 full | Hecho observado | Tipos y composicion local |
| Esto impedira build o ejecucion de esa skill | Inferencia | Test de blueprint requerido |
| FAST-LIO2 debe mantenerse como baseline | Propuesta | Benchmark antes de reemplazo |
| YOLOE sera el mejor detector | No afirmado | Debe compararse |
| RGB-D reducira error 3D | Hipotesis | Dataset y error mediano |
| Supervisor independiente reducira riesgo | Hipotesis de arquitectura | Inyeccion de fallos |
| PostGIS + pgvector mejora memoria | Hipotesis | Recall/conflictos/latencia |

## Cobertura de requisitos

| Requisito | Documento |
|---|---|
| Diagnostico actual | [`arquitectura_actual.md`](../01_diagnostico_dimos/arquitectura_actual.md) |
| Reutilizar/reemplazar/extender | [`brechas_y_reutilizacion.md`](../01_diagnostico_dimos/brechas_y_reutilizacion.md) |
| Arquitectura total | [`vision_general.md`](../02_arquitectura_propuesta/vision_general.md) |
| Diagramas | [`diagramas.md`](../02_arquitectura_propuesta/diagramas.md) |
| Agente | [`agente_y_orquestacion.md`](../03_agente_y_voz/agente_y_orquestacion.md) |
| Voz | [`voz_e_interaccion.md`](../03_agente_y_voz/voz_e_interaccion.md) |
| Vision | [`percepcion_visual.md`](../04_percepcion_y_mapeo/percepcion_visual.md) |
| LiDAR/SLAM | [`lidar_slam_y_reconstruccion.md`](../04_percepcion_y_mapeo/lidar_slam_y_reconstruccion.md) |
| Personas | [`personas_y_entornos_dinamicos.md`](../04_percepcion_y_mapeo/personas_y_entornos_dinamicos.md) |
| Memoria | [`memoria_semantica.md`](../05_memoria_y_navegacion/memoria_semantica.md) |
| Navegacion | [`navegacion.md`](../05_memoria_y_navegacion/navegacion.md) |
| Control/seguridad | [`control_g1_y_seguridad.md`](../06_control_y_observabilidad/control_g1_y_seguridad.md) |
| Observabilidad | [`observabilidad_y_datos.md`](../06_control_y_observabilidad/observabilidad_y_datos.md) |
| Evaluacion/paper | [`evaluacion_y_paper.md`](../07_evaluacion_y_roadmap/evaluacion_y_paper.md) |
| MVP/roadmap | [`mvp_y_roadmap.md`](../07_evaluacion_y_roadmap/mvp_y_roadmap.md) |

