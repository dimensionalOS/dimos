# Glosario

Ultima modificacion: 2026-06-11 11:46:53 -05 -0500

## Conceptos de arquitectura

| Termino | Explicacion |
|---|---|
| Modulo | proceso o componente con una responsabilidad y contratos de entrada/salida |
| Stream | secuencia tipada de mensajes, normalmente con timestamps |
| Blueprint | composicion de modulos y conexiones en DimOS |
| Contrato | esquema versionado que define datos, unidades, frames y semantica |
| Plano de datos | ruta de sensores a control que debe seguir funcionando sin LLM |
| Plano cognitivo | razonamiento, lenguaje, memoria de tarea y seleccion de skills |
| Gateway de skills | valida identidad, argumentos, precondiciones, permisos y timeout |
| Supervisor | autoridad determinista que permite, limita o rechaza acciones |
| Replay | reejecucion de datos grabados para depurar o comparar algoritmos |

## Marcos y tiempo

| Termino | Explicacion |
|---|---|
| TF | arbol de transformaciones entre sistemas de coordenadas |
| `map` | frame global corregido, puede cambiar con loop closure |
| `odom` | frame local continuo, no deberia saltar |
| `base_link`/`body` | frame ligado al cuerpo del robot |
| Extrinseca | pose rigida entre dos sensores |
| Intrinseca | parametros internos de una camara |
| Sincronizacion | relacion temporal entre muestras de sensores |
| Covarianza | representacion de incertidumbre de una estimacion |

## Percepcion y mapas

| Termino | Explicacion |
|---|---|
| Closed-set | detector limitado a clases entrenadas |
| Open-vocabulary | detector consultable con clases o texto no fijo |
| VLM | modelo que relaciona imagen y lenguaje |
| Tracking | asociacion de observaciones a una identidad a traves del tiempo |
| ReID | reidentificacion por apariencia despues de oclusion o salida de escena |
| LIO | odometria que fusiona lidar e IMU |
| SLAM | localizacion y construccion de mapa simultaneas |
| Loop closure | reconocimiento de un lugar previo para corregir deriva |
| Relocalizacion | recuperar la pose en un mapa existente |
| PGO | optimizacion de grafo de poses |
| Occupancy grid | celdas libres, ocupadas o desconocidas |
| Voxel map | grilla tridimensional |
| TSDF | volumen que integra distancias truncadas a superficies |
| ESDF | distancia euclidiana al obstaculo mas cercano, util para planificar |
| Surfel | elemento de superficie con posicion, normal y atributos |
| Costmap | mapa operacional de costos para navegacion |
| Decay | eliminacion o reduccion de evidencia con el tiempo |
| Clearing | marcado explicito de espacio que vuelve a estar libre |

## Navegacion y agente

| Termino | Explicacion |
|---|---|
| Goal geometrico | pose o region alcanzable en coordenadas del mapa |
| Goal semantico | descripcion como "estante de bebidas" |
| Grounding | convertir lenguaje o una deteccion en evidencia geometrica |
| Planner global | calcula ruta de largo alcance |
| Planner/controlador local | genera movimiento factible y evita obstaculos cercanos |
| Skill | operacion de alto nivel expuesta al agente |
| Tool calling | salida estructurada del LLM para invocar una funcion |
| MCP | protocolo para descubrir y llamar tools |
| Idempotencia | repetir una solicitud no duplica efectos peligrosos |
| Precondicion | estado que debe cumplirse antes de una accion |
| Postcondicion | evidencia que confirma el resultado |

## Evaluacion

| Termino | Explicacion |
|---|---|
| Success rate | fraccion de episodios completados |
| SPL | exito ponderado por eficiencia de longitud de ruta |
| SCT | exito ponderado por tiempo de finalizacion |
| APE | error absoluto de trayectoria |
| RPE | error relativo y deriva local |
| Ablation | experimento que elimina una parte para medir su aporte |
| Baseline | sistema de referencia reproducible |
| Sim-to-real | diferencia y transferencia entre simulacion y hardware |
| p95/p99 | percentiles que muestran colas de latencia o error |

## Distincion critica de memoria

- **Memoria conversacional:** contexto de dialogo y tarea.
- **Memoria episodica:** que ocurrio en un episodio y en que orden.
- **Memoria semantica:** entidades, atributos, relaciones y lugares.
- **Memoria geometrica:** mapas, poses y superficies.

No deben almacenarse como si fueran la misma base ni tener el mismo TTL.
