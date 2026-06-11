# Brechas, fortalezas y reutilizacion

Ultima modificacion: 2026-06-11 11:46:53 -05 -0500

## Fortalezas conceptuales

### Modularidad por procesos

**Hecho confirmado.** Los modulos DimOS encapsulan ciclo de vida, streams, RPC y
workers. Esta separacion es valiosa para aislar GPU, drivers nativos y fallos.

**Mantener conceptualmente:** componentes sustituibles, mensajes tipados, ciclo de
vida explicito y despliegue por procesos.

**Extender:** health checks, readiness, supervision tree, restart policy y contratos
versionados.

### Integracion directa con hardware

FAST-LIO2 habla con Livox SDK2 sin ROS, y los adaptadores G1 cubren WebRTC, DDS
high-level y DDS low-level.

**Mantener:** adaptadores finos que oculten SDKs.

**Reemplazar:** acceso concurrente sin arbitraje. El nuevo sistema debe exponer una
sola interfaz `LocomotionCommand` y una sola maquina de estados.

### Navegacion 3D practica

DimOS combina LIO, PGO, terreno, mapa con decay, A*, paths precomputados y seguimiento.
No conviene descartarlo antes de medirlo.

**Mantener como baseline:** todo el nav onboard.

**Extender:** API de goals, estados, cancelacion, reasons de fallo, capas humanas y
registro de planner decisions.

### Skills y MCP

Los skills convierten metodos tipados en tools y MCP desacopla descubrimiento del
agente.

**Mantener:** schemas generados, interfaz estandar y tool results estructurados.

**Reemplazar:** exposicion directa de `move` y permisos uniformes. MCP no debe ser el
supervisor de seguridad.

### Replay y visualizacion

El ecosistema tiene transportes, logs por run y Rerun.

**Mantener:** visualizacion multimodal y reproduccion.

**Extender:** formato de episodio estable, correlacion de trazas y metadatos de
experimento.

## Brechas de integracion

| Brecha | Evidencia local | Consecuencia |
|---|---|---|
| Agentic y nav separados | blueprints distintos | lenguaje no llega a goal real por contrato unico |
| Deteccion separada | `unitree-g1-detection` aparte | memoria/agente no consultan objetos 3D |
| Audio de entrada separado | pipeline fuera de G1 | no hay conversacion hablada integrada |
| Control duplicado | WebRTC, DDS high y DDS low | conflicto de autoridad |
| TF heterogeneo | `world/base_link`, `map/body`, camara | errores silenciosos de pose |
| Memoria no unificada | frames CLIP, tags RAM, DBs de objetos distintas | consultas inconsistentes |
| Seguridad parcial | timeout DDS y mux teleop, sin policy global | el LLM conserva demasiado poder |
| Estado del agente volatil | `_history` en memoria | reinicio pierde tarea y contexto |

## Componentes incompletos o experimentales

### `ObjectDBModule`

- fusion por nombre y solapamiento;
- `lookup()` vacio;
- impresiones de depuracion;
- logica `len(obj.detections)` inconsistente con un contador entero;
- no muestra persistencia.

**Decision:** no adoptarlo como memoria final. Reutilizar conversion 2D-3D y tipos,
pero escribir una capa de entidades nueva.

### `ObjectDB`

Tiene mejor diseño: pending/permanent, TTL, track ID y distancia. Aun faltan:

- persistencia;
- covarianza;
- historial de observaciones;
- relaciones topologicas;
- estado dinamico;
- embeddings multimodales;
- merge/split auditables.

**Decision:** usarlo como referencia de reglas de promocion, no como base de datos
definitiva.

### `SpatialMemory`

- usa modulos en `agents_deprecated`;
- guarda frames y metadatos;
- tiene riesgo de borrar al iniciar;
- tags no se observan persistidos en DB;
- la posicion depende de `world -> base_link`.

**Decision:** conservar como baseline de place recognition. No confundirlo con memoria
de objetos.

### Audio

- pipeline reactivo reutilizable;
- STT no streaming real de turnos;
- `KeyRecorder` exige teclado;
- idioma ingles por defecto;
- no hay cancelacion de TTS por barge-in.

**Decision:** reutilizar nodos de dispositivo si son estables; redisenar turn-taking.

## Que desarrollar desde cero

"Desde cero" aqui significa diseno y codigo de integracion propios, no reimplementar
algoritmos cientificos consolidados.

1. **Modelo de contratos:** timestamp, frame, covariance, provenance, schema version.
2. **Task executive:** maquina de estados durable con goals y postcondiciones.
3. **Skill gateway:** autorizacion, validacion, rate limit, timeout e idempotencia.
4. **Safety supervisor:** limites, zonas, estado G1, heartbeat y e-stop.
5. **Entity memory:** esquema, fusion, historial, TTL y consultas.
6. **Dynamic world model:** tracks y prediccion separados de SLAM.
7. **Semantic grounding:** de consulta a candidato, evidencia y goal alcanzable.
8. **Episode recorder/evaluator:** manifest, MCAP, trazas y metricas.

## Que reutilizar responsablemente

| Componente | Forma de reutilizacion | Condicion |
|---|---|---|
| FAST-LIO2 wrapper | servicio de odometria | benchmark y calibracion Mid-360 |
| PGO DimOS | baseline de loop closure | test de falsos cierres |
| TerrainAnalysis | mapa local | validar decay con personas |
| SimplePlanner/local planner | baseline nav | exponer estado y razones de fallo |
| Unitree SDK adapters | capa hardware | exclusividad de autoridad |
| YOLO/YOLOE wrappers | baseline vision | revisar pesos/licencias |
| conversion bbox-pointcloud | estimacion 3D | calibracion y error robusto |
| MCP schemas | integracion tool | gateway de seguridad delante |
| Rerun | debug de desarrollo | grabacion independiente |

## Reemplazos propuestos

| Actual | Reemplazo o extension | Motivo |
|---|---|---|
| `move(x,y,yaw,duration)` como skill | `navigate_to`, `follow`, `stop` declarativos | elimina control temporal del LLM |
| lista de tags en RAM | tabla `places` persistente | continuidad entre sesiones |
| Chroma de frames como memoria general | PostGIS + pgvector + observaciones | consultas estructuradas y espaciales |
| wiring solo por nombre/tipo | topics/servicios versionados y manifest | cambios auditables |
| `_history` volatil | checkpointer durable | reinicio y human-in-the-loop |
| geometry-only dynamic handling | tracks semanticos + costmap temporal | personas no son paredes |
| logs separados | `episode_id` y `trace_id` comunes | diagnostico causal |

## Riesgo de sobrearquitectura

**Inferencia tecnica.** Crear demasiados servicios antes de tener datasets puede
ocultar problemas basicos de calibracion.

Por eso el orden recomendado es:

1. sensores y clocks;
2. control seguro;
3. localizacion/mapa;
4. navegacion geometrica;
5. dinamica humana;
6. entidades;
7. lenguaje.

## Criterio de aceptacion de reutilizacion

Un componente se considera reutilizable solo si:

- tiene licencia compatible;
- puede ejecutarse detras de un contrato estable;
- sus fallos son observables;
- puede reproducirse con datos grabados;
- cumple presupuesto de recursos;
- tiene un owner y una estrategia de versionado;
- no obtiene autoridad mayor que la necesaria.

La matriz completa esta en [matrices.md](../08_trazabilidad/matrices.md).
