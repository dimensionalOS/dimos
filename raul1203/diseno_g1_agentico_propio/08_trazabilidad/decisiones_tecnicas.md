# Registro de decisiones tecnicas

Ultima modificacion: 2026-06-11 11:46:53 -05 -0500

## Convencion

- **Aceptada:** forma parte de la arquitectura propuesta.
- **Candidata:** requiere benchmark o validacion.
- **Pospuesta:** fuera del MVP.
- **Rechazada:** no satisface la frontera actual.

## ADR-001: el LLM no controla velocidad

**Estado:** Aceptada.

**Contexto:** la skill G1 actual permite `move(x, y, yaw, duration)` y el prompt
pide estimar duracion.

**Decision:** el catalogo agentico solo expone objetivos de tarea. Navegacion y
supervisor producen el comando.

**Consecuencia:** menor flexibilidad inmediata, mejor verificabilidad y
cancelacion.

## ADR-002: FAST-LIO2 es baseline

**Estado:** Aceptada.

**Contexto:** ya esta integrado con Mid-360, PGO y stack onboard.

**Decision:** instrumentar antes de reemplazar.

**Alternativas:** FAST-LIVO2 y Point-LIO quedan candidatas para fallos
especificos.

## ADR-003: RGB-D como rango visual primario

**Estado:** Candidata condicionada a hardware.

**Decision:** usar profundidad RGB-D para objetos cercanos y LiDAR como señal
complementaria. Monocular es fallback/prior.

**Razon:** evita tratar profundidad monocular como medida metrica de seguridad.

## ADR-004: vision de dos niveles

**Estado:** Aceptada.

**Decision:** detector rapido cerrado permanente para seguridad; detector
abierto bajo demanda para lenguaje.

**Candidato inicial:** YOLOE por adaptador existente, sujeto a benchmark.

## ADR-005: SAM 2 solo selectivo

**Estado:** Candidata.

**Decision:** ejecutar segmentacion cuando la mascara tenga una postcondicion
clara: mejorar pose, separar instancias o generar anotacion.

## ADR-006: memoria espacial relacional-vectorial

**Estado:** Aceptada como destino; despliegue puede comenzar en SQLite.

**Decision:** PostGIS para geometria e integridad, pgvector para similitud.

**Rechazado:** vector store como unica fuente de verdad.

## ADR-007: observacion inmutable, entidad versionada

**Estado:** Aceptada.

**Razon:** conserva procedencia y permite corregir asociaciones sin borrar
historia.

## ADR-008: personas fuera del mapa estatico

**Estado:** Aceptada.

**Decision:** tracks con covarianza y TTL alimentan una capa dinamica.

**Consecuencia:** requiere fusion 3D y politica de caducidad.

## ADR-009: adaptar la navegacion antes de migrarla

**Estado:** Aceptada.

**Decision:** crear `NavigationExecutive` sobre el stack DimOS actual.

**Alternativa candidata:** Nav2 MPPI si supera el baseline en los mismos
escenarios.

## ADR-010: una sola autoridad Unitree activa

**Estado:** Aceptada.

**Decision:** DDS/SDK2 de alto nivel para el MVP; WebRTC queda en compatibilidad
o teleop segun el banco. Bajo nivel queda fuera.

**Razon:** evitar comandos concurrentes y reducir superficie de fallo.

## ADR-011: supervisor independiente del agente

**Estado:** Aceptada.

**Decision:** proceso de seguridad con watchdog, mapa local y estado G1. No
depende de Internet ni de MCP.

## ADR-012: MCP solo para plano cognitivo

**Estado:** Aceptada.

**Decision:** tools locales autenticadas, scopes y allowlist. MCP no transporta
el lazo de control.

## ADR-013: MCAP + OpenTelemetry

**Estado:** Aceptada.

**Decision:** MCAP para señales de alta tasa; OpenTelemetry para trazas y
metricas correlacionadas.

**Razon:** separar replay robotico de causalidad de servicios.

## ADR-014: Rerun permanece

**Estado:** Aceptada.

**Decision:** extender la integracion existente. Foxglove solo entra si aporta
una capacidad medida que Rerun no cubre.

## ADR-015: voz critica local

**Estado:** Aceptada.

**Decision:** `stop` se reconoce por ruta local cerrada y existe boton/canal
fisico. El ASR abierto puede ser local o remoto para otras frases.

## ADR-016: Piper no se adopta sin revision

**Estado:** Candidata.

**Contexto:** el proyecto activo usa GPL-3.0, mientras el repositorio original
esta archivado.

**Decision:** revisar obligaciones de distribucion; mantener TTS remoto y
mensajes pregrabados como alternativas.

## ADR-017: sin reconocimiento facial en MVP

**Estado:** Pospuesta.

**Razon:** no es necesario para navegacion semantica y aumenta riesgos de
privacidad, sesgo y gobernanza.

## ADR-018: sin control de cuerpo completo agentico

**Estado:** Pospuesta.

**Razon:** la ruta de 29 motores a 500 Hz tiene requisitos y peligros distintos
de la navegacion de alto nivel.

## ADR-019: no afirmar seguridad certificada

**Estado:** Aceptada.

**Decision:** reportar arquitectura, pruebas y limites. Una matriz de fallos no
equivale a certificacion.

## ADR-020: adopcion por evidencia

**Estado:** Aceptada.

Toda tecnologia candidata debe declarar:

- baseline;
- escenario;
- configuracion;
- metrica;
- presupuesto de recursos;
- criterio de adopcion;
- licencia;
- resultado negativo si falla.

## Decisiones abiertas

| ID | Pregunta | Evidencia necesaria |
|---|---|---|
| O-01 | D455f o ZED 2i | Prueba montada en G1 |
| O-02 | GPU/computadora | Perfil completo de pipelines |
| O-03 | PostgreSQL a bordo o servicio | Latencia, disponibilidad y operacion |
| O-04 | DimOS local planner o MPPI | Escenarios dinamicos repetidos |
| O-05 | Modelo LLM local | Suite de tool use y VRAM |
| O-06 | PTP extremo a extremo | Capacidades reales de NIC/sensores |
| O-07 | Relocalizacion visual | Fallos observados de FAST-LIO2/PGO |
| O-08 | Limites de velocidad | Distancia de frenado y protocolo |

## Regla de cambio

Una decision aceptada puede cambiar, pero debe registrar nueva evidencia,
impacto sobre contratos, migracion de datos y repeticion de los benchmarks
afectados.

