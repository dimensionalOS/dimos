# Prompts para vibecodear Yanapaq G1

Ultima modificacion: 2026-06-12 16:23:07 -05 -0500

## Prompt inicial para un chat nuevo

Usa este prompt cuando abras un chat nuevo para empezar codigo:

```text
Vamos a implementar Yanapaq G1 dentro del repo DimOS en /home/raul/dimos.

Objetivo: crear una capa nueva para Unitree G1 basada en `unitree-g1-nav-onboard`,
reutilizando DimOS tanto como sea posible. No queremos repo separado. No
modifiques blueprints originales salvo que sea estrictamente necesario.

Contexto que debes leer primero:
- raul1203/yanapaq_g1/00_contexto_minimo_agente.md
- raul1203/yanapaq_g1/README.md
- dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py
- dimos/robot/unitree/g1/blueprints/primitive/unitree_g1_onboard.py
- dimos/robot/unitree/g1/effectors/high_level/dds_sdk.py
- dimos/navigation/movement_manager/movement_manager.py

Restricciones:
- Plataforma inicial: Unitree G1 + laptop Ubuntu por Ethernet.
- Base principal: `unitree-g1-nav-onboard`.
- Blueprint objetivo: `unitree-g1-yanapaq-dev`.
- No usar ROS2 como stack principal.
- No usar servidores locales de LLM/VLM.
- OpenAI API solo para cognicion/verificacion, no para control ni seguridad.
- No exponer movimiento directo al LLM.
- Todo movimiento debe pasar luego por un SafetySupervisor.
- Empezar con cambios pequenos, testeables y aditivos.

Primera tarea: audita los archivos G1 relevantes y propon los cambios minimos
para crear el blueprint inicial Yanapaq G1 sin romper DimOS. No implementes
todavia hasta mostrarme el plan.
```

## Prompt 1: crear blueprint skeleton

```text
Implementa solo el primer paso: crea el blueprint `unitree-g1-yanapaq-dev`
basado en `unitree-g1-nav-onboard`.

Alcance:
- Crear carpeta `dimos/robot/unitree/g1/blueprints/yanapaq/`.
- Crear `unitree_g1_yanapaq_dev.py`.
- Reutilizar `_unitree_g1_onboard`, `create_nav_stack`, `MovementManager` y
  visualizacion igual que el blueprint nav actual.
- No agregar LLM, memoria, YOLOE ni OpenAI todavia.
- No modificar comportamiento de blueprints existentes.
- Actualizar registry si el repo lo requiere.
- Ejecutar pruebas minimas de import/build/listado.

Devuelveme diff, comandos corridos y riesgos.
```

## Prompt 2: SafetySupervisor

```text
Agrega un modulo `SafetySupervisor` para Yanapaq G1.

Objetivo:
- Insertarlo entre `MovementManager.cmd_vel` y `G1HighLevelDdsSdk.cmd_vel`.
- Validar stale odom/mapa, limite de velocidad, heartbeat y stop.
- Publicar `safe_cmd_vel`.
- En modo inicial, ser conservador y transparente: si falta una senal critica,
  publica cero.

No usar LLM. No agregar memoria. No romper `unitree-g1-nav-onboard`.
Incluye tests unitarios con mensajes Twist y estados simulados.
```

## Prompt 3: MissionExecutive sin LLM

```text
Crea `MissionExecutive` para Yanapaq G1 sin LLM.

Debe aceptar misiones estructuradas por RPC o skill interna:
- VERIFY_OBJECT
- SEARCH_OBJECT
- GOTO_ROOM_PLACEHOLDER

Estados minimos:
IDLE, PLAN, NAVIGATE, VERIFY, UPDATE_MEMORY, SUCCEEDED, FAILED, CANCELLED.

No debe publicar velocidad. Solo debe llamar interfaces de alto nivel:
set_goal, cancel_goal, query_memory, update_memory.
Incluye tests de transiciones.
```

## Prompt 4: memoria SQLite minima

```text
Implementa memoria SQLite minima para Yanapaq G1.

Tablas iniciales:
- objects
- observations
- rooms
- missions

Requisitos:
- SQLite WAL.
- timestamps.
- last_seen.
- estado: observed, missing, moved, retired.
- evidencia por observacion.
- API Python/RPC simple para upsert/query.

No integrar YOLOE aun. Tests con base temporal.
```

## Prompt 5: YOLOE como candidato

```text
Integra YOLOE como detector de candidatos para Yanapaq G1.

Reutiliza `Yoloe2DDetector` y/o `ObjectSceneRegistrationModule` si conviene.
No llames OpenAI todavia.

Salida esperada:
- object_candidate con label, bbox, score, pose/pointcloud si existe,
  timestamp y frame_id.

Politica:
- YOLOE no decide verdad final.
- confidence alta + consistencia 3D permite guardar evidencia.
- confidence media marca `needs_vlm_verification`.
```

## Prompt 6: OpenAI VLM verifier

```text
Agrega verificacion visual con OpenAI API para candidatos ambiguos.

Requisitos:
- Solo mandar crops/keyframes, no stream continuo.
- Timeout y presupuesto.
- Registrar prompt, modelo, tokens/costo estimado y respuesta.
- Nunca usar VLM para control, velocidad o colision.
- Respuesta estructurada: confirmed, rejected, uncertain, rationale corto.
```

## Prompt 7: agente/MCP con allowlist

```text
Integra agente/MCP solo despues de tener MissionExecutive y SafetySupervisor.

El LLM no recibe tools de movimiento directo. Solo puede pedir misiones:
- verify_object
- search_object
- cancel_mission
- summarize_memory
- speak

El MissionExecutive decide si acepta, rechaza o pide confirmacion.
```

## Prompt de revision antes de tocar hardware

```text
Haz una revision de seguridad antes de correr en G1 real.

Busca:
- paths donde el LLM pueda llegar a cmd_vel;
- comandos sin timeout;
- DDS interface hardcodeada;
- falta de StopMove en shutdown;
- dos publishers compitiendo por movimiento;
- fallos de heartbeat;
- excepciones que dejen el robot moviendose.

No implementes features nuevas. Solo findings y parches minimos.
```
