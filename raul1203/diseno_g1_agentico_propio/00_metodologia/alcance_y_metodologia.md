# Alcance y metodologia

Ultima modificacion: 2026-06-11 11:46:53 -05 -0500

## Pregunta de investigacion

Como construir un sistema propio para Unitree G1 que ejecute tareas semanticas en un
almacen de forma modular, medible y segura, aprovechando software existente sin
confundir integracion con contribucion original?

## Alcance

Se estudian estos subsistemas:

- interfaz humana por voz;
- agente, tools y planificacion de tareas;
- camaras, deteccion, segmentacion y tracking;
- lidar, odometria, SLAM, loop closure y reconstruccion;
- memoria semantica de objetos y lugares;
- navegacion geometrica y semantica;
- personas y obstaculos dinamicos;
- control high-level y low-level del G1;
- seguridad, observabilidad, reproduccion y evaluacion.

El MVP termina en locomocion segura y verificacion de tareas. Manipulacion, control
whole-body avanzado y aprendizaje online quedan fuera.

## Corpus local inspeccionado

**Hecho confirmado.** Se inspecciono el codigo en el commit:

```text
06606d6f6ab767b659c597cc5bfe8e2a4eb56525
docs(g1): add real robot architecture analysis
```

Las areas principales fueron:

- `dimos/robot/unitree/g1/blueprints/`
- `dimos/robot/unitree/g1/connection.py`
- `dimos/robot/unitree/g1/effectors/`
- `dimos/robot/unitree/g1/skill_container.py`
- `dimos/hardware/sensors/lidar/fastlio2/`
- `dimos/navigation/nav_stack/`
- `dimos/navigation/movement_manager/`
- `dimos/perception/`
- `dimos/agents/mcp/`
- `dimos/agents/skills/`
- `dimos/stream/audio/`
- `dimos/visualization/rerun/`
- `pyproject.toml`

Tambien se revisaron los 31 Markdown de `../analisis_g1_real_dimos/`. Esa
documentacion se uso como indice de preguntas, pero las conclusiones de este directorio
se volvieron a contrastar con el codigo.

## Restricciones metodologicas

- No se ejecuto el G1, simulacion, replay ni blueprints.
- No se midio latencia real ni consumo de GPU; toda cifra no medida localmente se
  identifica como referencia externa o presupuesto.
- No se declara una tecnologia "mejor" sin especificar escenario, hardware y metrica.
- Una capacidad presente en el repositorio no se considera integrada hasta demostrar
  un flujo extremo a extremo en un blueprint y un contrato compatible.
- Una funcion que existe solo en simulacion no se extrapola automaticamente a hardware.
- Las licencias se tratan como requisito de seleccion, no como detalle posterior.

## Clasificacion de evidencia

| Etiqueta | Significado | Ejemplo |
|---|---|---|
| Hecho confirmado | visible en codigo o fuente primaria | `unitree_g1_nav_onboard` instancia FAST-LIO2 |
| Inferencia tecnica | efecto probable, no medido | dos autoridades de velocidad pueden competir |
| Propuesta propia | decision para el nuevo sistema | gateway determinista de skills |
| Candidato experimental | debe superar benchmark | cambiar SimplePlanner por Nav2 MPPI |

## Preguntas aplicadas a cada subsistema

1. Que problema resuelve.
2. Como lo resuelve DimOS.
3. Que archivos, clases y blueprints intervienen.
4. Que sensores, streams, modelos y dependencias utiliza.
5. Que esta conectado para G1 real.
6. Que existe solo en simulacion o aislado.
7. Que limitaciones y fallos operacionales tiene.
8. Que concepto conviene conservar.
9. Que debe reemplazarse o extenderse.
10. Que alternativas son candidatas.
11. Que costos de computo, integracion y licencia introducen.
12. Que propuesta se adopta.
13. Que metrica decide si mejora.
14. Que dificultad y riesgo tiene.

Para evitar repetir catorce encabezados en cada archivo, las respuestas aparecen en
tablas de diagnostico, comparaciones y fichas de decision.

## Criterios de comparacion

### Arquitectura

- claridad del contrato;
- aislamiento de fallos;
- determinismo;
- compatibilidad con replay;
- posibilidad de sustituir un componente;
- trazabilidad de una decision hasta un comando.

### Tiempo real y recursos

- latencia p50, p95 y p99;
- jitter;
- frecuencia sostenida;
- CPU, RAM, GPU y VRAM;
- energia y temperatura;
- degradacion cuando se pierde red.

### Percepcion

- precision, recall, mAP o HOTA segun la tarea;
- error de posicion 3D;
- estabilidad de identidad;
- rendimiento por condicion de luz, oclusion y movimiento;
- clases de almacen no vistas;
- licencia y necesidad de entrenamiento.

### Navegacion y seguridad

- success rate;
- SPL y tiempo;
- distancia minima a personas;
- colisiones y contactos;
- numero de replans, stops y recuperaciones;
- deriva y error de relocalizacion;
- intervenciones humanas.

### Memoria y agente

- precision de la entidad recuperada;
- respuesta correcta a consultas espaciales y temporales;
- llamadas de tool validas;
- tareas completadas sin aclaracion innecesaria;
- recuperacion tras timeout, reinicio o perdida del LLM.

## Hardware de referencia

La documentacion distingue dos perfiles:

| Perfil | Uso | Supuesto |
|---|---|---|
| Edge | seguridad, drivers, control, mapa local | computadora onboard, sin depender de Internet |
| Externo | LLM grande, vision pesada, entrenamiento | laptop/workstation con GPU y red local dedicada |

No se fija un Jetson concreto porque la variante exacta del G1 y el presupuesto no
estan confirmados. Las decisiones de modelo se expresan como presupuestos de VRAM y
latencia que deben medirse en el hardware comprado.

## Protocolo de seleccion de tecnologias

1. Definir dataset de almacen y ground truth.
2. Ejecutar baseline DimOS o equivalente sin cambios de algoritmo.
3. Integrar un candidato detras del mismo contrato.
4. Medir calidad, latencia y recursos en el hardware objetivo.
5. Revisar licencia y mantenibilidad.
6. Adoptar solo si supera el umbral acordado sin degradar seguridad.

Ejemplo: YOLOE no reemplaza al detector actual por ser mas reciente. Debe mejorar
recall de clases no vistas o consultas referenciales con una latencia compatible con
el ciclo de percepcion.

## Reproducibilidad

Cada experimento debe guardar:

- commit de software y hashes de modelos;
- configuracion resuelta;
- calibraciones;
- reloj y offsets temporales;
- version de firmware/SDK;
- hardware y modo de potencia;
- seed cuando aplique;
- MCAP del episodio;
- eventos del agente y skills;
- metricas derivadas y script de evaluacion.

## Fuentes externas

Solo se usaron papers, repositorios oficiales, documentacion oficial y fabricantes.
La lista, licencia reportada y fecha de consulta se encuentran en
[fuentes.md](../08_trazabilidad/fuentes.md).
