# Revision de literatura: navegacion semantica y segura en Unitree G1

Ultima modificacion: 2026-06-12 16:23:07 -05 -0500

## Alcance y criterio de busqueda

Esta revision cubre trabajos publicados o depositados entre enero de 2025 y
el 12 de junio de 2026. Se buscaron en arXiv las variantes `Unitree G1`,
`Unitree-G1`, `G1 humanoid`, `humanoid navigation`, `semantic navigation`,
`spatial memory`, `room navigation`, `human-aware navigation` y `social
navigation benchmark`. Se revisaron los articulos completos, sus tablas y
limitaciones, no solo los resumenes.

Se incluyeron:

- trabajos que ejecutan navegacion, razonamiento o seguridad en un G1 real;
- trabajos sin G1 que definen el benchmark mas pertinente para memoria,
  habitaciones o personas;
- articulos de locomocion solo cuando aportan una interfaz o garantia
  directamente util para la autonomia propuesta.

Se excluyeron como comparadores principales los trabajos centrados
exclusivamente en deportes, teleoperacion, manipulacion o imitacion de
movimientos. El corpus directo de navegacion G1 encontrado queda cubierto en
la Tabla 1; los trabajos adyacentes relevantes aparecen despues.

## Conclusion ejecutiva

El competidor mas cercano a una navegacion semantica completa es
**SysNav** [2], por su separacion entre razonamiento semantico, navegacion por
habitaciones y control. **FSR-VLN** [1] es la referencia directa para
recuperacion jerarquica de una meta en un mapa semantico construido por un G1.
**MIF** [5] es la referencia mas fuerte para memoria espacial que se adapta a
cambios. **EgoNav** [4] y **FocusNav** [6] son las referencias para movimiento
local en escenas dinamicas, mientras **SHIELD** [9] es la referencia de
seguridad formal sobre un controlador aprendido. **RAPT** [29] complementa
esa capa al detectar distribuciones anormales y diagnosticar fallos durante
el despliegue.

Sin embargo, ningun trabajo revisado demuestra conjuntamente en un G1:

1. memoria espacial transaccional que sobreviva reinicios;
2. representacion explicita de habitacion, region, entidad, vigencia y
   procedencia;
3. navegacion a un lugar nombrado con verificacion de postcondicion;
4. evaluacion social cuantitativa con personas;
5. cancelacion y recuperacion ante perdida del agente, localizacion o datos
   obsoletos;
6. un supervisor independiente que sea la unica autoridad de movimiento.

Ese es el espacio defendible de Yanapaq. El paper no deberia intentar vencer
a todos los sistemas en ObjectNav generico, sino demostrar que una mision
estrecha y operacionalmente importante se ejecuta mejor: **recordar un lugar,
recuperarlo despues de reiniciar, navegar hasta el en un G1 real, responder de
forma conservadora a una persona y terminar de forma segura ante fallos**.

## Tabla 1. Trabajos directamente relacionados con G1

| Trabajo | Evidencia en G1 | Aporte relevante | Personas | Memoria y habitaciones | Limite para comparar |
|---|---|---|---|---|---|
| FSR-VLN, 2025 [1] | G1 real; 87 instrucciones | Grafo piso-habitacion-vista-objeto y razonamiento rapido/lento | No evaluadas | Habitaciones explicitas; mapa esencialmente estatico | Reporta recuperacion de meta, no exito extremo a extremo |
| SysNav, 2026 [2] | 14 pruebas cualitativas G1 dentro de 190 pruebas reales | Razonamiento, navegacion por habitaciones y control desacoplados | Personas aparecen en algunas instrucciones | Grafo habitacion-vista-objeto y conectividad | Las 112 pruebas cuantitativas reales son en robot con ruedas |
| MerNav, 2026 [3] | Dos demostraciones cualitativas humanoides | Ciclo memoria-ejecucion-revision y recuperacion | No | Memoria corta y de tarea; se limpia entre tareas | Sin benchmark cuantitativo G1 ni persistencia entre sesiones |
| EgoNav, 2026 [4] | 1,137 m y 37.5 min reales | Navegacion visual local aprendida de datos humanos | Multitudes y cruces reales | Memoria visual rodante de corto horizonte | No usa metas semanticas ni memoria persistente |
| MIF, 2026 [5] | Oficina real de 100 m2 | Memoria multimodal actualizable y geometria segura de interaccion | Solo demostracion cualitativa | Afiliacion a cuartos y actualizacion por cambios | Requiere RTX 4090; no evalua reinicio ni navegacion social |
| FocusNav, 2026 [6] | Seis escenarios reales, incluido peaton dinamico | Navegacion local aprendida con atencion espacial | Si, como obstaculo dinamico | No | Requiere metas predefinidas; autores reconocen falta de semantica social |
| ReL-NWM, 2025 [26] | 6/9 tareas image-goal reales | Modelo de mundo latente eficiente y waypoints | No evaluadas | Contexto visual, no memoria semantica persistente | Muestra pequena, meta por imagen y sin razonamiento de habitaciones |
| Action Agent, 2026 [7] | 11/17 tareas reales completadas | Plan agentico por video y control por flujo | No | Memorias para generacion, no mapa persistente | Ejecucion real abierta durante cada movimiento |
| Humanoid-COA, 2025 [8] | G1 y H1-2 en apartamento | Descomposicion de lenguaje en acciones estructuradas | No | Semantica de cuartos implicita en las tareas | Prioriza loco-manipulacion; no mide persistencia ni seguridad social |
| SHIELD, 2025 [9] | G1 real | Filtro probabilistico con funciones de barrera | Evitacion cualitativa de humanos | No | No es sistema semantico ni benchmark social completo |
| SPARK, 2025 [27] | Casos de estudio G1 y benchmark simulado | Kit modular para proteccion y asistencia | Evitacion de una mano humana | No | Evalua seguridad de control, no misiones semanticas |
| CBF-RL, 2025/2026 [28] | G1 evita obstaculos y escaleras | Incorpora restricciones CBF durante entrenamiento RL | No evaluadas | No | No conserva un filtro externo ante fallos del sistema |
| RAPT, 2026 [29] | 24 ensayos OOD reales en G1 | Monitor a 50 Hz y diagnostico de fallos | No evaluadas | No | Detecta riesgo, pero no planifica ni verifica la mision |
| HANDOFF, 2026 [30] | Rollouts reales de tareas en G1 | Interfaz explicita de 10 dimensiones, expertos y recuperacion de caidas | No evaluadas | No | Loco-manipulacion; sin benchmark de navegacion semantica |
| Terrain Consistent RL, 2026 [10] | Mas de 70 m autonomos exteriores | Locomocion perceptiva con interfaz de velocidad y MPC-CBF | No evaluadas | Mapa geometrico preconstruido | Referencia de locomocion, no de tarea semantica |
| SafeGuard ASF, 2026 [11] | Patrulla y percepcion reales | Agente de inspeccion industrial | Detecta intrusos | Sin memoria espacial evaluada | Su escenario objetivo es una fabrica sin personas |
| Triple-Zero Collaborative Navigation, 2026 [12] | G1 y Go2 reales | Cooperacion VLM sin mapa ni entrenamiento | Entornos explicitamente sin humanos | No persistente | Problema multi-robot distinto |
| PhysicalAgent, 2025 [13] | Manipulacion real en G1 | Revision iterativa eleva el exito tras fallos | No | Memoria de ejecucion, no espacial | Aporta recuperacion, pero no navegacion |

## Resultados SOTA que si son informativos

Las cifras siguientes no son intercambiables. Cada una mide una parte distinta
del problema.

### Grounding semantico

FSR-VLN obtiene 92.0% de recuperacion correcta sobre 87 instrucciones reales,
frente a 60.9% de OK-Robot, 51.7% de HOV-SG y 34.5% de MobilityVLA [1]. Su
latencia reportada es 1.5 s cuando basta el camino rapido y 5.5 s cuando activa
razonamiento lento. Esta es una comparacion valida para el **resolver
semantico**, no para mission success, SPL, personas o tolerancia a fallos.

La principal leccion es conservar una jerarquia espacial. Buscar primero una
habitacion, despues una vista y finalmente una entidad reduce candidatos y
latencia. Su principal vacio es que el articulo deja la reconstruccion
adaptativa del grafo para trabajo futuro.

### Navegacion de edificio

SysNav es la referencia arquitectonica mas cercana [2]. En simulacion reporta:

| Benchmark | SR | SPL |
|---|---:|---:|
| HM3D-v1 | 63.7 | 30.5 |
| HM3D-v2 | 80.8 | 37.2 |
| MP3D | 50.7 | 18.1 |
| HM3D-OVON | 54.9 | 26.1 |

En sus pruebas cuantitativas reales sobre la plataforma con ruedas obtiene
entre 97.5% y 100% de SR, segun dificultad. No debe atribuirse ese resultado
al G1: las pruebas G1 publicadas son cualitativas. Para Yanapaq, SysNav sirve
como baseline conceptual y como referencia para representacion
habitacion-vista-objeto, pero una comparacion numerica justa exige ejecutar
ambos sistemas bajo el mismo protocolo y hardware.

### Navegacion hacia una imagen

ReL-NWM [26] obtiene en Gibson 60.0% de SR y 48.5% de SPL, con 1.49 s por
paso. En nueve escenarios reales con G1 completa seis tareas (66.7%) usando
solo la camara RGB del torso y un Jetson Orin de 16 GB; el controlador inferior
se encarga de evitar obstaculos. Es evidencia relevante de navegacion visual
eficiente en el hardware objetivo, pero no admite instrucciones de lenguaje,
clasificacion de habitaciones, memoria persistente ni evaluacion con personas.
Por ello es un comparador secundario, no el baseline principal de Yanapaq.

### Memoria ante cambios

MIF [5] reporta en G1 real 92% de grounding semantico bajo marcha y 94% de
exito al relocalizar objetos despues de actualizar su memoria, frente a 12% de
un grafo estatico. Tambien reporta 98% para eliminaciones y 86% para
incorporaciones tras actualizar el campo. Es el competidor mas fuerte para
entidades movidas o retiradas.

MIF no estudia persistencia operacional: caida y recuperacion del proceso,
versiones de mapa, transacciones, procedencia consultable o entidades
ambiguas. Yanapaq puede diferenciarse si prueba esos casos, en lugar de
limitarse a afirmar que posee "memoria".

DynaMem [33], DovSG [34] y el reciente DREAM [35] confirman que actualizar
objetos movidos, agregados o retirados ya es una linea competitiva. DREAM
supera a DynaMem en tareas largas de manipulacion movil y mantiene memoria
acotada, aunque no usa G1. Por tanto, la novedad defendible no es simplemente
"actualizar el mapa", sino separar y evaluar conjuntamente estructura
persistente, objetos movibles con historial y personas efimeras, con
recuperacion tras reinicio y navegacion verificable en G1.

### Navegacion dinamica

EgoNav [4] recorre 201 m durante 7.5 min en su condicion dinamica, con seis
intervenciones y 96% de autonomia. El sistema tambien atraviesa escenas con
vidrio y puertas. FocusNav [6] alcanza 87.02% de exito en su simulacion
dinamica no estructurada, con frecuencia de colision 4.56, y presenta pruebas
reales con peatones.

Ambos resultados muestran buena navegacion local, pero no prueban cumplimiento
social. Ninguno reporta de forma sistematica distancia interpersonal, tiempo a
colision, invasion de espacio personal, comodidad humana o conducta en puertas
y pasillos compartidos.

### Seguridad

SHIELD [9] combina un modelo probabilistico de dinamica residual con una
funcion de barrera de control y opera a 100 Hz sobre controladores de
locomocion desconocidos. Sus experimentos reales incluyen obstaculos y
evitacion de una persona. Terrain Consistent RL [10] demuestra que una politica
de locomocion puede exponerse mediante una interfaz SE(2) limpia a un
planificador convencional y protegerse con MPC-CBF.

SPARK [27] aporta un toolbox y benchmark modular para seguridad humanoide.
CBF-RL [28] integra las barreras durante el entrenamiento para que el G1 evite
obstaculos y escaleras sin un filtro de seguridad en tiempo de ejecucion. Esa
propiedad mejora el controlador, pero no cubre comandos obsoletos, perdida de
servicios ni errores del planificador.

RAPT [29] cubre precisamente el monitoreo durante despliegue: funciona a 50 Hz
con propiocepcion, mejora 12.5 puntos porcentuales la deteccion sobre el mejor
baseline en robot real, recupera 18 de 24 casos OOD y alcanza 75% de exactitud
top-1 al diagnosticar 16 fallos. HANDOFF [30], depositado en junio de 2026,
refuerza otra decision de arquitectura: una interfaz de tarea explicita entre
el agente y expertos de locomocion, movimiento y recuperacion de caidas. Ni
RAPT ni HANDOFF resuelven memoria semantica, pero ambos son mejores referencias
que afirmar genericamente que el LLM "controla" al robot.

Estos trabajos respaldan la separacion entre planificacion de tarea,
navegacion y autorizacion fisica. No sustituyen el benchmark de fallos de
Yanapaq: comando vencido, perdida de heartbeat, localizacion perdida,
cancelacion, reinicio del orquestador y perdida del modelo cognitivo.

## Habitaciones y analisis semantico

Los sistemas recientes siguen tres estrategias principales:

1. **Jerarquia explicita.** FSR-VLN representa piso, habitacion, vista y objeto
   [1]. La consulta desciende por esa jerarquia.
2. **Grafo room-centric.** SysNav construye nodos de habitacion con categoria,
   vistas, conectividad y contencion de objetos [2]. El VLM decide que
   habitacion explorar y el planificador ejecuta la geometria.
3. **Agrupacion mas captioning.** SGImagineNav agrupa objetos en regiones,
   asigna etiquetas de habitacion mediante un VLM e imagina habitaciones
   probablemente conectadas usando conocimiento comun [14].

Otros sistemas muestran dos mecanismos reproducibles. Cheng et al. [37]
clasifican cada habitacion mediante votacion de un LLM sobre varias
observaciones. SCOUT [38] segmenta habitaciones desde la topologia y las
puertas, asigna objetos a cada region e infiere el tipo de cuarto por su
contenido. La etiqueta debe permitir `UNKNOWN` o `MIXED` y cambiar solo con
evidencia acumulada, no porque se mueva un unico mueble.

MIF mantiene afiliacion de objetos a habitaciones [5], pero la clasificacion
del tipo de cuarto no es su variable principal. En general, los papers
reportan exito de busqueda final y no aislan si el fallo provino de:

- clasificar mal la habitacion;
- confundir dos habitaciones del mismo tipo;
- inferir mal una relacion objeto-habitacion;
- seleccionar una habitacion correcta pero una region incorrecta;
- no reconocer una habitacion mixta o desconocida;
- conservar una etiqueta obsoleta despues de una remodelacion.

El benchmark mas adecuado para cubrir esta brecha es **HieraNav/LangMap**
[15]. Define metas en cuatro niveles: escena, tipo de habitacion, instancia de
region e instancia de objeto. Contiene mas de 18,000 tareas humanamente
verificadas y 414 categorias. Su baseline PlaNaVid obtiene 42.6% de SR
multi-meta y solo 14.3% de `SeqSR@2`, lo que confirma que resolver secuencias
jerarquicas sigue siendo dificil.

**IntentionNav** [24] es una prueba secundaria util si el paper incluye
ordenes indirectas como "necesito calentar esta comida". Separa inferencia de
intencion (`IM`), llegada al vecindario (`OSR`), exito terminal (`SR`) y
grounding visual final (`GSR`). En 2,000 episodios, sus agentes de referencia
promedian 0.249 de SR y solo 0.055 de GSR, mostrando que inferir una meta no
garantiza terminar frente al objeto correcto.

Para Yanapaq se recomienda medir por separado:

- exactitud y macro-F1 del tipo de habitacion;
- `Recall@1` de la instancia de habitacion;
- precision/recall de aristas `CONNECTED_TO` e `IN_ROOM`;
- exactitud de contencion objeto-habitacion;
- SR y SPL estratificados por escena, habitacion, region e instancia;
- tasa de respuesta `UNKNOWN` correcta en cuartos ambiguos o fuera de
  taxonomia;
- calibracion de confianza, no solo etiqueta top-1.

## Que significa realmente "memoria" en estos trabajos

La palabra se usa para mecanismos diferentes:

| Sistema | Memoria real utilizada | Que no demuestra |
|---|---|---|
| EgoNav [4] | Ventana visual de 360 grados y corto horizonte | Persistencia entre tareas o reinicios |
| MerNav [3] | Memoria corta, memoria de tarea y conocimiento comun | Su memoria espacial de tarea se elimina al terminar |
| FSR-VLN [1] | Grafo jerarquico preconstruido | Actualizacion dinamica y procedencia |
| SysNav [2] | Representacion estructurada del edificio | Ciclo de vida transaccional de entidades |
| MIF [5] | Campo espacial topologico con actualizacion local | Recuperacion tras fallo y auditoria historica |
| AstraNav-Memory [17] | Hasta cientos de frames comprimidos en contexto | Fuente de verdad espacial o persistencia operacional |
| SCOPE [16] | Grafo de potencial y snapshots de exploracion | Entidades versionadas entre sesiones |
| GOAT-Bench [18] | Reutilizacion de observaciones en una secuencia de metas | Reinicio real, mapas corregidos o procedencia |

En GOAT-Bench, SCOPE reporta 73.7% SR y 53.5% SPL [16].
AstraNav-Memory reporta 62.7% SR y 56.9% SPL en `Val-Unseen` [17]. No hay un
unico "SOTA" absoluto porque cambian entrenamiento, modelos, subconjuntos y
protocolos; debe citarse la configuracion exacta.

LMEE-Bench [19] agrega preguntas sobre atributos, conteo, ubicacion, relaciones
y estado despues de explorar. Es util para comprobar que la memoria permite
responder sobre el mundo y no solo reencontrar un vector parecido. En su
benchmark propio, MemoryExplorer alcanza 23.53% SR y 14.99% SPL; en un
subconjunto de GOAT `Val-Unseen`, 46.40% y 28.03%.

La contribucion diferenciadora de Yanapaq debe formularse como **memoria
espacial operacional**: entidades y relaciones persistentes, ligadas a una
version de mapa, con incertidumbre, vigencia, evidencia y recuperacion despues
de reiniciar. Esa definicion es mas fuerte y verificable que "historial visual
largo".

## Estan listos los SOTA para entornos con personas

La respuesta corta es **no, no como sistemas autonomos no supervisados**.

| Trabajo | Evidencia con personas | Evaluacion social cuantitativa | Veredicto |
|---|---|---|---|
| EgoNav [4] | Recorridos reales entre personas | Intervenciones y autonomia, no proxemica | Prometedor para evitacion local |
| FocusNav [6] | Escenario real con peatones | Exito/colision global; sin confort | Parcial |
| SHIELD [9] | Evitacion real cualitativa | Sin protocolo social amplio | Capa de seguridad, no solucion social |
| MIF [5] | Demostracion de detenerse a 1.5 m y saludar | No | Demostracion, no evidencia suficiente |
| SysNav [2] | Personas como referencia semantica en instrucciones | No | La persona es una meta/objeto semantico |
| SafeGuard ASF [11] | Deteccion de intrusos | No | Seguridad industrial, no convivencia |
| Hi-Dyna Graph [36] | Subgrafos dinamicos de humanos y objetos | No | Representacion prometedora, no benchmark social G1 |

LookOut [31] y EgoCogNav [32] recolectan trayectorias egocentricas humanas para
aprender movimiento, prediccion de cabeza y decisiones cognitivas. Son fuentes
utiles para preentrenar o analizar comportamiento humano, pero no despliegan
su politica de navegacion en un G1 ni constituyen evidencia de seguridad social
del robot.

Las demostraciones con una persona no equivalen a navegacion social. Un sistema
listo para espacios compartidos debe medir seguridad, eficiencia, legibilidad
y comodidad en cruces, adelantamientos, puertas, grupos, oclusiones y cambios
repentinos de direccion, siguiendo una bateria de escenarios y no una sola
demostracion. Las recomendaciones de Francis et al. [25] son una base adecuada
para separar eficiencia, seguridad, naturalidad y experiencia humana.

Los benchmarks mas utiles son:

- **HA-VLN 2.0** [20]: 16,844 instrucciones, interacciones con multiples
  humanos y metricas `TCR`, `CR`, error de navegacion y SR sin colision.
- **HuNavSim 2.0** [21]: generacion reproducible de peatones mediante behavior
  trees, integracion con Isaac Sim/Gazebo/Webots y 32 metricas sociales.
- **SocialNav-SUB** [22]: 4,968 preguntas en 60 escenas reales de SCAND para
  razonamiento espacial, espaciotemporal y social; usa `PA` y `CWPA`.
- **Follow-Bench** [23]: multitudes, puertas, intersecciones y distancias
  proxemicas; solo es pertinente si se incorpora seguimiento de personas, que
  no forma parte del MVP.

SocialNav-SUB encuentra que incluso los VLM recientes quedan por debajo de
baselines humanos y, en varios casos, de reglas geometricas simples. Por ello,
el VLM no deberia autorizar frenado ni decidir por si solo si una trayectoria
es fisicamente segura.

## Benchmark recomendado para Yanapaq

### 1. Afirmacion principal

> Yanapaq mejora la ejecucion verificable de misiones semanticas persistentes
> en un G1 real, manteniendo comportamiento conservador ante personas y fallos.

Esta afirmacion requiere un benchmark integrado propio porque ningun benchmark
existente combina todas sus variables. Debe complementarse con benchmarks
estandar por componente.

### 2. Comparadores correctos

| Afirmacion | Comparador principal | Metrica compartida |
|---|---|---|
| Grounding jerarquico | FSR-VLN/HMSG [1] | RSR o exactitud del objetivo, latencia |
| Navegacion semantica | SysNav [2] y pipeline sin memoria | SR, SPL, SCT, error terminal |
| Navegacion image-goal en G1 | ReL-NWM [26] | SR, SPL, latencia, bajo protocolo adaptado |
| Memoria ante cambios | MIF [5] y memoria visual/vectorial | Recall@1, stale rate, relocation/removal/addition |
| Memoria de largo horizonte | SCOPE/AstraNav sobre GOAT [16,17] | SR y SPL bajo protocolo identico |
| Habitaciones | HieraNav/LangMap [15] | SR/SPL por nivel y `SeqSR@k` |
| Personas | HA-VLN 2.0/HuNavSim 2.0 [20,21] | CR, distancia, TTC, intrusion social |
| Control local dinamico | FocusNav/EgoNav [4,6] | exito, colisiones, intervenciones |
| Fallos y autoridad | RAPT, SPARK, SHIELD y ablaciones propias [9,27,29] | deteccion, diagnostico, parada, movimiento no autorizado |

No se recomienda presentar una sola tabla mezclando 92% de retrieval de
FSR-VLN, 94% de relocalizacion de MIF, 87% de FocusNav y el SR extremo a
extremo de Yanapaq. Son denominadores y tareas diferentes.

### 3. Protocolo real principal: Persistent Semantic PlaceNav-G1

Entornos:

- dos edificios o dos plantas con al menos seis tipos de habitacion;
- habitaciones repetidas del mismo tipo;
- objetos grandes fijos, objetos movibles y lugares nombrados;
- rutas cortas, medias y largas;
- zonas con puertas, intersecciones y pasillos estrechos.

Condiciones minimas:

1. meta geometrica conocida;
2. lugar nombrado en la misma sesion;
3. lugar nombrado despues de reiniciar el orquestador;
4. lugar recordado despues de correccion global del mapa;
5. dos lugares u objetos con el mismo nombre;
6. objeto movido, retirado y agregado;
7. consulta por tipo de habitacion;
8. consulta por instancia de habitacion o relacion espacial;
9. persona cruzando perpendicular y diagonalmente;
10. persona bloqueando temporalmente una puerta;
11. perdida del servicio cognitivo durante movimiento;
12. localizacion degradada, comando vencido y cancelacion.

Cada condicion automatizable deberia tener al menos 20 repeticiones. Las
pruebas con personas deben empezar con obstaculo blando o maniqui, continuar
con un actor informado y usar operador de seguridad.

### 4. Metricas obligatorias

Tarea y semantica:

- mission success rate;
- grounded success: region correcta y meta visible/verificada;
- exactitud del objetivo semantico;
- aclaraciones y seleccion incorrecta entre homonimos;
- SR y SPL por nivel escena/habitacion/region/instancia.

Memoria:

- `Recall@1`, MRR y nDCG;
- recuperacion despues de reinicio;
- tasa de entidad obsoleta devuelta;
- precision/recall de fusion y division;
- exactitud de `IN_ROOM` y `CONNECTED_TO`;
- porcentaje de respuestas con evidencia y version de mapa validas;
- latencia p50/p95 y memoria utilizada.

Navegacion:

- SR, SPL y SCT;
- error terminal, longitud y tiempo;
- oscilaciones, replans e intervenciones;
- exito de recuperacion por causa.

Personas:

- colisiones y near misses;
- distancia minima y percentil 5;
- TTC minimo;
- tiempo y numero de invasiones de zona personal;
- exito en cruce, espera y puerta;
- velocidad al aproximarse;
- valoracion de comodidad solo despues de cumplir limites objetivos.

Seguridad y fallos:

- tasa de fallos que terminan en estado seguro;
- movimiento no autorizado;
- latencia hasta comando cero y hasta reposo;
- distancia de parada;
- comandos vencidos aceptados/rechazados;
- continuidad segura tras perdida del LLM;
- causa terminal correctamente registrada.

### 5. Ablaciones de mayor valor cientifico

Para un short paper no conviene dispersarse. Las cuatro ablaciones mas
informativas son:

1. memoria completa frente a recuperacion visual/vectorial;
2. memoria completa sin vigencia ni procedencia;
3. capa de personas y supervisor frente a evitacion geometrica basica;
4. orquestador determinista frente a llamadas agenticas directas.

Una quinta ablacion, si hay espacio, es detector abierto continuo frente a
detector abierto bajo demanda. No se debe desactivar el supervisor en pruebas
libres con el robot real.

## Propuesta de comparacion para el short paper

La tabla principal del paper deberia contener solo tres sistemas ejecutados
bajo el mismo protocolo:

1. baseline geometrico/reactivo con memoria visual simple;
2. Yanapaq sin memoria persistente;
3. Yanapaq completo.

Columnas recomendadas:

`SR`, `Grounded-SR`, `SPL`, `Recall@1-post-restart`, `Room-F1`,
`min-distance`, `unsafe-motion` y `stop-latency-p95`.

Una segunda tabla corta debe ubicar el resultado respecto a literatura, sin
afirmar comparabilidad directa:

- FSR-VLN: 92% de retrieval en 87 instrucciones reales;
- SysNav: resultados de simulacion y pruebas G1 cualitativas;
- MIF: 94% de relocalizacion post-actualizacion;
- FocusNav: 87.02% en simulacion dinamica;
- EgoNav: 96% de autonomia en su tramo dinamico;
- ReL-NWM: 6/9 tareas image-goal reales en G1;
- SCOPE: 73.7% SR y 53.5% SPL en GOAT-Bench;
- PlaNaVid: 42.6% SR multi-meta en LangMap.

La discusion debe explicar por que el aporte no es un nuevo controlador
neuronal: la contribucion esta en integrar lenguaje, memoria persistente,
semantica jerarquica, navegacion y autoridad segura en una mision reproducible.

## Informacion que suele olvidarse

- **Postcondicion:** llegar a una coordenada no prueba que se alcanzo la
  entidad correcta. Debe verificarse region, distancia y evidencia visual.
- **Open set:** una habitacion puede ser mixta o desconocida. Forzar siempre
  una etiqueta conocida infla resultados y crea errores peligrosos.
- **Vigencia:** los objetos movibles necesitan historial y cierre de la pose
  anterior, no sobrescritura.
- **Evidencia negativa:** no observar un objeto donde deberia estar ayuda a
  decidir que una memoria esta obsoleta.
- **Topologia:** clasificar habitaciones sin medir sus conexiones no basta
  para navegacion de edificio.
- **Campo visual:** una camara frontal deja zonas ciegas durante giros; el
  LiDAR y la capa geometrica deben mantener la seguridad.
- **Desfase temporal:** deteccion correcta pero tardia puede ser mas peligrosa
  que una clasificacion imperfecta.
- **Fuera de distribucion:** una politica puede seguir produciendo comandos
  plausibles mientras falla. Se debe medir deteccion y diagnostico en linea,
  como propone RAPT, ademas del resultado terminal.
- **Privacidad:** los tracks de personas deben ser efimeros; no se necesita
  reconocimiento facial para navegacion social.
- **Carga computacional:** deben reportarse GPU, VRAM, potencia, frecuencia y
  p95. MIF, por ejemplo, usa una RTX 4090.
- **Denominador real:** siempre separar resultados en simulacion, robot con
  ruedas, cuadrupedo y G1; tambien separar demostraciones cualitativas de
  ensayos cuantitativos.
- **Cambios de modelo remoto:** version, fecha y prompts deben congelarse o
  registrarse.
- **Resultados fallidos:** deben conservarse; eliminarlos impide evaluar
  seguridad y recuperacion.

## Veredicto para Yanapaq

La literatura reciente confirma que la arquitectura propuesta no esta
obsoleta por usar componentes deterministas. Los sistemas mas robustos
desacoplan razonamiento, representacion espacial, planificacion, control y
seguridad; HANDOFF muestra que incluso los enfoques agenticos mas recientes
usan una interfaz compacta y explicita hacia el control [30]. Las redes
neuronales son valiosas para percepcion, lenguaje y
locomocion, pero no reemplazan contratos, estados de mision, vigencia,
cancelacion ni autoridad fisica.

El nicho con mayor probabilidad de producir una contribucion clara es:

> **Navegacion semantica persistente y verificable en un G1 real, con
> comprension jerarquica de lugares, respuesta conservadora a personas y
> recuperacion segura ante fallos.**

La afirmacion de "mejor que SOTA" solo seria defendible en el benchmark propio
y bajo el mismo protocolo. En benchmarks externos debe hablarse de
competitividad por componente y reconocer diferencias de plataforma, tarea y
metrica.

## Referencias

[1] X. Zhou et al., ["FSR-VLN: Fast and Slow Reasoning for Vision-Language
Navigation with Hierarchical Multi-modal Scene
Graph"](https://arxiv.org/abs/2509.13733), 2025.

[2] H. Zhu et al., ["SysNav: Multi-Level Systematic Cooperation Enables
Real-World, Cross-Embodiment Object
Navigation"](https://arxiv.org/abs/2603.06914), 2026.

[3] D. Qi et al., ["MerNav: A Highly Generalizable Memory-Execute-Review
Framework for Zero-Shot Object Goal
Navigation"](https://arxiv.org/abs/2602.05467), 2026.

[4] W. Wang et al., ["Learning Humanoid Navigation from Human
Data"](https://arxiv.org/abs/2604.00416), 2026.

[5] P. Jiang et al., ["Learning to Evolve: Multi-modal Interactive Fields for
Robust Humanoid Navigation in Dynamic
Environments"](https://arxiv.org/abs/2605.21935), RSS 2026.

[6] Y. Zhang et al., ["FocusNav: Spatial Selective Attention with Waypoint
Guidance for Humanoid Local Navigation"](https://arxiv.org/abs/2601.12790),
2026.

[7] J. Sam et al., ["Action Agent: Agentic Video Generation Meets
Flow-Constrained Diffusion"](https://arxiv.org/abs/2605.01477), 2026.

[8] C. Wen et al., ["Humanoid Agent via Embodied Chain-of-Action Reasoning
with Multimodal Foundation Models for Zero-Shot
Loco-Manipulation"](https://arxiv.org/abs/2504.09532), 2025.

[9] L. Yang et al., ["SHIELD: Safety on Humanoids via CBFs In Expectation
on Learned Dynamics"](https://arxiv.org/abs/2505.11494), 2025.

[10] W. D. Compton et al., ["Terrain Consistent Reference-Guided RL for
Humanoid Navigation Autonomy"](https://arxiv.org/abs/2605.15517), 2026.

[11] T. N. Canh et al., ["SafeGuard ASF: SR Agentic Humanoid Robot System for
Autonomous Industrial Safety"](https://arxiv.org/abs/2603.25353), 2026.

[12] Y. Wang et al., ["Can a Robot Walk the Robotic Dog: Triple-Zero
Collaborative Navigation for Heterogeneous Multi-Agent
Systems"](https://arxiv.org/abs/2603.21723), 2026.

[13] A. Lykov et al., ["PhysicalAgent: Towards General Cognitive Robotics with
Foundation World Models"](https://arxiv.org/abs/2509.13903), 2025.

[14] Y. Hu et al., ["Imaginative World Modeling with Scene Graphs for
Embodied Agent Navigation"](https://arxiv.org/abs/2508.06990), 2025.

[15] B. Miao et al., ["LangMap: A Human-Verified Benchmark for Hierarchical
Open-Vocabulary Goal Navigation"](https://arxiv.org/abs/2602.02220), 2026.

[16] N. Wang et al., ["Expand Your SCOPE: Semantic Cognition over
Potential-Based Exploration for Embodied Visual
Navigation"](https://arxiv.org/abs/2511.08935), AAAI 2026.

[17] B. Ren et al., ["AstraNav-Memory: Contexts Compression for Long
Memory"](https://arxiv.org/abs/2512.21627), CVPR 2026.

[18] M. Khanna et al., ["GOAT-Bench: A Benchmark for Multi-Modal Lifelong
Navigation"](https://openaccess.thecvf.com/content/CVPR2024/html/Khanna_GOAT-Bench_A_Benchmark_for_Multi-Modal_Lifelong_Navigation_CVPR_2024_paper.html),
CVPR 2024.

[19] S. Wang et al., ["Explore with Long-term Memory: A Benchmark and
Multimodal LLM-based Reinforcement Learning Framework for Embodied
Exploration"](https://arxiv.org/abs/2601.10744), 2026.

[20] Y. Dong et al., ["HA-VLN 2.0: An Open Benchmark and Leaderboard for
Human-Aware Navigation in Discrete and Continuous Environments with Dynamic
Multi-Human Interactions"](https://arxiv.org/abs/2503.14229), 2025/2026.

[21] M. Escudero-Jimenez et al., ["HuNavSim 2.0: An Enhanced Human
Navigation Simulator for Human-Aware Robot
Navigation"](https://arxiv.org/abs/2507.17317), 2025.

[22] M. J. Munje et al., ["SocialNav-SUB: Benchmarking VLMs for Scene
Understanding in Social Robot
Navigation"](https://arxiv.org/abs/2509.08757), 2025.

[23] H. Ye et al., ["Follow-Bench: A Unified Motion Planning Benchmark for
Socially-Aware Robot Person Following"](https://arxiv.org/abs/2509.10796),
2025/2026.

[24] L. Qian et al., ["IntentionNav: A Benchmark for Intent-Driven Object
Navigation from Implicit Human Instruction"](https://arxiv.org/abs/2605.23187),
2026.

[25] A. Francis et al., ["Principles and Guidelines for Evaluating Social
Robot Navigation Algorithms"](https://arxiv.org/abs/2306.16740), ACM THRI,
2025.

[26] Z. Zhang et al., ["Efficient Image-Goal Navigation with Representative
Latent World Model"](https://arxiv.org/abs/2511.11011), 2025.

[27] Y. Sun et al., ["SPARK: Safe Protective and Assistive Robot
Kit"](https://arxiv.org/abs/2502.03132), 2025.

[28] L. Yang et al., ["CBF-RL: Safety Filtering Reinforcement Learning in
Training with Control Barrier Functions"](https://arxiv.org/abs/2510.14959),
ICRA 2026.

[29] H. Munn et al., ["RAPT: Model-Predictive Out-of-Distribution Detection
and Failure Diagnosis for Sim-to-Real Humanoid
Robots"](https://arxiv.org/abs/2602.01515), 2026.

[30] L. Yang et al., ["HANDOFF: Humanoid Agentic Task-Space Whole-Body Control
via Distilled Complementary Teachers"](https://arxiv.org/abs/2606.06493),
2026.

[31] B. Pan et al., ["LookOut: Real-World Humanoid Egocentric
Navigation"](https://arxiv.org/abs/2508.14466), 2025.

[32] Z. Qiu et al., ["EgoCogNav: Cognition-aware Human Egocentric
Navigation"](https://arxiv.org/abs/2511.17581), 2025.

[33] P. Liu et al., ["DynaMem: Online Dynamic Spatio-Semantic Memory for Open
World Mobile Manipulation"](https://arxiv.org/abs/2411.04999), ICRA 2025.

[34] Z. Yan et al., ["Dynamic Open-Vocabulary 3D Scene Graphs for Long-Term
Language-Guided Mobile Manipulation"](https://arxiv.org/abs/2410.11989), IEEE
RA-L 2025.

[35] Z. Yan et al., ["DREAM: Dynamic Resilient Spatio-Semantic Memory with
Hybrid Localization for Mobile Manipulation"](https://arxiv.org/abs/2606.00576),
2026.

[36] J. Hou et al., ["Hi-Dyna Graph: Hierarchical Dynamic Scene Graph for
Robotic Autonomy in Human-Centric
Environments"](https://arxiv.org/abs/2506.00083), 2025.

[37] Y. Cheng et al., ["Intelligent Spatial Perception by Building
Hierarchical 3D Scene Graphs for Indoor Scenarios with the Help of
LLMs"](https://arxiv.org/abs/2503.15091), 2025.

[38] I. Mahdi et al., ["Relational Semantic Reasoning on 3D Scene Graphs for
Open World Interactive Object Search"](https://arxiv.org/abs/2603.05642),
2026.
