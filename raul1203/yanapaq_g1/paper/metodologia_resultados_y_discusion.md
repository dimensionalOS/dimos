# Yanapaq: metodologia, resultados y discusion

Ultima modificacion: 2026-06-12 16:23:07 -05 -0500

> **Nota de integridad cientifica.** Las cifras de la seccion de resultados son
> datos sinteticos plausibles creados para maquetar y revisar el paper. No
> corresponden a ensayos ejecutados y deben reemplazarse por mediciones reales
> antes de someter el manuscrito.

## Metodologia

Yanapaq se concibio como una arquitectura agentica para que un humanoide
Unitree G1 convierta instrucciones en lenguaje natural en misiones fisicas
verificables. El sistema separa deliberadamente cuatro niveles de autoridad:
interaccion, deliberacion, autonomia y seguridad. El modelo de lenguaje
interpreta la intencion y propone una mision estructurada, pero no genera
velocidades ni comandos articulares. Una maquina de estados determinista
valida permisos, precondiciones, plazos y criterios de finalizacion; los
subsistemas de autonomia calculan el movimiento; y un supervisor independiente
es la unica puerta hacia el controlador del robot.

El G1 emplea un LiDAR 3D con IMU y una camara RGB-D rigidamente calibrada. La
odometria LiDAR-inercial estima una pose local continua, mientras una capa de
optimizacion global mantiene la consistencia del mapa entre recorridos. Se
conservan tres representaciones separadas: un mapa geometrico persistente, un
mapa local de obstaculos con caducidad y una capa dinamica para personas. Esta
separacion evita que una persona o un objeto movil se conviertan en una pared
permanente y permite que el control local siga siendo continuo aun cuando se
actualiza el mapa global.

La percepcion visual sigue una estrategia de dos niveles. Un detector ligero
opera continuamente sobre clases criticas, principalmente personas y
obstaculos frecuentes. Las consultas de vocabulario abierto se ejecutan solo
cuando la instruccion nombra una entidad desconocida o existe ambiguedad. Las
detecciones se asocian temporalmente, se proyectan a 3D mediante profundidad y
transformaciones calibradas, y se publican como observaciones con covarianza.
Una observacion individual nunca se considera automaticamente un hecho
persistente.

La memoria espacial fusiona observaciones en entidades versionadas. Cada
entidad conserva etiqueta, alias, geometria, pose e incertidumbre, vigencia,
movilidad y referencias a la evidencia que la origino. Los lugares se modelan
como regiones navegables con puntos de aproximacion, no como coordenadas
aisladas. Las consultas combinan restricciones geometricas, temporales y
similitud semantica, lo que permite resolver expresiones como "la mesa del
laboratorio" y mantener la respuesta despues de reiniciar el sistema.

La entrada de voz se procesa mediante deteccion de actividad, reconocimiento
de habla y estimacion de confianza. El resultado se transforma en una
`MissionSpec` tipada que contiene objetivo, restricciones, plazo y
postcondicion. Si hay varios destinos plausibles o baja confianza, el robot
solicita aclaracion. Una vez autorizada la mision, la navegacion resuelve la
entidad a una region alcanzable, calcula una ruta global y actualiza el
movimiento local sin volver a consultar al modelo de lenguaje. El exito se
declara solo cuando se verifica la postcondicion espacial.

El supervisor se ejecuta con mayor frecuencia que el plano cognitivo. Arbitra
entre autonomia, teleoperacion y paro; limita la velocidad segun localizacion,
obstaculos, personas y salud computacional; y rechaza comandos vencidos. La
perdida del modelo de lenguaje no afecta el frenado. La perdida de
localizacion, mapa local o telemetria conduce a velocidad cero o a un estado
degradado. Cada mision registra sensores, decisiones, planes, comandos
solicitados y autorizados, eventos de seguridad y causa terminal para permitir
reproduccion y analisis causal.

### Protocolo experimental

El protocolo propuesto utiliza dos entornos interiores controlados, un almacen
y un laboratorio conectado por pasillos. Se evaluan ocho escenarios: destino
geometrico, lugar recordado tras reinicio, nombres ambiguos, objeto de
vocabulario abierto, cruce de una persona, bloqueo temporal, perdida del
servicio cognitivo y cancelacion durante movimiento. Cada escenario se repite
20 veces por condicion, con orden aleatorio, velocidad maxima de 0.35 m/s y la
misma configuracion de sensores y control.

La linea base reactiva emplea la misma localizacion, planificacion y
supervisor, pero carece de memoria persistente de entidades, grounding
espacio-temporal y orquestacion durable. Las metricas principales son tasa de
mision completada (SR), SPL, exito ponderado por tiempo (SCT), exactitud del
objetivo semantico, `Recall@1` tras reinicio, separacion minima respecto a
personas, intervenciones, latencia y movimiento no autorizado. SPL y SCT siguen
las definiciones estandar de navegacion [1], [2]; la evaluacion junto a personas
sigue las recomendaciones de seguridad, confort y legibilidad de [7]. Los
intervalos de confianza se calculan al 95 % y todos los fallos se incluyen en
el analisis.

## Resultados y discusion

**Resultados sinteticos para maquetacion.** En 160 misiones por condicion,
Yanapaq completo 141 misiones (88.1 %, IC95 %: 82.2-92.3), frente a 107
(66.9 %, IC95 %: 59.3-73.7) de la linea base. La diferencia de 21.2 puntos
porcentuales fue consistente con una mayor precision al resolver objetivos y
con menos recuperaciones agotadas.

| Metrica | Linea base reactiva | Yanapaq |
|---|---:|---:|
| Mision completada, SR | 66.9 % | **88.1 %** |
| SPL | 0.51 | **0.69** |
| SCT | 0.44 | **0.63** |
| Objetivo semantico correcto | 76.3 % | **93.1 %** |
| `Recall@1` de lugares tras reinicio | 67.5 % | **92.5 %** |
| Cruces con persona completados | 70.0 % | **90.0 %** |
| Paradas protectoras por 100 misiones | 11.9 | **4.4** |

La mayor ganancia aparecio en objetivos nombrados despues de reiniciar el
sistema. Al retirar la memoria persistente, `Recall@1` disminuyo de 92.5 % a
67.5 %, confirmando que los embeddings por si solos no sustituyen la
geometria, vigencia y procedencia. En escenas con personas, la separacion
minima observada fue 0.96 m y el percentil 5 fue 1.04 m. No se registraron
colisiones; los fallos se manifestaron como espera conservadora o cancelacion,
no como aproximaciones agresivas.

Se inyectaron 60 fallos distribuidos entre perdida del servicio cognitivo,
localizacion perdida, mapa local obsoleto y comandos vencidos. En los 60 casos
el sistema alcanzo reposo sin aceptar movimiento no autorizado. El tiempo p95
desde la deteccion del fallo hasta el comando cero fue 0.19 s, y hasta reposo
fisico fue 0.63 s a la velocidad maxima ensayada. Esto sugiere que la principal
contribucion no es solo mejorar el razonamiento, sino impedir que un fallo
cognitivo se propague al actuador.

La comparacion externa requiere cautela. FSR-VLN reporta 92 % de exito al
recuperar el objetivo sobre 87 instrucciones recogidas con un G1 real [3]. El
93.1 % de grounding sintetico de Yanapaq seria competitivo con esa referencia,
pero el 88.1 % extremo a extremo incluye planificacion, movimiento, personas y
fallos, por lo que ambas cifras no son directamente equivalentes. En GOAT-Bench, un
benchmark simulado de navegacion multimodal y memoria persistente [4], SCOPE
reporta 73.7 % de SR y 53.5 % de SPL [5]. Aunque los valores de Yanapaq son
numericamente mayores, el entorno conocido, el menor numero de categorias y la
plataforma fisica impiden afirmar superioridad sobre esa referencia SOTA.

Finalmente, los resultados son comparables en magnitud con el 86.6 % reportado
para navegacion secuencial por un agente humanoide con razonamiento
estructurado [6], pero Yanapaq aborda un alcance distinto: prioriza
persistencia, navegacion social, recuperacion de fallos y autoridad segura, no
manipulacion de cuerpo completo. La evidencia esperada respaldaria que separar
lenguaje probabilistico, ejecucion determinista y supervision fisica mejora la
fiabilidad del sistema. No demostraria seguridad certificada ni generalizacion
a edificios, velocidades o poblaciones no evaluadas.

## Referencias

[1] P. Anderson et al., ["On Evaluation of Embodied Navigation
Agents"](https://arxiv.org/abs/1807.06757), 2018.

[2] N. Yokoyama et al., ["Success Weighted by Completion Time: A Dynamics-Aware
Evaluation Criteria for Embodied Navigation"](https://arxiv.org/abs/2103.08022),
2021.

[3] X. Zhou et al., ["FSR-VLN: Fast and Slow Reasoning for Vision-Language
Navigation with Hierarchical Multi-modal Scene
Graph"](https://arxiv.org/abs/2509.13733), 2025.

[4] M. Khanna et al., ["GOAT-Bench: A Benchmark for Multi-Modal Lifelong
Navigation"](https://openaccess.thecvf.com/content/CVPR2024/html/Khanna_GOAT-Bench_A_Benchmark_for_Multi-Modal_Lifelong_Navigation_CVPR_2024_paper.html),
CVPR 2024.

[5] N. Wang et al., ["Expand Your SCOPE: Semantic Cognition over
Potential-Based Exploration for Embodied Visual
Navigation"](https://arxiv.org/abs/2511.08935), 2026.

[6] Y. Hao et al., ["Embodied Chain of Action Reasoning with Multi-Modal
Foundation Model for Humanoid
Loco-Manipulation"](https://arxiv.org/abs/2504.09532), 2025.

[7] A. Francis et al., ["Principles and Guidelines for Evaluating Social Robot
Navigation Algorithms"](https://arxiv.org/abs/2306.16740), ACM THRI, 2025.
