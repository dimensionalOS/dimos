# Evaluacion integral y estructura de paper

Ultima modificacion: 2026-06-11 12:05:34 -05 -0500

## Pregunta de investigacion

¿Puede una arquitectura que separa lenguaje, orquestacion, autonomia y
seguridad mejorar la ejecucion verificable de misiones semanticas en un
humanoide G1, manteniendo memoria espacial persistente y comportamiento
conservador ante personas y fallos?

## Hipotesis

H1. Un orquestador determinista reduce acciones invalidas y mejora recuperacion
frente a un agente que invoca movimiento directo.

H2. La memoria que combina geometria, vigencia, embeddings y procedencia
mejora la resolucion de lugares respecto a recuperacion visual sola.

H3. La separacion entre mapa estatico y tracks dinamicos reduce bloqueos
persistentes y mantiene distancia a personas.

H4. El supervisor independiente evita movimiento no autorizado durante fallos
del agente, red o planificador.

H5. Un detector abierto bajo demanda aumenta recall semantico con menor coste
que ejecutarlo continuamente.

## Variables

| Tipo | Variables |
|---|---|
| Independientes | Arquitectura de agente, memoria, detector, plan local, presencia de supervisor |
| Dependientes | Exito, SPL/SCT, latencia, error espacial, distancia, intervenciones, consumo |
| Control | Mapa, ordenes, velocidad maxima, pesos, hardware, iluminacion, semillas |
| Confusoras | Aprendizaje del operador, carga de bateria, temperatura, trafico humano |

## Baselines

1. DimOS G1 agentico actual, en las capacidades que puedan ejecutarse con
   seguridad y sin modificarlo.
2. Pipeline propuesto con agente, pero sin memoria persistente.
3. Pipeline propuesto completo.
4. Navegacion DimOS nativa frente al candidato Nav2 MPPI, solo si ambos
   alcanzan integracion estable.
5. Detector cerrado continuo frente a cerrado + abierto bajo demanda.

La brecha actual de navegacion se reporta como resultado de diagnostico; no se
fuerza un baseline imposible.

## Escenarios

| ID | Escenario | Dificultad |
|---|---|---|
| S1 | Ir a una pose conocida | Geometrica basica |
| S2 | Ir a un lugar nombrado tras reinicio | Memoria |
| S3 | Resolver dos objetos con mismo nombre | Ambiguedad |
| S4 | Encontrar un objeto fuera del vocabulario cerrado | Percepcion abierta |
| S5 | Persona cruza la ruta | Dinamica |
| S6 | Paso bloqueado temporalmente | Recuperacion |
| S7 | Perdida del LLM durante navegacion | Fallo cognitivo |
| S8 | Localizacion degradada/perdida | Fallo de autonomia |
| S9 | Cancelacion multimodal | Seguridad |
| S10 | Repetir mision en mapa actualizado | Persistencia |
| S11 | Consulta de objeto ausente/falso positivo | Percepcion |
| S12 | Mismo escenario en simulacion, replay y real | Sim-real |

## Diseño experimental

- Al menos 20 repeticiones por condicion automatizable como objetivo.
- Orden aleatorio de condiciones.
- Mismos limites de velocidad.
- Calibracion y bateria registradas.
- Operador ciego a la condicion cuando sea posible.
- Fallos inyectados en tiempo predefinido.
- Exitos y fallos conservados.
- Intervalos de confianza y tamaño de efecto.
- Prueba estadistica elegida despues de inspeccionar distribucion, no por
  costumbre.

Los ensayos con personas requieren protocolo, consentimiento, area controlada
y un operador de seguridad.

## Metricas integrales

### Tarea

- tasa de mision completada;
- tiempo a completar;
- postcondicion verificada;
- aclaraciones;
- reintentos;
- error de destino.

### Navegacion

- SPL;
- SCT;
- longitud y suavidad;
- oscilaciones;
- intervenciones;
- colisiones y near misses.

### Percepcion y memoria

- precision, recall y mAP;
- HOTA/IDF1;
- error 3D;
- Recall@k/MRR;
- fusiones y divisiones incorrectas;
- persistencia tras reinicio.

### Agente

- intencion;
- seleccion de tool;
- argumentos validos;
- acciones rechazadas;
- unsafe action rate;
- latencia y coste.

### Seguridad y sistema

- distancia/tiempo de parada;
- separacion minima;
- comandos vencidos/rechazados;
- p95 de pipelines;
- CPU, GPU, RAM, VRAM y potencia;
- disponibilidad y recuperacion.

La latencia extremo a extremo se mide desde fin de instruccion hasta inicio
seguro, y desde aparicion de obstaculo hasta comando limitado. Se reportan
subtramos para no ocultar el cuello de botella.

## Comparacion sim-real

La simulacion se usa para cobertura y fallos, no para declarar rendimiento
real. Protocolo:

1. definir el mismo escenario, geometria y `MissionSpec`;
2. ejecutar simulacion con latencia/ruido nominales;
3. reproducir un episodio real sin salida de control;
4. ejecutar G1 real a velocidad calificada;
5. comparar exito, trayectoria, eventos y distribuciones de latencia;
6. cuantificar la brecha por subsistema;
7. ajustar solo despues de congelar el resultado inicial.

Metricas de brecha:

- diferencia de success rate;
- error de trayectoria y tiempo;
- diferencia de distancia minima;
- frecuencia de recuperaciones;
- error de deteccion/profundidad;
- latencia y recursos;
- fallos presentes solo en real.

Las colisiones simuladas sirven para explorar. Los limites del G1 real se
derivan de pruebas fisicas controladas.

## Criterios de exito del prototipo

| Area | Puerta |
|---|---|
| Seguridad | Cero movimiento no autorizado en matriz de fallos |
| Navegacion estatica | >= 90 % en escenario definido |
| Personas | Cero colisiones/near misses y distancia minima cumplida |
| Memoria | >= 90 % Recall@1 en lugares controlados tras reinicio |
| Agente | Cero acciones inseguras aceptadas en suite adversarial |
| Observabilidad | 100 % de ensayos con manifiesto y causa terminal |
| Tiempo real | Sin crecimiento sostenido de backlog |

Son criterios internos iniciales, no resultados obtenidos ni garantias.

## Ablaciones

| Ablacion | Pregunta |
|---|---|
| Sin orquestador | ¿Cuantos errores evita la maquina de estados? |
| Sin procedencia | ¿Afecta la correccion de conflictos de memoria? |
| Sin embedding | ¿Cuanto aporta lo semantico frente a espacio/clase? |
| Sin capa social | ¿Cambia distancia y comodidad? |
| Detector abierto continuo | ¿Vale su coste frente a bajo demanda? |
| Sin PGO | ¿Cuanto afecta deriva a memoria/navegacion? |
| Sin supervisor, solo simulacion | ¿Que fallos llegarian al actuador? |

La ablacion del supervisor no se ejecuta con movimiento libre en G1 real.

## Amenazas a validez

- Un solo edificio y un solo robot.
- Sensores y calibracion particulares.
- Participantes familiarizados con robotica.
- Escenarios preparados.
- Modelo remoto que cambia sin version fija.
- Ground truth limitado.
- Baja cantidad de eventos peligrosos.
- Diferencias de integracion entre candidatos.

Se mitigan publicando configuraciones, repeticiones, fallos y limites de
generalizacion.

## Trazabilidad de resultados

```mermaid
flowchart LR
    H[Hipotesis] --> SC[Escenario]
    SC --> CFG[Configuracion]
    CFG --> EP[Episodio]
    EP --> MET[Metricas]
    MET --> STAT[Analisis]
    STAT --> CLAIM[Afirmacion]
    CLAIM --> LIMIT[Limitaciones]
```

Ninguna afirmacion del paper debe carecer de episodios y configuracion
reproducibles.

## Estructura sugerida del paper

1. **Introduccion**
   Problema de convertir instrucciones abiertas en autonomia fisica
   verificable.
2. **Trabajo relacionado**
   Agentes roboticos, memoria espacial, navegacion social y LLM tool use.
3. **Diagnostico de plataforma**
   Capacidades y brechas observadas en DimOS/G1.
4. **Arquitectura**
   Capas, autoridad, contratos y despliegue.
5. **Percepcion y memoria**
   Observaciones, entidades, mapas y recuperacion.
6. **Orquestacion y seguridad**
   MissionSpec, maquinas de estado y supervisor.
7. **Metodologia experimental**
   Escenarios, baselines, metricas y protocolo.
8. **Resultados**
   Sistema completo, comparativas y ablaciones.
9. **Discusion**
   Fallos, privacidad, coste y generalizacion.
10. **Limitaciones y etica**
    Personas, datos, afirmaciones de seguridad.
11. **Conclusion**
    Respuesta a hipotesis y trabajo futuro.

## Figuras y tablas del paper

- diagrama de autoridad extremo a extremo;
- secuencia instruccion-movimiento-evidencia;
- modelo de memoria;
- arquitectura de seguridad;
- mapa de despliegue;
- tabla de componentes reutilizados;
- tabla de benchmarks visuales;
- resultados de navegacion;
- matriz de fallos;
- ablaciones;
- distribucion de latencia y distancia de parada.

## Contribuciones que si pueden defenderse

1. Arquitectura de autoridad desacoplada para G1 agentico.
2. Contrato unificado de mision, skill, navegacion y movimiento seguro.
3. Memoria espacial con procedencia y vigencia.
4. Benchmark integrado de instruccion, percepcion, memoria, navegacion y fallo.
5. Evidencia empirica de los trade-offs, si los ensayos la confirman.

No debe presentarse como contribucion propia FAST-LIO2, YOLOE, MCP, LangGraph,
PostGIS, Nav2, Rerun ni los SDK de Unitree.

## Paquete de replicacion

- configuraciones sin secretos;
- contratos y esquemas;
- escenarios;
- scripts de metricas;
- versiones de modelos;
- datos anonimizados publicables;
- episodios sinteticos/replay cuando los reales no puedan publicarse;
- instrucciones de hardware y calibracion;
- lista de desviaciones del protocolo.
