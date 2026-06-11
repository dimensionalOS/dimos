# Ideas para paper

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Identificar que partes del repo pueden servir como base para un paper propio.

## Explicacion conceptual desde cero

Un paper no deberia vender solo "use un LLM en un robot". Lo fuerte seria demostrar integracion y evaluacion en hardware humanoide real.

## Archivos, clases y funciones relevantes

- Base agentica: `dimos/agents/mcp/*`
- G1 agent: `dimos/robot/unitree/g1/blueprints/agentic/*`
- Nav real: `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- Memoria: `dimos/perception/spatial_perception.py`
- Detection: `dimos/perception/detection/*`
- Control: `dimos/robot/unitree/g1/effectors/high_level/*`

## Flujo de datos

Ideas de contribucion:

```text
LLM tools -> semantic memory -> grounded goal -> nav stack -> safety -> verified action
```

## Inputs y outputs

Inputs experimentales:

- Instrucciones en lenguaje natural.
- Escenas reales con objetos/lugares.
- Mapas lidar.
- Logs de navegacion.

Outputs medibles:

- Tasa de exito.
- Tiempo a completar.
- Error final al goal.
- Fallos de grounding.
- Intervenciones de safety.
- Calidad de memoria.

## Como se conecta con otras partes

DimOS da la infraestructura modular. Tu paper puede proponer el pegamento cientifico: memoria de objetos, adaptador nav, safety y evaluacion.

## Que funciona hoy en G1 real

Hay suficientes componentes para una base experimental real: agente, control, percepcion, lidar y nav. El trabajo interesante esta en integrarlos con rigor.

## Que parece incompleto o separado

La falta de integracion es una oportunidad, pero tambien un riesgo. Hay que convertir brechas en contribuciones concretas, no solo en TODOs.

## Notas para mi proyecto/paper

Titulos posibles:

- "Grounded Semantic Navigation for Humanoid Robots with Tool-Calling Agents"
- "From Visual Place Memory to Object-Grounded Action on a Unitree G1"
- "A Safety-Arbitrated LLM Tool Stack for Real Humanoid Navigation"

Experimentos recomendados:

- Teach-and-return a lugares.
- Buscar objeto visible vs recordado.
- Navegar a persona/objeto con deteccion 3D.
- Fallos controlados: objeto no visible, goal bloqueado, comando ambiguo.
