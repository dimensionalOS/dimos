# Ejemplos de tareas

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Mostrar ejemplos concretos: buscar objeto, navegar a lugar, recordar y volver.

## Explicacion conceptual desde cero

Un ejemplo ayuda a ver que subsistemas se activan y donde se rompe el stack actual.

## Archivos, clases y funciones relevantes

- `dimos/agents/skills/navigation.py`
- `dimos/perception/spatial_perception.py`
- `dimos/perception/detection/module3D.py`
- `dimos/navigation/nav_stack/main.py`
- `dimos/agents/skills/speak_skill.py`

## Flujo de datos

### Ejemplo 1: "busca a una persona"

Ideal:

```text
texto -> agente -> deteccion persona YOLO
  -> PersonTracker target
  -> nav/visual servoing o follow
  -> speak estado
```

Actual:

- `unitree-g1-detection` puede detectar/person track.
- `unitree-g1-full` no incluye ese blueprint.

### Ejemplo 2: "ve a la cocina"

Ideal:

```text
texto -> navigate_with_text("cocina")
  -> tagged location o SpatialMemory.query_by_text
  -> PoseStamped
  -> nav onboard
```

Actual:

- `SpatialMemory.query_by_text` existe.
- `NavigationSkillContainer` existe.
- Falta proveedor `NavigationInterfaceSpec` hacia nav onboard.

### Ejemplo 3: "recuerda este lugar como base y vuelve luego"

Ideal/actual parcial:

```text
tag_location("base")
  -> pose actual -> RobotLocation -> ChromaDB location collection

navigate_with_text("base")
  -> query_tagged_location
  -> PoseStamped
  -> _navigation.set_goal
```

Actual:

- Tagging existe si hay odom/TF compatible.
- Volver requiere navegacion integrada.

## Inputs y outputs

Inputs:

- Texto.
- Camara.
- Lidar/odom.
- Memoria previa.

Outputs:

- Tags.
- Goals.
- Movimiento.
- Voz.

## Como se conecta con otras partes

Estos ejemplos cruzan casi todos los subsistemas. Son buenos tests end-to-end.

## Que funciona hoy en G1 real

Cada ejemplo tiene piezas, pero los ejemplos 1 y 2 requieren fusionar blueprints. El ejemplo 3 requiere resolver TF y nav.

## Que parece incompleto o separado

Faltan contratos claros para:

- Memoria de objetos.
- Goals seguros.
- Estado de llegada.
- Confirmacion del usuario.

## Notas para mi proyecto/paper

Usar estos tres ejemplos como escenarios experimentales: "person search", "semantic place navigation", "teach-and-return".
