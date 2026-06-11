# Nav stack y texto a goal

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Responder como decide el sistema a donde navegar y como convierte texto en coordenada o goal.

## Explicacion conceptual desde cero

Hay dos problemas distintos:

1. Decidir un goal semantico: "ve a la cocina", "ve a la persona", "vuelve a la mesa".
2. Mover el robot hasta ese goal: planificacion geometrica, evitar obstaculos, emitir `cmd_vel`.

`NavigationSkillContainer` resuelve el problema 1 parcialmente. `unitree-g1-nav-onboard` resuelve el problema 2. Hoy no estan unidos por la misma interfaz.

## Archivos, clases y funciones relevantes

- `dimos/agents/skills/navigation.py`: `NavigationSkillContainer`
- `dimos/navigation/navigation_spec.py`: `NavigationInterfaceSpec`
- `dimos/navigation/replanning_a_star/module.py`: `ReplanningAStarPlanner`
- `dimos/navigation/nav_stack/main.py`: `create_nav_stack`
- `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- `dimos/navigation/nav_stack/modules/simple_planner/simple_planner.py`
- `dimos/navigation/nav_stack/modules/local_planner/local_planner.py`
- `dimos/navigation/nav_stack/modules/path_follower/path_follower.py`

## Flujo de datos

### Texto a goal en `NavigationSkillContainer`

```text
navigate_with_text(query)
  -> query_tagged_location(query)
       si existe: RobotLocation -> PoseStamped(map)
  -> _navigate_to_object(query)
       Qwen bbox en imagen -> ObjectTracking.track(bbox)
  -> query_by_text(query)
       CLIP/ChromaDB -> metadata pos_x/pos_y/rot_z -> PoseStamped(map)
  -> _navigation.set_goal(goal_pose)
```

### Movimiento geometrico en nav onboard

```text
goal stream -> SimplePlanner/FarPlanner -> way_point
way_point -> LocalPlanner -> path/effective_cmd_vel
path -> PathFollower -> nav_cmd_vel
nav_cmd_vel -> MovementManager -> cmd_vel
cmd_vel -> G1HighLevelDdsSdk
```

## Inputs y outputs

Inputs semanticos:

- `location_name` para tags.
- `query` textual.
- Imagen actual.
- Memoria espacial.

Inputs geometricos:

- `registered_scan`.
- `corrected_odometry`.
- `terrain_map`, `terrain_map_ext`.
- `goal`.

Outputs:

- `PoseStamped` semantico.
- `PointStamped` goal/waypoint en nav stack.
- `Twist` final.

## Como se conecta con otras partes

`NavigationSkillContainer` no publica goals por stream; llama `_navigation.set_goal(pose)`. El nav stack onboard no implementa ese metodo. ReplanningAStar si lo implementa, pero no esta incluido en los blueprints G1 reales principales.

## Que funciona hoy en G1 real

Nav onboard puede navegar con goals por streams. Texto a coordenada existe como logica en skill. La union exacta texto -> nav onboard falta.

## Que parece incompleto o separado

Hay que implementar un adaptador:

```text
NavigationInterfaceSpec.set_goal(PoseStamped)
  -> convertir PoseStamped a PointStamped
  -> publicar en stream "goal" o equivalente
  -> monitorear goal_reached/cancel
```

Tambien hay que resolver frame `map` vs `world`.

## Notas para mi proyecto/paper

La contribucion "text-to-navigation on humanoid real hardware" debe demostrar grounding semantico, transformacion a goal y ejecucion segura por planner.
