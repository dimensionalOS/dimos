# Comandos de inspeccion seguros

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Listar comandos que ayudan a inspeccionar sin mover el robot ni arrancar blueprints reales.

## Explicacion conceptual desde cero

Inspeccionar no es lo mismo que ejecutar. En un repo de robotica, cualquier comando `dimos run ...` puede abrir sensores, red o control de hardware. Los comandos de esta lista leen archivos, registro o estado.

## Archivos, clases y funciones relevantes

- `dimos/robot/cli/dimos.py`
- `dimos/robot/all_blueprints.py`
- `dimos/core/run_registry.py`

## Flujo de datos

```text
comando lectura -> stdout -> analisis humano
```

## Inputs y outputs

Inputs:

- Repo local.
- Archivos Python/Markdown.

Outputs:

- Lista de blueprints.
- Estado/logs si hay un proceso ya corriendo.
- Busquedas de codigo.

## Como se conecta con otras partes

Estos comandos sirven para validar documentacion y orientarse antes de tocar hardware.

## Que funciona hoy en G1 real

Comandos seguros para inspeccion:

```bash
git status --short
rg -n "unitree_g1|unitree-g1" dimos/robot/all_blueprints.py dimos/robot/unitree/g1
rg -n "NavigationInterfaceSpec|class ReplanningAStarPlanner|def set_goal" dimos/navigation
sed -n '1,220p' dimos/robot/unitree/g1/blueprints/agentic/unitree_g1_full.py
sed -n '1,220p' dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py
dimos list
dimos status
dimos log -n 50
```

`dimos list` solo lista nombres; no deberia arrancar robot. Si el entorno no tiene dependencias instaladas, puede fallar al importar.

## Que parece incompleto o separado

Una inspeccion estatica no prueba que el hardware funcione. Solo prueba que el codigo declara cierta composicion.

## Notas para mi proyecto/paper

Guardar outputs de inspeccion en anexos puede ayudar a reproducibilidad, pero los resultados experimentales deben venir de logs y rosbag/lcm reales.
