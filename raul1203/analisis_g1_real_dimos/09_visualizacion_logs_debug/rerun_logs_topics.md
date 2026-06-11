# Rerun, logs y topics

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar como ver que esta pasando en DimOS sin mover el robot.

## Explicacion conceptual desde cero

DimOS tiene tres capas de observabilidad:

- Logs por run: JSONL en `~/.local/state/dimos/logs/<run-id>/main.jsonl`.
- Topics LCM/pLCM: mensajes de streams.
- Rerun: visualizacion de imagenes, pointclouds, TF, paths, goals y mapas.

## Archivos, clases y funciones relevantes

- `dimos/robot/cli/dimos.py`: `status`, `log`, `topic echo`, `rerun-bridge`.
- `dimos/core/run_registry.py`
- `dimos/core/log_viewer.py`
- `dimos/visualization/vis_module.py`
- `dimos/visualization/rerun/bridge.py`
- `dimos/robot/unitree/g1/g1_rerun.py`
- `dimos/navigation/nav_stack/main.py`: `nav_stack_rerun_config`

## Flujo de datos

```text
Module Out/In -> Transport -> LCM topic
  -> RerunBridgeModule subscribe-all
  -> msg.to_rerun() o visual_override
  -> Rerun entity world/...
```

Logs:

```text
dimos run ...
  -> run_id
  -> log_dir
  -> main.jsonl
  -> dimos log -f
```

## Inputs y outputs

Inputs:

- Mensajes LCM.
- Logs estructurados.
- Rerun config.

Outputs:

- Vista 2D/3D.
- Web viewer si habilitado.
- Terminal logs.

## Como se conecta con otras partes

`vis_module(viewer_backend=global_config.viewer)` se incluye en blueprints G1 base y nav onboard. `unitree-g1-nav-onboard` usa `nav_stack_rerun_config` con overrides G1.

## Que funciona hoy en G1 real

Rerun esta integrado en los blueprints. La CLI permite:

- `dimos status`
- `dimos log`
- `dimos log -f`
- `dimos topic echo`
- `dimos rerun-bridge`
- `dimos mcp status`

## Que parece incompleto o separado

La visualizacion puede mostrar topics, pero no garantiza que los frames esten semanticamente alineados. Para debug serio hace falta una checklist de TF, topic rates, timestamps y latencias.

## Notas para mi proyecto/paper

Rerun puede servir para figuras del paper: mapa, trayectoria, detecciones 3D y tool calls. Guardar logs y screenshots reproducibles es clave.
