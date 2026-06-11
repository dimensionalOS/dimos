# Odometria y TF frames

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar los frames principales y por que la coherencia TF es una brecha importante.

## Explicacion conceptual desde cero

Un frame es un sistema de coordenadas. Para navegar o proyectar objetos 3D, el robot necesita saber transformaciones entre frames: camara, lidar, cuerpo, odom y mapa. Si los nombres no coinciden, un modulo puede estar publicando datos correctos pero otro no puede usarlos.

## Archivos, clases y funciones relevantes

- `dimos/navigation/nav_stack/frames.py`: `FRAME_MAP="map"`, `FRAME_ODOM="odom"`, `FRAME_BODY="body"`, `FRAME_SENSOR="sensor"`.
- `dimos/hardware/sensors/lidar/fastlio2/module.py`: publica TF `odom -> body`.
- `dimos/navigation/nav_stack/modules/pgo/pgo.py`: publica TF `map -> odom`.
- `dimos/hardware/sensors/camera/module.py`: publica `camera_link -> camera_optical`.
- `dimos/robot/unitree/g1/blueprints/primitive/uintree_g1_primitive_no_nav.py`: camara en `sensor -> camera_link`.
- `dimos/perception/spatial_perception.py`: busca TF `world -> base_link`.

## Flujo de datos

```text
Nav onboard:
map -> odom -> body

Camara G1 primitive:
sensor -> camera_link -> camera_optical

SpatialMemory actual:
world -> base_link
```

## Inputs y outputs

Inputs:

- TFs de FastLIO2, PGO y CameraModule.
- Mensajes con `frame_id`.

Outputs:

- Poses en `map`, `world`, `body`, `base_link` o `camera_link`, segun modulo.

## Como se conecta con otras partes

La navegacion y deteccion 3D necesitan TF para:

- Saber donde esta el robot.
- Transformar pointclouds.
- Proyectar detecciones de camara a mundo.
- Guardar frames de memoria con pose.

## Que funciona hoy en G1 real

El nav stack define frames `map/odom/body/sensor`. FastLIO2 y PGO publican TFs coherentes dentro de ese stack. CameraModule publica TF de camara.

## Que parece incompleto o separado

Hay mezcla de nombres:

- `SpatialMemory` busca `world -> base_link`.
- `PersonTracker` busca `world -> camera_link`.
- Nav stack usa `map -> body` y `sensor`.

Para integrar todo, hay que normalizar frames o publicar aliases TF.

## Notas para mi proyecto/paper

Muchos fallos de robotica embodied parecen "LLM error", pero son errores de frames. Documentar y validar TF debe ser parte del protocolo experimental.
