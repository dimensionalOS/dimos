# Deteccion 3D y boxes

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar como DimOS convierte una deteccion 2D en una deteccion 3D con pointcloud.

## Explicacion conceptual desde cero

Una bbox 2D solo dice "este rectangulo en la imagen contiene algo". Para obtener 3D, DimOS proyecta la nube de puntos al plano de la camara, selecciona los puntos que caen dentro del rectangulo y limpia esa subnube con filtros.

## Archivos, clases y funciones relevantes

- `dimos/perception/detection/type/detection3d/pointcloud.py`: `Detection3DPC.from_2d`
- `dimos/perception/detection/type/detection3d/imageDetections3DPC.py`
- `dimos/perception/detection/type/detection3d/bbox.py`
- `dimos/perception/detection/type/detection3d/pointcloud_filters.py`
- `dimos/perception/detection/module3D.py`

## Flujo de datos

```text
Detection2DBBox + world_pointcloud + camera_info + world_to_optical_transform
  -> convertir pointcloud world a frame camara
  -> descartar puntos detras de camara
  -> proyectar puntos 3D a pixeles 2D
  -> filtrar puntos dentro de bbox
  -> aplicar filtros: raycast, radius_outlier, statistical
  -> Detection3DPC(pointcloud, center, pose)
```

## Inputs y outputs

Inputs:

- `Detection2DBBox`
- `PointCloud2`
- `CameraInfo`
- `Transform`

Outputs:

- `Detection3DPC`
- centro 3D.
- pose.
- bounding box de pointcloud.

## Como se conecta con otras partes

`Detection3DModule.start()` alinea detecciones 2D con pointcloud usando `align_timestamped`. Luego publica pointclouds recortadas para las tres detecciones principales.

## Que funciona hoy en G1 real

El blueprint `unitree-g1-detection` esta preparado para robot real si hay `color_image`, `pointcloud` y TF coherente.

## Que parece incompleto o separado

No esta incluido en `unitree-g1-full`. Tampoco aparece conectado a `SpatialMemory` para guardar objetos. Si TF o calibracion de camara no es precisa, la proyeccion 3D falla o recorta puntos equivocados.

## Notas para mi proyecto/paper

Esta parte es central para claims de grounding fisico. Conviene medir error de bbox 3D, estabilidad temporal y tasa de falsos positivos antes de integrarla con navegacion autonoma.
