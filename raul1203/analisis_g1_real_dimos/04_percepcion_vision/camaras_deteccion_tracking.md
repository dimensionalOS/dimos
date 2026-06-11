# Camaras, deteccion y tracking

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar como ve el G1 real en DimOS, que deteccion existe y que limitaciones tiene.

## Explicacion conceptual desde cero

El G1 agentic/full usa una webcam local por `CameraModule`. Esa camara produce `color_image` y `camera_info`, y publica TF de `camera_link` y `camera_optical`.

La deteccion de objetos/personas no esta dentro de `unitree-g1-full`; esta en `unitree-g1-detection`. Ese blueprint toma imagen + pointcloud, aplica YOLO y proyecta bboxes 2D a 3D usando la nube de puntos y TF.

## Archivos, clases y funciones relevantes

- `dimos/hardware/sensors/camera/module.py`: `CameraModule`
- `dimos/hardware/sensors/camera/webcam.py`: `Webcam`
- `dimos/robot/unitree/g1/blueprints/primitive/uintree_g1_primitive_no_nav.py`: `_create_webcam`
- `dimos/robot/unitree/g1/blueprints/perceptive/unitree_g1_detection.py`
- `dimos/perception/detection/module2D.py`: `Detection2DModule`
- `dimos/perception/detection/module3D.py`: `Detection3DModule`
- `dimos/perception/detection/moduleDB.py`: `ObjectDBModule`
- `dimos/perception/detection/person_tracker.py`: `PersonTracker`
- `dimos/perception/object_tracker.py`: `ObjectTracking`

## Flujo de datos

### Camara en G1 agentic/full

```text
Webcam(camera_index=0, fps=15, stereo_slice="left")
  -> CameraModule
  -> color_image: Out[Image]
  -> camera_info: Out[CameraInfo]
  -> TF sensor -> camera_link -> camera_optical
```

### Deteccion 3D en blueprint separado

```text
color_image -> YoloPersonDetector -> detections_2d
detections_2d + pointcloud + TF -> Detection3DModule
Detection3DModule -> detected pointcloud/image crops
ObjectDBModule -> objetos temporales
PersonTracker -> target PoseStamped
```

## Inputs y outputs

Inputs:

- Imagen RGB.
- CameraInfo.
- PointCloud2.
- TF world/camera.

Outputs:

- `Detection2DArray`.
- Pointclouds recortadas de objetos.
- Imagenes recortadas.
- `target: PoseStamped` para persona.

## Como se conecta con otras partes

`NavigationSkillContainer` puede pedir un bbox con Qwen sobre la imagen actual y luego llamar `_object_tracking.track(bbox)`. En `unitree-g1-full`, `_object_tracking` puede resolverse a `ObjectTracking`, pero ese modulo necesita `depth` para tracking 3D robusto y el G1 base solo crea webcam monocular.

## Que funciona hoy en G1 real

- Camara RGB por webcam en blueprints G1 reales.
- Deteccion YOLO/persona en `unitree-g1-detection`.
- Proyeccion 3D basada en pointcloud en `Detection3DModule`.

## Que parece incompleto o separado

- Deteccion 3D/persona no esta conectada al agente.
- `ObjectTracking` incluido en `unitree_g1` espera depth, pero la camara base no publica depth.
- La memoria espacial no guarda objetos con identidad persistente; guarda frames/poses.

## Notas para mi proyecto/paper

La linea de investigacion fuerte es fusionar bboxes 2D, pointcloud, memoria y navegacion: "veo una silla", "la ubico en mapa", "la recuerdo", "vuelvo a ella".
