# Memoria espacial vs memoria de objetos

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Responder si DimOS guarda objetos o solo frames/poses, y que le falta para memoria de objetos.

## Explicacion conceptual desde cero

Una memoria de lugar dice: "en esta pose vi una escena parecida a X". Una memoria de objeto dice: "este objeto X esta en esta ubicacion, con esta geometria, lo vi estas veces, con esta confianza". DimOS G1 actual esta mas cerca de memoria de lugar.

## Archivos, clases y funciones relevantes

- Lugar/frames: `dimos/perception/spatial_perception.py`
- Vector DB: `dimos/agents_deprecated/memory/spatial_vector_db.py`
- Objetos temporales: `dimos/perception/detection/moduleDB.py`
- Deteccion 3D: `dimos/perception/detection/type/detection3d/pointcloud.py`
- ReID experimental: `dimos/perception/detection/reid/embedding_id_system.py`

## Flujo de datos

Memoria actual:

```text
frame + pose -> CLIP vector -> ChromaDB -> nearest frame
```

Memoria de objetos deseada:

```text
YOLO/Qwen bbox -> pointcloud crop -> Detection3DPC
  -> object association / re-id
  -> object_id + class/name + pose + bbox3d + confidence
  -> semantic DB
  -> query "where is the red cup?"
```

## Inputs y outputs

Inputs actuales:

- Frames.
- Poses.
- Tags.

Inputs necesarios para objetos:

- Detecciones 2D/3D.
- Pointclouds.
- ReID visual.
- Confirmaciones temporales.

Outputs actuales:

- Frame metadata.
- RobotLocation.

Outputs deseados:

- ObjectLocation.
- Historial de observaciones.
- Estado de confianza.

## Como se conecta con otras partes

`ObjectDBModule` ya acumula `Object3D`, pero esta en `unitree-g1-detection`, no en `unitree-g1-full`, y no alimenta `SpatialMemory`.

## Que funciona hoy en G1 real

Hay piezas para ambos mundos: SpatialMemory para lugares y ObjectDBModule para objetos temporales. Pero no hay memoria unificada.

## Que parece incompleto o separado

- No hay persistencia robusta de objetos.
- No hay query MCP para "listar objetos recordados" en G1 full.
- No hay actualizacion de ubicacion de objeto al moverlo.
- No hay modelo de incertidumbre.

## Notas para mi proyecto/paper

La distincion lugar vs objeto puede ser una contribucion conceptual clara. Proponer "object-grounded spatial memory" sobre humanoide real seria mas fuerte que solo usar CLIP retrieval.
