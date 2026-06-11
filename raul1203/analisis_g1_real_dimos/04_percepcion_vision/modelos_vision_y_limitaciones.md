# Modelos de vision y limitaciones

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Identificar modelos usados para vision: YOLO, Qwen, CLIP y su papel real.

## Explicacion conceptual desde cero

DimOS usa varios tipos de modelos visuales:

- YOLO: deteccion rapida de objetos/personas en imagen.
- Qwen-VL: modelo vision-lenguaje usado para preguntas y bboxes por texto.
- CLIP: embeddings de imagen/texto para memoria espacial.

Cada modelo resuelve un problema distinto. No son intercambiables.

## Archivos, clases y funciones relevantes

- YOLO general: `dimos/perception/detection/detectors/yolo.py`
- YOLO persona: `dimos/perception/detection/detectors/person/yolo.py`
- Qwen: `dimos/models/vl/qwen.py`
- Bbox por texto: `dimos/navigation/visual/query.py`
- CLIP memoria legacy: `dimos/agents_deprecated/memory/image_embedding.py`
- Spatial DB: `dimos/agents_deprecated/memory/spatial_vector_db.py`
- SpatialMemory: `dimos/perception/spatial_perception.py`

## Flujo de datos

```text
YOLO:
Image -> ultralytics YOLO.track -> ImageDetections2D -> Detection2DArray

Qwen:
Image + prompt textual -> QwenVlModel.query -> JSON bbox -> ObjectTracking.track

CLIP:
Image frame -> embedding -> ChromaDB
Text query -> text embedding -> nearest frame metadata -> pose
```

## Inputs y outputs

YOLO:

- Input: imagen.
- Output: clases, scores, bboxes, tracks.

Qwen:

- Input: imagen y descripcion.
- Output: texto o bbox JSON.

CLIP:

- Input: imagenes y textos.
- Output: embeddings y distancias semanticas.

## Como se conecta con otras partes

YOLO conecta con `Detection2DModule`, `Detection3DModule` y `PersonTracker`. Qwen conecta con `navigate_with_text` para buscar un objeto visible. CLIP conecta con `SpatialMemory.query_by_text`.

## Que funciona hoy en G1 real

Los modelos estan cableados en codigo. YOLO esta en el blueprint `unitree-g1-detection`. Qwen se instancia en `NavigationSkillContainer`. CLIP/ChromaDB se usa en `SpatialMemory`.

## Que parece incompleto o separado

- YOLO detection no alimenta automaticamente el agente G1 full.
- Qwen bbox depende de tener imagen reciente y de respuesta JSON correcta.
- CLIP devuelve frames similares, no objetos persistentes.
- No hay una capa unificada que compare YOLO, Qwen y memoria para confirmar objetivos.

## Notas para mi proyecto/paper

Una mejora publicable seria una memoria semantica multi-fuente: YOLO para deteccion, Qwen para descripcion, CLIP para retrieval y pointcloud/SLAM para ubicacion geometrica.
