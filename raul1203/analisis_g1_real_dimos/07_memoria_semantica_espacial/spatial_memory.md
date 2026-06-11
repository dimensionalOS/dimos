# Spatial memory

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar que es memoria espacial en DimOS exactamente y que recuerda realmente.

## Explicacion conceptual desde cero

`SpatialMemory` guarda imagenes del entorno asociadas a poses del robot. Cada cierto tiempo, toma el ultimo frame, busca la pose por TF, genera un embedding visual y lo guarda en ChromaDB con metadata de posicion y rotacion.

Esto permite queries como "donde vi algo parecido a una cocina", porque CLIP puede comparar texto con imagenes. Pero el resultado es "un frame parecido en esta pose", no "un objeto detectado con identidad persistente".

## Archivos, clases y funciones relevantes

- `dimos/perception/spatial_perception.py`: `SpatialMemory`
- `dimos/perception/spatial_memory_spec.py`: `SpatialMemorySpec`
- `dimos/agents_deprecated/memory/spatial_vector_db.py`: `SpatialVectorDB`
- `dimos/agents_deprecated/memory/image_embedding.py`: `ImageEmbeddingProvider`
- `dimos/types/robot_location.py`: `RobotLocation`
- `dimos/agents/skills/navigation.py`: `tag_location`, `navigate_with_text`

## Flujo de datos

```text
color_image -> SpatialMemory._latest_video_frame
TF world -> base_link -> current_pose
frame -> ImageEmbeddingProvider(CLIP) -> embedding
embedding + metadata(pos_x,pos_y,pos_z,rot_x,rot_y,rot_z,timestamp,frame_id)
  -> SpatialVectorDB -> ChromaDB
```

Query:

```text
query_by_text("kitchen")
  -> text embedding
  -> ChromaDB nearest frames
  -> metadata
  -> NavigationSkillContainer -> PoseStamped
```

Tags:

```text
tag_location("charging station")
  -> RobotLocation(name, position, rotation)
  -> location_collection in ChromaDB
```

## Inputs y outputs

Inputs:

- `color_image`.
- TF pose.
- Text query.
- Named location.

Outputs:

- Query results con metadata.
- Tagged locations.
- Stats de frames procesados/guardados.

## Como se conecta con otras partes

`NavigationSkillContainer` usa:

- `tag_location` para guardar ubicaciones nombradas.
- `query_tagged_location` para volver a lugares nombrados.
- `query_by_text` para buscar frames semanticamente parecidos.

## Que funciona hoy en G1 real

`SpatialMemory` esta incluido en `unitree-g1` y por tanto en `unitree-g1-agentic/full`. Usa ChromaDB persistente por defecto en `assets/output/memory/spatial_memory/chromadb_data`.

## Que parece incompleto o separado

- Busca TF `world -> base_link`, que puede no coincidir con nav stack `map -> body`.
- Guarda frames, no objetos.
- `query_by_location` parece buscar metadata `x/y`, pero `SpatialMemory` guarda `pos_x/pos_y`, posible mismatch.
- `new_memory=True` por defecto puede borrar memoria previa al iniciar.

## Notas para mi proyecto/paper

La memoria actual es una buena base para "visual place recognition". Para "semantic object memory" falta asociar detecciones 3D, identidad, timestamps, confianza y relaciones espaciales.
