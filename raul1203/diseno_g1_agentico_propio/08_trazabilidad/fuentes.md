# Fuentes y evidencia consultada

Ultima modificacion: 2026-06-11 12:07:27 -05 -0500

Fecha de consulta de enlaces externos: **2026-06-11**.

Se priorizaron repositorios oficiales, documentacion de los proyectos y papers
primarios. Las caracteristicas citadas no sustituyen benchmarks en el hardware
objetivo.

## Evidencia local DimOS

Base inspeccionada:

- repositorio local `/home/raul/dimos`;
- rama `raul/documentacion`;
- commit `06606d6f6ab767b659c597cc5bfe8e2a4eb56525`;
- documentacion previa `raul1203/analisis_g1_real_dimos/`.

Archivos clave consultados:

- `dimos/robot/unitree/g1/blueprints/`;
- `dimos/robot/unitree/g1/system_prompt.py`;
- `dimos/robot/unitree/g1/g1_connection.py`;
- `dimos/robot/unitree/g1/g1_high_level_dds.py`;
- `dimos/robot/unitree/g1/g1_whole_body_connection.py`;
- `dimos/robot/unitree/g1/unitree_g1_skill_container.py`;
- `dimos/agents/mcp/mcp_client.py`;
- `dimos/agents/mcp/mcp_server.py`;
- `dimos/agents/skills/navigation.py`;
- `dimos/perception/`;
- `dimos/navigation/`;
- `dimos/mapping/`;
- `pyproject.toml`.

## Unitree y sensores

| Fuente primaria | Uso |
|---|---|
| [Unitree G1 Developer Guide](https://support.unitree.com/home/en/G1_developer) | Plataforma y guias oficiales |
| [Unitree DDS Services Interface](https://support.unitree.com/home/en/G1_developer/dds_services_interface) | Interfaces DDS del G1 |
| [unitree_sdk2](https://github.com/unitreerobotics/unitree_sdk2) | SDK oficial C++ |
| [unitree_sdk2_python](https://github.com/unitreerobotics/unitree_sdk2_python) | SDK oficial Python y licencia |
| [Livox Mid-360 specifications](https://www.livoxtech.com/mid-360/specs) | FOV, alcance, tasa, tiempo e IMU |
| [RealSense D455f](https://realsenseai.com/products/real-sense-depth-camera-d455f/) | Caracteristicas de RGB-D candidato |
| [Stereolabs ZED 2i](https://www.stereolabs.com/store/products/zed-2i) | Caracteristicas de estereo candidato |

## Vision y tracking

| Fuente primaria | Uso |
|---|---|
| [YOLOE official repository](https://github.com/THU-MIG/yoloe) | Codigo, modelos y modos de prompt |
| [YOLOE paper](https://arxiv.org/abs/2503.07465) | Metodo y evaluacion reportada |
| [YOLO-World official repository](https://github.com/AILAB-CVC/YOLO-World) | Implementacion oficial |
| [YOLO-World CVPR 2024 paper](https://openaccess.thecvf.com/content/CVPR2024/html/Cheng_YOLO-World_Real-Time_Open-Vocabulary_Object_Detection_CVPR_2024_paper.html) | Metodo y resultados |
| [Grounding DINO official repository](https://github.com/IDEA-Research/GroundingDINO) | Implementacion y licencia |
| [Grounding DINO ECCV paper](https://www.ecva.net/papers/eccv_2024/papers_ECCV/papers/06319.pdf) | Descripcion primaria |
| [OWLv2 model card](https://huggingface.co/google/owlv2-large-patch14) | Pesos/configuracion oficial |
| [OWLv2 paper](https://arxiv.org/html/2306.09683v3) | Metodo de escalado |
| [OWL-ViT paper](https://arxiv.org/abs/2205.06230) | Antecedente open-vocabulary de OWLv2 |
| [RT-DETR official repository](https://github.com/lyuwenyu/RT-DETR) | Baseline cerrado candidato |
| [SAM 2 official repository](https://github.com/facebookresearch/sam2) | Segmentacion de imagen/video |
| [ByteTrack official repository](https://github.com/FoundationVision/ByteTrack) | Tracker candidato |
| [BoT-SORT official repository](https://github.com/NirAharon/BoT-SORT) | Tracker usado como baseline |
| [Depth Anything V2 official repository](https://github.com/DepthAnything/Depth-Anything-V2) | Profundidad monocular y licencias por modelo |
| [Metric3D official repository](https://github.com/YvanYin/Metric3D) | Candidato de profundidad metrica y licencia |

## LiDAR, SLAM y reconstruccion

| Fuente primaria | Uso |
|---|---|
| [FAST-LIO official repository](https://github.com/hku-mars/FAST_LIO) | Implementacion de referencia |
| [FAST-LIO2 paper](https://arxiv.org/abs/2107.06829) | Metodo LIO baseline |
| [FAST-LIVO2 official repository](https://github.com/hku-mars/FAST-LIVO2) | Candidato LiDAR-inercial-visual |
| [FAST-LIVO2 paper](https://arxiv.org/html/2408.14035v2) | Metodo y experimentos |
| [Point-LIO official repository](https://github.com/hku-mars/Point-LIO) | Candidato de odometria |
| [LIO-SAM official repository](https://github.com/TixiaoShan/LIO-SAM) | Referencia de grafo de factores |
| [KISS-ICP official repository](https://github.com/PRBonn/kiss-icp) | Baseline diagnostico |
| [RTAB-Map official site](https://introlab.github.io/rtabmap/) | SLAM y memoria multimodal |
| [Open3D TSDF integration](https://www.open3d.org/docs/latest/tutorial/t_reconstruction_system/integration.html) | Reconstruccion TSDF |
| [Voxblox official repository](https://github.com/ethz-asl/voxblox) | TSDF/ESDF candidato |
| [nvblox paper](https://arxiv.org/html/2311.00626v2) | Reconstruccion GPU candidata |
| [Kalibr official repository](https://github.com/ethz-asl/kalibr) | Calibracion camara-IMU/temporal |
| [ROS 2 Approximate Time Synchronizer](https://docs.ros.org/en/rolling/p/message_filters/doc/Tutorials/Approximate-Synchronizer-Python.html) | Sincronizacion aproximada candidata |

## Navegacion

| Fuente primaria | Uso |
|---|---|
| [Nav2 concepts](https://docs.nav2.org/concepts/index.html) | Arquitectura y conceptos |
| [Nav2 MPPI controller](https://docs.nav2.org/configuration/packages/configuring-mppic.html) | Plan local candidato |
| [Nav2 DWB controller](https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html) | Baseline local alterno |
| [Nav2 costmaps](https://docs.nav2.org/configuration/packages/configuring-costmaps.html) | Capas de coste |
| [TEB local planner](https://github.com/rst-tu-dortmund/teb_local_planner) | Planificador local candidato |
| [Nav2 social costmap plugin](https://github.com/robotics-upo/nav2_social_costmap_plugin) | Capa social candidata |
| [ORCA](https://gamma.cs.unc.edu/ORCA/) | Evitacion reciproca multiagente |
| [SPL paper](https://vladlen.info/papers/navigation-evaluation.pdf) | Metrica de navegacion |
| [SCT paper](https://arxiv.org/abs/2103.08022) | Metrica temporal |
| [evo official repository](https://github.com/MichaelGrupp/evo) | ATE/RPE y evaluacion de trayectoria |

## Memoria y almacenamiento

| Fuente primaria | Uso |
|---|---|
| [PostGIS manual](https://postgis.net/docs/using_postgis_dbmanagement.html) | Tipos y consultas espaciales |
| [pgvector official repository](https://github.com/pgvector/pgvector) | Busqueda vectorial en PostgreSQL |
| [Qdrant search documentation](https://qdrant.tech/documentation/search/search/) | Alternativa vectorial |
| [SQLite R-Tree](https://sqlite.org/rtree.html) | Alternativa embebida espacial |

## Agentes, MCP y modelos

| Fuente primaria | Uso |
|---|---|
| [MCP specification 2025-06-18](https://modelcontextprotocol.io/specification/2025-06-18) | Contrato de protocolo |
| [MCP security best practices](https://modelcontextprotocol.io/docs/tutorials/security/security_best_practices) | Amenazas y mitigaciones |
| [NSA CSI: Securely Using MCP](https://www.nsa.gov/Portals/75/documents/Cybersecurity/CSI_MCP_SECURITY.pdf?ver=bmgiSbNQLP6Z_GiWtRt6bg%3D%3D) | Guia de seguridad, mayo de 2026 |
| [LangGraph overview](https://docs.langchain.com/oss/python/langgraph/overview) | Runtime de agentes |
| [LangGraph persistence](https://docs.langchain.com/oss/python/langgraph/persistence) | Checkpoints |
| [LangGraph interrupts](https://docs.langchain.com/oss/python/langgraph/interrupts) | Pausa y reanudacion |
| [Qwen3 official repository](https://github.com/QwenLM/Qwen3) | Modelo local candidato |
| [Qwen function calling](https://qwen.readthedocs.io/en/latest/framework/function_call.html) | Tool use oficial |
| [llama.cpp official repository](https://github.com/ggml-org/llama.cpp) | Runtime local cuantizado |

## Voz

| Fuente primaria | Uso |
|---|---|
| [OpenAI Whisper official repository](https://github.com/openai/whisper) | ASR de referencia |
| [faster-whisper official repository](https://github.com/SYSTRAN/faster-whisper) | Runtime ASR local |
| [Silero VAD official repository](https://github.com/snakers4/silero-vad) | VAD candidato |
| [Piper active repository](https://github.com/OHF-Voice/piper1-gpl) | TTS local y licencia actual |
| [Piper archived repository](https://github.com/rhasspy/piper) | Historial y redireccion del proyecto |

## Observabilidad

| Fuente primaria | Uso |
|---|---|
| [Rerun overview](https://rerun.io/docs/overview/what-is-rerun) | Visualizacion multimodal |
| [Foxglove documentation](https://docs.foxglove.dev/docs) | Visualizador candidato |
| [OpenTelemetry specification](https://opentelemetry.io/docs/specs/otel/) | Trazas, metricas y logs |
| [MCAP specification](https://mcap.dev/spec) | Formato de episodios |

## Limitaciones de la consulta

- La documentacion externa cambia; cada implementacion debe fijar version o
  commit.
- Las licencias de codigo, pesos y datasets pueden diferir dentro del mismo
  proyecto.
- Las cifras de papers no se consideran rendimiento del G1.
- No se ejecutaron benchmarks ni comandos sobre un robot real para esta
  propuesta.
- Las fuentes sustentan candidatos y conceptos; las decisiones propias estan
  registradas en [`decisiones_tecnicas.md`](decisiones_tecnicas.md).
