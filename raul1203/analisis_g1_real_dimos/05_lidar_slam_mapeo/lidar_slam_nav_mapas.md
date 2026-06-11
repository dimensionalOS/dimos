# Lidar, SLAM y mapas

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar como usa lidar el G1 real, que hace FastLIO2 y que mapas se generan para navegacion.

## Explicacion conceptual desde cero

El lidar Mid-360 produce nubes de puntos. FastLIO2 combina lidar e IMU para estimar odometria local y registrar scans en un marco continuo. PGO puede corregir deriva con pose graph optimization y publicar una transformacion `map -> odom`.

La navegacion no usa "imagen" como mapa principal; usa pointclouds procesadas en mapas de terreno y costmaps.

## Archivos, clases y funciones relevantes

- `dimos/robot/unitree/g1/blueprints/navigation/unitree_g1_nav_onboard.py`
- `dimos/hardware/sensors/lidar/fastlio2/module.py`: `FastLio2`
- `dimos/navigation/nav_stack/main.py`: `create_nav_stack`
- `dimos/navigation/nav_stack/modules/pgo/pgo.py`: `PGO`
- `dimos/navigation/nav_stack/modules/terrain_analysis/terrain_analysis.py`
- `dimos/navigation/nav_stack/modules/terrain_map_ext/terrain_map_ext.py`
- `dimos/navigation/nav_stack/modules/simple_planner/simple_planner.py`
- `dimos/robot/unitree/g1/config.py`: `G1.internal_odom_offsets["mid360_link"]`

## Flujo de datos

```text
Livox Mid-360
  -> FastLio2
       outputs:
       - lidar: PointCloud2
       - odometry: Odometry
       - global_map: PointCloud2
       - TF odom -> body
  -> remap lidar -> registered_scan
  -> remap global_map -> global_map_fastlio
```

Luego:

```text
registered_scan + odometry
  -> PGO
       corrected_odometry
       global_map_pgo
       TF map -> odom
  -> TerrainAnalysis
       terrain_map
  -> TerrainMapExt
       terrain_map_ext
  -> planners
```

## Inputs y outputs

Inputs:

- `host_ip`, `lidar_ip`, puertos Livox.
- Lidar scans.
- Odometria de FastLIO2.

Outputs:

- `registered_scan`.
- `odometry`.
- `corrected_odometry`.
- `global_map_fastlio`.
- `global_map_pgo`.
- `terrain_map`.
- `terrain_map_ext`.

## Como se conecta con otras partes

`unitree-g1-nav-onboard` configura:

- `host_ip` por `LIDAR_HOST_IP` o `192.168.123.164`.
- `lidar_ip` por `LIDAR_IP` o `192.168.123.120`.
- Mount Mid-360 a 1.2 m en `G1`.
- `map_freq=1.0`.
- `create_nav_stack(planner="simple")`.

## Que funciona hoy en G1 real

El blueprint onboard esta claramente orientado a hardware real: usa FastLIO2 + Livox Mid-360 y Unitree DDS SDK.

## Que parece incompleto o separado

Este stack no incluye agente ni memoria. Tampoco expone directamente `NavigationInterfaceSpec`, por lo que no se conecta con `navigate_with_text` tal como esta.

## Notas para mi proyecto/paper

FastLIO2 + PGO + terrain maps dan la base geometrica para claims de navegacion real. Para paper, medir drift, frecuencia de mapas, calidad de costmap y tasa de llegada a goals.
