# Yanapaq G1

Ultima modificacion: 2026-06-12 16:23:07 -05 -0500

## Objetivo

Yanapaq G1 es la version enfocada en Unitree G1 de un sistema de mision
semantica persistente sobre DimOS. El objetivo inicial no es crear otro runtime,
sino integrar y endurecer piezas existentes de DimOS para que el G1 pueda:

- navegar en interiores conocidos;
- recordar habitaciones y objetos;
- verificar si una memoria sigue vigente;
- actualizar objetos movidos o ausentes;
- operar con personas cerca de forma conservadora;
- usar OpenAI API para lenguaje y verificacion visual;
- evitar que el LLM controle velocidades o actuadores directamente.

## Decision de implementacion

Se trabajara dentro del clone/fork de DimOS, no en un repo separado. El primer
blueprint propio debe llamarse:

```text
unitree-g1-yanapaq-dev
```

Ruta esperada:

```text
dimos/robot/unitree/g1/blueprints/yanapaq/unitree_g1_yanapaq_dev.py
```

La base tecnica principal sera:

```text
unitree-g1-nav-onboard
```

Se reutilizara de DimOS:

- `G1HighLevelDdsSdk`;
- `FastLio2`;
- `create_nav_stack`;
- `MovementManager`;
- `Yoloe2DDetector`;
- `ObjectSceneRegistrationModule`;
- `SpatialMemory` y `TemporalMemory` solo como referencia parcial;
- MCP/skills solo con allowlist segura.

## Documentos activos

```text
00_contexto_minimo_agente.md
01_prompts_para_vibecodear.md
02_plan_de_trabajo_codigo.md
paper/
figuras/
```

## Documentos historicos

Los analisis anteriores estan en:

```text
../99_archivo_no_leer/
```

No son basura: son respaldo. Pero son demasiado largos para el contexto inicial
de un agente.
