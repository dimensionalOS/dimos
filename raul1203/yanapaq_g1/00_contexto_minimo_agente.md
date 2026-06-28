# Contexto minimo para agentes

Ultima modificacion: 2026-06-12 16:23:07 -05 -0500

## Resumen corto

Estamos construyendo **Yanapaq G1** dentro del repo DimOS. La meta es crear un
blueprint completo para Unitree G1 que una navegacion, seguridad, memoria
semantica, percepcion y OpenAI API, reutilizando tanto DimOS como sea posible.

No hay que leer todo `raul1203/`. La carpeta historica `99_archivo_no_leer/`
solo se consulta si se pide explicitamente.

## Decision principal

Trabajar en este repo:

```text
/home/raul/dimos
```

con una rama nueva:

```bash
git checkout main
git pull origin main
git checkout -b feat/yanapaq-g1-mvp
```

No clonar DimOS de nuevo al inicio. Crear otro clone solo si se necesita un
ambiente limpio para pruebas destructivas o comparacion contra upstream.

## Base DimOS

La base principal sera:

```text
unitree-g1-nav-onboard
```

Razon: es el stack G1 real mas cercano a movimiento util, porque contiene
LiDAR, FAST-LIO2, navegacion, planificador, MovementManager y DDS/SDK2.

No usar `unitree-g1-agentic` como base principal. Sirve como referencia de
MCP/skills, pero expone movimiento directo al agente y no integra la memoria
semantica activa requerida.

## Blueprint objetivo

Crear:

```text
unitree-g1-yanapaq-dev
```

Ruta esperada:

```text
dimos/robot/unitree/g1/blueprints/yanapaq/unitree_g1_yanapaq_dev.py
```

Primero debe arrancar casi igual que `unitree-g1-nav-onboard`. Luego se agregan
modulos Yanapaq de forma incremental.

## Reglas tecnicas

- No modificar DimOS core si no es necesario.
- No romper blueprints existentes.
- No exponer `move`, `move_velocity` ni `cmd_vel` al LLM.
- Todo movimiento debe pasar por un supervisor.
- OpenAI API se usa para lenguaje y verificacion visual, no para control.
- No usar ROS2 como stack principal.
- No usar servidores locales de LLM/VLM.
- Empezar con texto/comandos estructurados; lenguaje natural va al final.
- Cada cambio debe tener prueba o verificacion minima.

## Orden de implementacion

1. Crear blueprint `unitree-g1-yanapaq-dev` basado en navegacion G1.
2. Insertar `SafetySupervisor` entre navegacion y adaptador G1.
3. Crear `MissionExecutive` con maquina de estados simple.
4. Crear memoria SQLite minima para objetos, rooms y observaciones.
5. Integrar YOLOE como detector de candidatos.
6. Integrar OpenAI VLM solo para verificacion por incertidumbre.
7. Integrar MCP/LLM con allowlist de tools de alto nivel.

## Primer caso de prueba

Caso unico inicial:

```text
"Revisa si la lonchera azul sigue en la cocina. Si no esta, buscala en
habitaciones cercanas y actualiza tu memoria."
```

Ese caso fuerza a resolver: memoria persistente, verificacion visual,
navegacion, objetos movidos, busqueda activa y actualizacion.
