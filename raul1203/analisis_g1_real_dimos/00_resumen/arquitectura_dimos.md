# Arquitectura DimOS

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar los conceptos base de DimOS: modules, streams, transports, blueprints, RPC, specs, skills y MCP.

## Explicacion conceptual desde cero

Un `Module` es una unidad de comportamiento. Puede leer datos con `In[T]`, publicar datos con `Out[T]`, exponer metodos RPC con `@rpc` y exponer tools al agente con `@skill`.

Un stream es tipado. Por ejemplo, `color_image: Out[Image]` y `color_image: In[Image]` se pueden conectar porque tienen mismo nombre y mismo tipo.

Un transport es el bus fisico/logico que lleva datos. Ejemplos:

- `LCMTransport`: mensajes LCM tipados.
- `pLCMTransport`: objetos Python pickled por LCM, usado para `str`, LangChain messages, etc.
- `pSHMTransport`: shared memory pickled, util para imagenes grandes.
- `ROSTransport`: puente a ROS.
- DDS/SDK: usado por Unitree SDK2 en modulos G1 especificos.

Un `Blueprint` es una composicion de modulos. `autoconnect()` junta blueprints y elimina duplicados por clase de modulo.

Un `Spec` es una interfaz: un modulo puede pedir `_navigation: NavigationInterfaceSpec` y el coordinador busca otro modulo que implemente los metodos requeridos. Si no existe y no es opcional, falla.

## Archivos, clases y funciones relevantes

- `dimos/core/module.py`: `Module`, `ModuleBase`, `ModuleConfig`.
- `dimos/core/stream.py`: `In`, `Out`, `Transport`.
- `dimos/core/transport.py`: `LCMTransport`, `pLCMTransport`, `pSHMTransport`, `ROSTransport`.
- `dimos/core/coordination/blueprints.py`: `BlueprintAtom`, `Blueprint`, `autoconnect`.
- `dimos/core/coordination/module_coordinator.py`: `_connect_streams`, `_connect_module_refs`.
- `dimos/spec/utils.py`: `Spec`, `spec_structural_compliance`, `spec_annotation_compliance`.
- `dimos/agents/annotation.py`: decorador `@skill`.

## Flujo de datos

```text
Codigo blueprint
  -> BlueprintAtom detecta annotations
  -> ModuleCoordinator despliega modulos
  -> _connect_streams asigna transports
  -> _connect_module_refs inyecta proxies RPC segun Specs
  -> start_all_modules inicia procesos
```

## Inputs y outputs

- Entrada del blueprint: clases de modulos y configuracion.
- Salida del blueprint: sistema corriendo con procesos, topics y RPCs.
- Entrada de streams: mensajes tipados.
- Salida de specs: proxies RPC inyectados como atributos.

## Como se conecta con otras partes

El agente usa este mismo sistema. `McpServer` observa todos los modulos, recopila sus skills con `get_skills()` y las expone como tools. `McpClient` consulta esas tools y las transforma a tools LangChain.

## Que funciona hoy en G1 real

El mecanismo general de blueprints, streams, transports, specs y MCP existe y es reutilizado por todos los blueprints G1. El control directo G1 por WebRTC encaja bien en `G1ConnectionSpec`.

## Que parece incompleto o separado

La arquitectura permite integrar todo, pero la composicion G1 real no lo hace completamente. La incompatibilidad mas clara es entre `NavigationSkillContainer` y `unitree_g1_nav_onboard`.

## Notas para mi proyecto/paper

DimOS es interesante porque separa composicion declarativa, buses de datos y capacidades agenticas. Esa separacion ayuda a argumentar modularidad, reproducibilidad y extensibilidad en robotica generalista.
