# System prompt G1

Ultima modificacion: 2026-06-11 11:14:18 -05 -0500

## Objetivo del archivo

Explicar el prompt especifico del G1 y por que importa para evitar habilidades hallucinated.

## Explicacion conceptual desde cero

Un system prompt es la instruccion fija que guia al LLM. En DimOS hay un prompt generico Go2 y un prompt G1. El G1 debe usar su propio prompt porque sus skills y riesgos son distintos.

## Archivos, clases y funciones relevantes

- `dimos/robot/unitree/g1/system_prompt.py`: `G1_SYSTEM_PROMPT`
- `dimos/robot/unitree/g1/blueprints/agentic/_agentic_skills.py`
- `dimos/agents/mcp/mcp_client.py`: `McpClientConfig.system_prompt`

`_agentic_skills` pasa:

```python
McpClient.blueprint(system_prompt=G1_SYSTEM_PROMPT)
```

## Flujo de datos

```text
G1_SYSTEM_PROMPT
  -> McpClientConfig
  -> create_agent(system_prompt=...)
  -> LLM decide acciones segun skills disponibles
```

## Inputs y outputs

Input:

- Lista textual de capacidades: movimiento, gestos, modos, navegacion, comunicacion.

Output:

- Comportamiento esperado del agente: hablar con `speak`, priorizar seguridad, usar tools correctas.

## Como se conecta con otras partes

El prompt recomienda `speak` porque la persona no ve el texto del agente. Tambien instruye usar `navigate_with_text` para navegacion, aunque esa skill requiere integracion adicional.

## Que funciona hoy en G1 real

El prompt G1 existe y esta correctamente seleccionado en `_agentic_skills`. Describe:

- Identidad: Daneel.
- Seguridad critica.
- Comunicacion por voz.
- Movimiento directo.
- Gestos de brazos.
- Modos de movimiento.
- Navegacion y tagged locations.

## Que parece incompleto o separado

El prompt promete navegacion como si estuviera disponible, pero en el blueprint agentic/full falta el proveedor de `NavigationInterfaceSpec`. Esto puede inducir al LLM a llamar una tool que no puede funcionar si el sistema no construye o no esta cableado.

## Notas para mi proyecto/paper

El prompt deberia ser coherente con el grafo real de capacidades. Una contribucion practica es generar prompts desde el grafo MCP real y anotar tools no disponibles o no seguras.
