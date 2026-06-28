# Workspace Raul1203

Ultima modificacion: 2026-06-12 16:23:07 -05 -0500

## Estado actual

La carpeta activa para el proyecto es:

```text
raul1203/yanapaq_g1/
```

Todo lo demas que quedo como historial fue movido a:

```text
raul1203/99_archivo_no_leer/
```

Ese archivo historico conserva analisis previos, pruebas y notas antiguas, pero
no debe cargarse como contexto inicial de un agente. Solo se consulta si un
documento activo lo pide explicitamente.

## Que debe leer un agente nuevo

Orden recomendado:

1. `yanapaq_g1/README.md`
2. `yanapaq_g1/00_contexto_minimo_agente.md`
3. `yanapaq_g1/01_prompts_para_vibecodear.md`
4. `yanapaq_g1/paper/especificacion_tecnica_software_y_despliegue.md`

Si el trabajo es de paper, leer tambien:

```text
yanapaq_g1/paper/metodologia_resultados_y_discusion.md
yanapaq_g1/paper/revision_literatura_g1_2025_2026.md
yanapaq_g1/paper/arquitectura_memoria_semantica_activa.md
```

## Politica de contexto

- No leer `99_archivo_no_leer/` por defecto.
- No cargar todas las figuras salvo que se pidan.
- No guardar API keys en Markdown.
- Trabajar el codigo dentro del repo DimOS, en una rama nueva.
