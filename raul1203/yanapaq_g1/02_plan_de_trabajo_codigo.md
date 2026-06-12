# Plan de trabajo de codigo

Ultima modificacion: 2026-06-12 16:23:07 -05 -0500

## Recomendacion de entorno

Trabajar en el repo actual:

```text
/home/raul/dimos
```

Razon: ya contiene DimOS, los blueprints G1, dependencias locales, assets y
notas. Re-clonar desde cero ahora agregaria friccion y duplicaria estado.

Crear otra copia solo si:

- se necesita probar instalacion limpia;
- se quiere comparar contra `origin/main` sin cambios locales;
- se van a hacer pruebas destructivas;
- la rama actual queda muy contaminada.

## Rama recomendada

```bash
git checkout main
git pull origin main
git checkout -b feat/yanapaq-g1-mvp
```

Si hay trabajo local que no se debe perder, crear backup antes:

```bash
git branch backup/antes-yanapaq-$(date +%Y%m%d-%H%M%S)
```

## Filosofia de cambios

1. Aditivo primero.
2. Integrar por blueprints nuevos.
3. Modificar DimOS core solo con evidencia.
4. Mantener blueprints originales funcionando.
5. Probar cada capa antes de agregar la siguiente.

## Estructura esperada de codigo

```text
dimos/robot/unitree/g1/blueprints/yanapaq/
  unitree_g1_yanapaq_dev.py

dimos/robot/unitree/g1/yanapaq/
  safety_supervisor.py
  mission_executive.py
  semantic_memory.py
  object_lifecycle.py
  active_verifier.py
  openai_vlm_verifier.py
  specs.py
  tests/
```

Si algun modulo resulta portable a otros humanoides, moverlo despues a:

```text
dimos/yanapaq/
```

No empezar ahi. Primero resolver G1 real.

## MVP tecnico

MVP 0:

- blueprint `unitree-g1-yanapaq-dev` arranca;
- conserva navegacion G1 existente;
- no contiene LLM.

MVP 1:

- `SafetySupervisor` filtra comandos;
- stop confiable;
- no hay camino LLM -> velocidad directa.

MVP 2:

- `MissionExecutive` ejecuta misiones estructuradas;
- se puede cancelar;
- deja logs.

MVP 3:

- memoria SQLite recuerda objeto/habitacion/pose;
- se puede consultar entre sesiones.

MVP 4:

- YOLOE detecta candidatos;
- OpenAI VLM verifica solo ambiguedad;
- memoria se actualiza con evidencia.

## Criterio de avance

No pasar al siguiente MVP si el anterior no:

- arranca;
- se detiene limpiamente;
- tiene prueba minima;
- tiene logs entendibles;
- no rompe blueprints originales.
