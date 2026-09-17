# Portable HRI Reference System

Integración y pruebas reproducibles de la arquitectura portable HRI sobre ROS 2.

## Prueba `Speak` con Capabilities2

La primera prueba vertical administra `hri_capability_interfaces/Speak` mediante
Capabilities2. El proveedor `hri_naoqi_providers/NaoSpeak` expone
`/hri/speak` y traduce la solicitud al servicio SinfonIA `/nao/say`.

La prueba local usa un backend `/nao/say` simulado; no requiere ni mueve un
robot. Comprueba selección, traducción segura (`animated=false`), propagación
del resultado, timeout con recuperación, backend ausente y liberación del
proveedor.

Desde un workspace con las dependencias compiladas y su `install/setup.bash`
cargado:

```bash
python3 tests/capabilities_nao_speak_probe.py
```

Para comprobar que el mismo `NaoSpeakRunner` puede reutilizarse con la interfaz
equivalente de Pepper mediante remapeo ROS 2:

```bash
python3 tests/capabilities_nao_speak_probe.py --backend-service /pepper/say
```

Esta es una prueba de reutilización del runner: el proveedor todavía se
identifica como `hri_naoqi_providers/NaoSpeak`. No sustituye la futura
especificación explícita `PepperSpeak`.

El manifiesto `portable_hri.repos` fija las dependencias externas conocidas.
Los repositorios propios permanecen en `main` durante el desarrollo y deberán
fijarse a etiquetas o commits antes del experimento final.

## Prueba física

La prueba con NAO requiere que el workspace del laboratorio esté cargado y que
el driver ya exponga `/nao/say`. El proveedor no arranca el driver, no envía
movimiento y siempre desactiva voz animada. Una frase ya aceptada por NAOqi no
puede cancelarse a través de este servicio.

Con el driver verificado y el overlay del proyecto cargado:

```bash
python3 tests/capabilities_nao_speak_physical.py
```

El script comprueba el resultado técnico, libera el proveedor y solicita la
confirmación auditiva del observador antes de aprobar la ejecución.

Con el driver de Pepper exponiendo `/pepper/say`, la prueba diagnóstica del
mismo runner se ejecuta así:

```bash
python3 tests/capabilities_nao_speak_physical.py \
  --backend-service /pepper/say \
  --text "Hola, esta es una prueba de Capabilities2 en Pepper."
```

El remapeo se aplica únicamente al proceso temporal de Capabilities2. La
aplicación conserva `/hri/speak`; el resultado seguirá reportando `NaoSpeak`
hasta implementar y registrar el proveedor definitivo `PepperSpeak`.
