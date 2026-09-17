# Portable HRI Reference System

Integración y pruebas reproducibles de la arquitectura portable HRI sobre ROS 2.

## Aplicación portable con YASMIN

`hri_reference_app` contiene la primera aplicación de referencia en Python. Su
máquina de estados YASMIN establece un vínculo con Capabilities2, solicita el
contrato `hri_capability_interfaces/Speak`, ejecuta `/hri/speak` y libera la
capacidad tanto si la voz termina correctamente como si el servicio del robot
falla.

La aplicación no llama directamente a `/nao/say` ni a `/pepper/say`. El mismo
flujo se ejecuta en ambas plataformas y el `launch` limita la diferencia al
servicio ROS 2 que usa el proveedor C++ existente:

```text
YASMIN -> Speak -> Capabilities2 -> NaoSpeakRunner -> /nao/say
                                            \-----> /pepper/say (remapeado)
```

Instalar dependencias, compilar y cargar el overlay:

```bash
sudo apt install ros-jazzy-yasmin ros-jazzy-yasmin-ros
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to hri_reference_app --symlink-install
source install/setup.bash
```

Con el driver correspondiente ejecutándose en otra terminal y exponiendo
`/nao/say` o `/pepper/say`, usar uno de estos comandos:

```bash
ros2 launch hri_reference_app speak_demo.launch.py \
  robot:=nao \
  text:="Hola, esta es una aplicación portable en NAO."

ros2 launch hri_reference_app speak_demo.launch.py \
  robot:=pepper \
  text:="Hola, esta es una aplicación portable en Pepper."
```

El resultado final se imprime como JSON. `outcome` debe ser
`application_succeeded` y `released` debe ser `true`. La ruta Pepper todavía
reutiliza `hri_naoqi_providers/NaoSpeak` mediante remapeo; no representa un
proveedor Pepper definitivo.

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
