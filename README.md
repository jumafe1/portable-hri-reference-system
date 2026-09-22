# Portable HRI Reference System

Integración y pruebas reproducibles de la arquitectura portable HRI sobre ROS 2.

## Desplazamiento relativo portable

`hri_capability_interfaces/MoveRelative` expresa un objetivo relativo en el
plano mediante `x_m`, `y_m` y `theta_rad`. El proveedor
`hri_naoqi_providers/NaoqiMoveRelative` desactiva primero la vida autónoma,
delega en `/nao/move_to` y contrasta el desplazamiento alcanzado con
`/nao/odom`. Los tres nombres internos pueden remapearse a las interfaces
equivalentes de Pepper sin cambiar `/hri/move_relative`.

La prueba local no requiere robot y usa servicios y odometría simulados:

```bash
colcon build \
  --packages-select hri_capability_interfaces hri_naoqi_providers \
  --symlink-install
source install/setup.bash
python3 tests/capabilities_naoqi_move_relative_probe.py
```

Comprueba catálogo y selección, odometría obligatoria, orden de desactivación
de vida autónoma, traducción y verificación del objetivo, límites, valores no
finitos, exclusión mutua, objetivo no alcanzado y liberación. El proveedor
acepta inicialmente hasta `±0.5 m` en X, `±0.3 m` en Y y `±π/2 rad`.

Esta implementación no convierte un *timeout* en cancelación. El servicio
SinfonIA disponible no expone `ALMotion.stopMove`; después de un *timeout* el
proveedor bloquea nuevas órdenes hasta ser liberado y reiniciado. Por esta
razón, la prueba mediante Capabilities2 con un robot físico permanece
deliberadamente pendiente hasta incorporar una vía de parada segura.

## Detección portable de personas

`hri_capability_interfaces/DetectPeople` abstrae la detección 2D de personas.
El proveedor `hri_yolo_providers/YoloDetectPeople` consume
`/yolo/detections`, filtra la clase `person` y expone
`/hri/detect_people`. La aplicación no depende de `yolo_msgs` ni del tópico de
cámara de un robot específico.

La prueba directa aceptada en NAO usó `yolov8n.pt` sobre CPU, recibió la cámara
frontal aproximadamente a 4 Hz y publicó detecciones aproximadamente a 3 Hz.
Se observaron personas con confianza 0,91–0,93 y un arreglo vacío al cubrir la
cámara. Esta evidencia valida cámara y motor YOLO; no sustituye la prueba
posterior a través de Capabilities2.

La prueba local del proveedor usa mensajes YOLO simulados y no requiere robot:

```bash
colcon build --packages-up-to hri_yolo_providers --symlink-install
source install/setup.bash
python3 tests/capabilities_yolo_detect_people_probe.py
```

Comprueba datos ausentes, selección del proveedor, filtrado por clase y
confianza, rechazo de valores inválidos, traducción del cuadro 2D, frame vacío,
datos vencidos y liberación.
`yolo_ros` es un proceso externo: Capabilities2 administra el adaptador, no la
inferencia ni el driver de cámara.

Con el driver del robot y `yolo_ros` publicando detecciones reales, ejecutar:

```bash
python3 tests/capabilities_yolo_detect_people_physical.py
```

La prueba espera una detección a través de `/hri/detect_people`, libera el
proveedor y pide confirmar que había una persona visible. No debe aprobarse
usando un publicador simulado: la evidencia física queda pendiente hasta
ejecutar este comando con la cámara del robot.

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
python3 -c "import yasmin, yasmin_ros; print('YASMIN disponible')"
```

La comprobación de importación debe ejecutarse en la misma terminal desde la
que se iniciará la aplicación. Si falla, el paquete pudo compilar porque
YASMIN es una dependencia de ejecución, pero la máquina de estados no podrá
arrancar hasta instalarla y volver a cargar los entornos.

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
