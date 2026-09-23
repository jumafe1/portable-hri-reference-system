# Portable HRI Reference System

Integración y pruebas reproducibles de la arquitectura portable HRI sobre ROS 2.

## Desplazamiento relativo portable

`hri_capability_interfaces/MoveRelative` expresa un objetivo relativo en el
plano mediante `x_m`, `y_m` y `theta_rad`. El proveedor
`hri_naoqi_providers/NaoqiMoveRelative` delega en `/nao/move_to` y contrasta el
desplazamiento alcanzado con `/nao/odom`. El proveedor no lee ni modifica la
vida autónoma. Los dos nombres internos pueden remapearse a las interfaces
equivalentes de Pepper sin cambiar `/hri/move_relative`.

La prueba local no requiere robot y usa servicios y odometría simulados:

```bash
colcon build \
  --packages-select hri_capability_interfaces hri_naoqi_providers \
  --symlink-install
source install/setup.bash
python3 tests/capabilities_naoqi_move_relative_probe.py
```

Comprueba catálogo y selección, odometría obligatoria, traducción y
verificación del objetivo, límites, valores no finitos, exclusión mutua,
objetivo no alcanzado y liberación. El proveedor acepta inicialmente hasta
`±0.5 m` en X, `±0.3 m` en Y y `±π/2 rad`.

Esta implementación no convierte un *timeout* en cancelación. El servicio
SinfonIA disponible no expone `ALMotion.stopMove`; después de un *timeout* el
proveedor bloquea nuevas órdenes hasta ser liberado y reiniciado. Por esta
razón, cualquier prueba física debe limitarse a una orden pequeña, despejada y
supervisada; no debe interpretarse el *timeout* como una orden de parada.

Para una validación física supervisada, con el driver del robot ya iniciado,
usar `--x` y `--y` en metros y `--theta` en radianes. Por ejemplo:

```bash
python3 tests/capabilities_naoqi_move_relative_physical.py \
  --robot pepper --x 0.1 --y 0.0 --theta 0.0

python3 tests/capabilities_naoqi_move_relative_physical.py \
  --robot nao --x 0.1 --y 0.0 --theta 0.0
```

El script es el cliente de prueba del proveedor: registra y solicita
`MoveRelative`, comprueba `move_to`, odometría y activación, y exige escribir
`MOVER` antes de actuar. Después verifica el desplazamiento mediante
odometría, libera la capacidad y pide confirmación visual. No modifica la vida
autónoma ni usa `cmd_vel`. Debido a la ausencia de cancelación remota, debe
ejecutarse con espacio libre, supervisión directa y acceso al botón físico.

### Estado YASMIN `MoveRelative`

`move_demo` usa el mismo contrato y proveedor desde YASMIN. Recibe `x` y `y`
en metros y `theta` en radianes; solicita la capacidad mediante Capabilities2,
envía una sola orden efectiva a `move_to`, conserva el resultado estimado por
odometría y libera la capacidad. Puede reintentar durante tres segundos solo si
el proveedor responde `odometry_unavailable_before_motion`, rechazo que ocurre
antes de llamar a `move_to`. La espera del resultado es de 35 segundos porque
el proveedor admite hasta 30 segundos para la operación.

La prueba sin robot verifica NAO y Pepper simulados, objetivo no alcanzado,
odometría ausente, validación de argumentos y liberación:

```bash
colcon build --packages-up-to hri_reference_app --symlink-install
source install/setup.bash
python3 tests/yasmin_move_demo_probe.py
```

Con el driver del robot activo y `/nao/odom` o `/pepper/odom` publicando, la
primera prueba física supervisada puede solicitar 10 cm hacia adelante. El
`launch` exige valores explícitos y `confirm:=MOVER` antes de iniciar nodos:

```bash
ros2 launch hri_reference_app move_demo.launch.py robot:=nao x:=0.1 y:=0.0 theta:=0.0 confirm:=MOVER
# Para Pepper: sustituir robot:=nao por robot:=pepper.
```

Un resultado aprobado debe contener `application_succeeded`,
`message: "motion_completed"`, los campos `achieved` y `released: true`.
Esta aplicación no modifica la vida autónoma. La liberación de Capabilities2
no detiene un movimiento NAOqi en curso; el operador debe mantener la
supervisión y acceso al botón físico durante la prueba.

## Detección portable de personas

`hri_capability_interfaces/DetectPeople` abstrae la detección 2D de personas.
El proveedor `hri_yolo_providers/YoloDetectPeople` consume
`/yolo/detections`, filtra la clase `person` y publica el flujo portable
`/hri/people`; también conserva `/hri/detect_people` para consultas puntuales.
La aplicación no depende de `yolo_msgs` ni del tópico de cámara de un robot
específico. La imagen anotada `/yolo/dbg_image` se usa solamente para observar
la inferencia, no como contrato de la arquitectura.

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

Comprueba datos ausentes, selección del proveedor, disponibilidad del tópico,
filtrado por clase y confianza, rechazo de valores inválidos, traducción del
cuadro 2D, flujo continuo, frame vacío, datos vencidos y liberación.
`yolo_ros` es un proceso externo: Capabilities2 administra el adaptador, no la
inferencia ni el driver de cámara.

### Estado YASMIN `DetectPeople`

`people_demo` adquiere `DetectPeople` mediante Capabilities2, observa el tópico
portable `/hri/people` hasta encontrar una persona o agotar el tiempo y libera
la capacidad en ambos casos. El campo `robot` documenta la plataforma de la
prueba; la cámara que procesa YOLO se selecciona al iniciar `yolo_ros`.

La prueba sin robot publica detecciones YOLO simuladas y comprueba los casos de
persona presente y frames vacíos, además de la liberación del proveedor:

```bash
colcon build --packages-up-to hri_reference_app --symlink-install
source install/setup.bash
python3 tests/yasmin_people_demo_probe.py
```

Para probarlo físicamente, iniciar en terminales separadas el driver del robot
y `yolo_ros` con `input_image_topic:=/nao/camera/front/image_raw` o
`input_image_topic:=/pepper/camera/front/image_raw`. Después ejecutar:

```bash
ros2 launch hri_reference_app people_demo.launch.py robot:=nao timeout:=20.0
# Para Pepper: sustituir robot:=nao por robot:=pepper.
```

Si aparece una persona, el JSON final debe indicar `application_succeeded`,
una lista `people` no vacía y `released: true`. Sin personas debe indicar
`application_no_person` y `released: true`. La visualización anotada permanece
disponible en `/yolo/dbg_image`, pero no forma parte del contrato portable.
El autor completó este recorrido físico con NAO y Pepper. La salida conservada
de NAO reportó `application_succeeded`, una persona y `released: true`; la
ejecución de Pepper fue confirmada por el autor sin compartir su JSON.

Con el driver del robot y `yolo_ros` publicando detecciones reales, ejecutar:

```bash
python3 tests/capabilities_yolo_detect_people_physical.py
```

La prueba espera una detección a través de `/hri/detect_people`, libera el
proveedor y pide confirmar que había una persona visible. No debe aprobarse
usando un publicador simulado. Este probe puntual sigue sin una ejecución
física conservada; la prueba física aprobada corresponde al flujo continuo
`people_demo` descrito arriba.

## Aplicación portable con YASMIN

`hri_app` es una única entrada Python/YASMIN con tres modos independientes:
`speak`, `detect_people` y `move_relative`. Selecciona en el mismo proceso la
máquina de estados del modo solicitado; cada una establece el vínculo con
Capabilities2, adquiere su contrato, lo usa y libera la capacidad. El servidor
registra los tres proveedores, pero una ejecución activa solo el necesario.
Los ejecutables antiguos `speak_demo`, `people_demo` y `move_demo` se conservan
como comandos de regresión; no son tres aplicaciones finales distintas.

La nueva entrada no encadena aún detección, voz y movimiento. Su portabilidad
se apoya en los contratos `/hri/speak`, `/hri/people` y `/hri/move_relative`,
sin llamar desde YASMIN a servicios específicos de NAO o Pepper. El `launch`
remapea esos servicios para el robot elegido. `detect_people` termina al hallar
una persona o vencer el plazo; la imagen continua con rectángulos sigue
disponible aparte en `/yolo/dbg_image` cuando YOLO está activo.

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

Con el driver del robot ya ejecutándose en otra terminal, estos comandos usan
la **misma aplicación**. Sustituir `robot:=nao` por `robot:=pepper` para probar
la otra plataforma. El modo de percepción requiere además `yolo_ros` ya
recibiendo la cámara del robot y publicando `/yolo/detections`:

```bash
ros2 launch hri_reference_app hri_app.launch.py robot:=nao mode:=speak text:="Hola desde la aplicación HRI."
ros2 launch hri_reference_app hri_app.launch.py robot:=nao mode:=detect_people timeout:=20.0
```

Solo con espacio libre, supervisión física y el estado mecánico del robot
comprobado, ejecutar el modo de movimiento; `confirm:=MOVER` es obligatorio y
el objetivo se limita a `|x|≤0.5 m`, `|y|≤0.3 m`, `|θ|≤π/2 rad`:

```bash
ros2 launch hri_reference_app hri_app.launch.py robot:=nao mode:=move_relative x:=0.1 y:=0.0 theta:=0.0 confirm:=MOVER
```

El JSON final debe indicar `application_succeeded` y `released: true`. En
`detect_people`, `application_no_person` indica un plazo vencido sin detección,
no un fallo del proveedor. El modo de movimiento debe además reportar
`motion_completed`; la odometría es estimada, no una medición externa exacta.
El `launch` no inicia ni el driver ni YOLO. El proveedor `NaoSpeak` y el de
movimiento se reutilizan entre ambos robots mediante remapeos; no se crean
proveedores Pepper independientes si la interfaz ya es compatible.

La regresión sin hardware se ejecuta desde la raíz de este repositorio, con
los overlays ROS 2 y del proyecto cargados:

```bash
ROS_DOMAIN_ID=171 python3 tests/unified_hri_app_probe.py
```

Esta prueba comprueba las seis combinaciones de robot y modo con servicios y
mensajes simulados; **no acredita** que la nueva entrada haya corrido aún en
los robots físicos. No ejecutarla a la vez que el sistema real en el mismo
dominio ROS.

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
