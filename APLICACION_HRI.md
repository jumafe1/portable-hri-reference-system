# Aplicación HRI integrada

Esta es la primera composición de las capacidades ya probadas: detectar personas,
seguir a una persona, hablar y transcribir. El punto de entrada es
`aplicacion_hri.launch.py`; ROS 2 launch inicia los procesos y YASMIN organiza el
comportamiento. Capabilities2 conserva la adquisición, identidad y liberación de
los proveedores. No hace falta reunir los módulos en un único script grande.

## Probar en NAO

Dejar abierto el driver de NAO que ya se usa. Cerrar los launches anteriores de
HRI y las instancias manuales de YOLO y Whisper antes de usar este arranque.
No cambia IP, red, postura, vida autónoma ni configuración del driver.

En esta rama, con el driver abierto y el workspace cargado, el comando es:

```bash
source ~/Documents/portable_hri_ws/install/setup.bash
ros2 launch hri_reference_app aplicacion_hri.launch.py
```

Este lanzamiento **habilita micrófono y movimiento**, con los consentimientos ya
configurados para la demostración solicitada. Inicia en pausa: espera «Nao, sígueme».
Para comprobar percepción y voz sin habilitar la base:

```bash
ros2 launch hri_reference_app aplicacion_hri.launch.py enable_motion:=false
```

El perfil predeterminado conserva imagen 320×240, `CameraTop_optical_frame`,
`target_width_ratio=0.33` y velocidad máxima de avance `0.1 m/s`. La cercanía se
estima con el ancho de la caja respecto al ancho de la imagen: NAO avanza mientras
ese ancho sea menor que 0,30 y deja de avanzar a partir de ahí. No es una distancia
en metros y depende de la cámara, por eso se fija en el perfil del robot. Con
`target_width_ratio:=0` se vuelve al criterio anterior por altura
(`target_height_ratio=0.75`). La velocidad solicitada sigue siendo proporcional al
error visual. La cabeza no se orienta.

Con imagen de 320 píxeles de ancho, el seguimiento avanza aunque la persona esté
descentrada. Dentro de ±24 píxeles conserva la velocidad calculada por tamaño
aparente; entre 24 y 60 la reduce progresivamente hasta un tercio mientras gira.
Desde ±60 píxeles sigue avanzando a ese tercio mientras gira para centrar, en vez
de detenerse a girar en el sitio. Estos márgenes escalan con el ancho de imagen. La cercanía sigue impidiendo avanzar, y las
paradas por imagen antigua, persona ausente o varias personas siguen activas.

Al arrancar se cargan YOLO y el modelo pequeño español de Vosk. Whisper no se inicia.
YASMIN configura `DetectPeople`, `TranscribeSpeech`, `Speak` y, si está habilitada,
`MoveVelocity`; Capabilities2 conserva el vínculo y la propiedad de los proveedores.
El saludo inicial sigue desactivado.

| Orden completa | Efecto |
|---|---|
| «Nao, detente», «Nao, para», «Nao, quédate quieto» | Pausa persistente y respuesta del grupo «Aquí me quedo». |
| «Nao, no me sigas», «Nao, no vengas conmigo», «Nao, no te muevas», «Nao, no sigas» | También pausa; no se busca la palabra «sigas» como subcadena. |
| «Nao, sígueme», «Nao, ven conmigo», «Nao, vuelve a seguirme» | Respuesta del grupo «Te voy a seguir» y reanudación condicionada a percepción válida. |

Se exige la palabra de activación y una frase final completa. Los resultados parciales,
antiguos, repetidos o fuera de la lista no ejecutan acciones. El reconocedor puede
confundir sonidos; es un control de interacción, no un paro de emergencia.
Las respuestas se eligen al azar, se envían por `Speak` después del acuse de parada,
y se filtran resultados captados durante la respuesta y un segundo posterior.
El seguimiento continúa procesando imágenes y enviando ceros mientras se habla.

La pausa por voz no se levanta al volver a entrar en la imagen: requiere otra orden
para seguir. Una vez autorizado seguir, se conservan las pausas y recuperaciones
por persona ausente, varias personas o imagen antigua. Reanudar espera tres imágenes
nuevas válidas. Si falla el reconocedor o la respuesta hablada, se mantiene la pausa.

Cambiar vocabulario, respuestas y pausa inicial en
[`hri_reference_app/config/voice_commands.yaml`](hri_reference_app/config/voice_commands.yaml).
Cambiar robot, proveedores, topics y modelos en
[`hri_reference_app/config/nao.yaml`](hri_reference_app/config/nao.yaml).
Los límites visuales y de velocidad admiten los mismos argumentos de lanzamiento.

Para conservar las alternativas anteriores:

```bash
# Seguimiento sin escucha ni control por voz (inicia el seguimiento visual).
ros2 launch hri_reference_app aplicacion_hri.launch.py transcribe_speech:=false

# Transcripción libre con Whisper, sin órdenes de movimiento por voz.
ros2 launch hri_reference_app aplicacion_hri.launch.py \
  speech_backend:=whisper voice_commands:=false
```

En una instalación nueva, preparar Vosk una sola vez (descarga oficial de ~39 MB):

```bash
cd ~/Documents/portable_hri_ws
bash src/portable-hri-reference-system/scripts/setup_vosk.sh
colcon build --symlink-install --packages-select \
  hri_speech_providers hri_vosk_providers hri_whisper_providers hri_reference_app
source install/setup.bash
```

El script utiliza `.venvs/vosk` y `models/vosk-model-small-es-0.42` dentro del workspace;
no instala Torch ni cambia el entorno de Whisper. El lanzamiento posterior es local,
sin descarga automática. El modelo y el entorno no se incluyen en Git.

La terminal muestra avisos y errores de HRI, Vosk/Whisper y YOLO, además de las líneas de
arranque y cierre de launch. Los diagnósticos repetitivos de seguimiento y voz
quedan ocultos. Para depurar, añadir `log_level:=info`; para imprimir también
transcripciones, añadir `print_transcript:=true` junto con ese nivel.

El arranque integrado desactiva el backend opcional NNPACK de PyTorch en YOLO.
Así evita los avisos nativos repetidos `Could not initialize NNPACK! ... Unsupported
hardware` en esta CPU, que no dependen del nivel del logger ROS. La inferencia
continúa usando los otros kernels de CPU de PyTorch.

La carga del modelo puede tardar. Con `log_level:=info` se ven `model_ready` y
`capture_started`. El saludo termina antes de solicitar captura, para no transcribir
la propia voz de NAO. Un saludo fallido impide pasar a seguimiento/escucha y libera
su proveedor. `greet:=true` permite habilitarlo cuando se quiera probar la voz.

Al desaparecer la persona, haber varias o faltar imágenes recientes, la base se
detiene y espera, conservando la recuperación que ya se probó. Los fallos críticos
de proveedores, vínculo, geometría o comandos siguen cerrando la aplicación.
`Ctrl+C` solicita parada, libera capacidades y termina los procesos iniciados por
este launch. El driver externo continúa abierto. La salida actual del seguimiento
puede indicar `cancelled_or_ros_shutdown` y código 1 al cancelar; no implica por sí
sola un fallo de parada. Revisar `stop_acknowledged` y los campos de liberación.

La prueba conjunta debe comprobar texto reconocido, seguimiento,
pausa/reanudación al salir de imagen o entrar otra persona, y cierre con Ctrl+C.
Esta revisión tiene pruebas de software y compilación; la ejecución física conjunta
queda pendiente de esa prueba en NAO.

## Opciones

```bash
ros2 launch hri_reference_app aplicacion_hri.launch.py --show-args
ros2 topic echo /hri/speech/transcript --field text
```

| Opción | Uso |
|---|---|
| `enable_motion:=true confirm:=MOVER` | Habilitar seguimiento físico; en esta rama está habilitado, con pausa inicial por voz. |
| `confirm_listen:=ESCUCHAR` | Ya configurado en el arranque integrado de esta rama. |
| `transcribe_speech:=false` | Ejecutar seguimiento visual sin escucha ni control por voz. |
| `speech_backend:=whisper voice_commands:=false` | Recuperar la transcripción libre con Whisper. |
| `voice_commands_config:=/ruta/comandos.yaml` | Vocabulario, respuestas y pausa inicial. |
| `vosk_model_path:=/ruta/modelo vosk_python:=/ruta/python` | Modelo local y entorno Python de Vosk. |
| `greet:=true` | Habilitar el saludo; por defecto está desactivado. |
| `log_level:=warn` | Predeterminado: ocultar mensajes informativos de seguimiento, Whisper, YOLO y Capabilities2. |
| `log_level:=info print_transcript:=true` | Recuperar los diagnósticos detallados y el texto reconocido en terminal. |
| `record_diagnostics:=true` | Guardar tiempos, decisiones, cola de voz y CPU en archivos, sin aumentar la salida de terminal. |
| `diagnostics_dir:=/ruta/ensayo` | Directorio del registro opcional; por defecto se crea dentro del directorio de logs de esta ejecución. |
| `start_yolo:=false` | Reutilizar el detector externo, en el topic del perfil. |
| `start_whisper_worker:=false` | Reutilizar un worker externo compatible. |
| `yolo_debug:=true` | Iniciar también la imagen de depuración de YOLO. |
| `yolo_model:=/ruta/modelo.pt` | Pesos locales de YOLO. |
| `whisper_model_path:=/ruta/modelo.pt` | Pesos locales de Whisper. |
| `target_width_ratio:=0.33 max_forward_speed:=0.1` | Sobrescribir esos valores del perfil. Un ancho mayor acerca más el robot; `0` usa `target_height_ratio`. |

Los workers externos no son detenidos por este launch. Usar una sola instancia HRI
en el mismo dominio ROS: los contratos `/hri/*` y el servidor no están aislados por
robot. El texto impreso puede permanecer en los logs de ROS launch.

Los modelos no se descargan automáticamente. En esta instalación se usan
`yolov8n.pt` en la raíz del workspace y `~/.cache/whisper/base.pt`. El primer camino
se resuelve desde la instalación de colcon, independientemente del directorio
actual; en otro despliegue se puede dar una ruta absoluta.

## Revisar pausas y rendimiento sin llenar la terminal

El nivel `warn` descarta los mensajes informativos también del archivo ROS del
nodo. Por eso una ejecución anterior con ese nivel puede tener el log de
`hri_app` vacío: no permite reconstruir retrospectivamente cada pausa.

Para registrar el siguiente ensayo:

```bash
ros2 launch hri_reference_app aplicacion_hri.launch.py \
  record_diagnostics:=true
```

Al arrancar se imprime una única ruta `Diagnóstico HRI en archivo: .../summary.json`.
Ese archivo se actualiza cada segundo y contiene frecuencias de detección,
antigüedad de imágenes, motivos de espera (incluidos los de voz), cola de voz, uso de CPU y memoria.
`events.jsonl` en el mismo directorio permite relacionar las decisiones del
seguimiento con la carga y los estados de voz. Los archivos de eventos rotan:
como máximo se conservan tres de 10 MiB por ejecución. El resumen conserva los
contadores de todo el ensayo; las métricas recientes de flujo cubren 30 segundos.

Es un proceso observador independiente: se suscribe a los mensajes de diagnóstico
y detección ya publicados, y lee `/proc` para medir los procesos de este launch.
No consume imágenes ni audio ni escribe transcripciones. El muestreo de CPU es
cada segundo; el detalle de seguimiento se guarda a 5 Hz y en cada cambio de
motivo. También añade una pequeña carga de observación. Los mensajes son
best-effort: una pérdida del observador no demuestra una pérdida del productor.
`unknown_follow_time_s` separa interrupciones de observación de los tiempos
atribuidos a decisiones del controlador.

Para comparar el seguimiento con y sin voz, hacer dos ensayos separados de
60–90 s en condiciones parecidas. Detener el primero con Ctrl+C y repetir el
mismo comando añadiendo `transcribe_speech:=false`. Cada ejecución genera su
propio directorio. Comparar `streams.yolo`, `streams.people`, los tiempos en
`waiting_fresh_images`/`reacquiring_person`, y los eventos de CPU durante
`speech.state` (Vosk procesa el flujo continuamente). En `cpu_percent_one_core`, 100 % corresponde a un núcleo
lógico, por lo que un proceso con varios hilos puede superar 100 %.

Este registro ayuda a localizar las pausas; no cambia los límites del controlador
ni demuestra por sí solo una parada física. Si se necesita distinguir cámara de
YOLO, sigue disponible el observador de timestamps `tests/follow_stream_diagnostic.py`.

## Módulos y portabilidad

| Responsabilidad | Archivo o contrato |
|---|---|
| Arranque de procesos y prevalidación | `hri_reference_app/launch/aplicacion_hri.launch.py` |
| Cámara, audio, modelos y proveedores NAO | `hri_reference_app/config/nao.yaml` |
| Máquina de estados YASMIN | `hri_reference_app/scripts/follow_mode.py` |
| Saludo mediante `Speak` | `hri_reference_app/scripts/startup_speak.py` |
| Seguimiento visual y recuperación | `follow_session.py`, `follow_controller.py` |
| Transporte portable de movimiento | `follow_motion.py` → `MoveVelocity` |
| Escucha continua | `follow_speech.py` → `TranscribeSpeech` |
| Adaptación al robot / motor perceptivo | Paquetes `hri_*_providers`, gestionados por Capabilities2 |

`FollowPerson` es un comportamiento que combina `DetectPeople` y `MoveVelocity`.
`Speak` y `TranscribeSpeech` son capacidades independientes que esta composición
añade. El saludo es un texto fijo: aún no hay diálogo, modelo de lenguaje ni
interpretación de órdenes de movimiento a partir de la transcripción.

Para otro robot, copiar el perfil, cambiar `robot`, los topics y los identificadores
de proveedor, y pasar `robot:=mi_robot profile:=/ruta/mi_robot.yaml`. Los proveedores
se seleccionan desde el perfil; el controlador visual no necesita nombres de NAO.
La opción `speech_backend` selecciona explícitamente Vosk o Whisper en esta entrada.
Un robot de ruedas necesita un proveedor `MoveVelocity` que cumpla el contrato,
incluyendo confirmación de parada, límites y watchdog; un adaptador de voz implementa
`Speak`. Cambiar el nombre del robot no proporciona esos adaptadores.

Los workers Vosk y Whisper actuales aceptan `naoqi_pcm_s16le`, mensajes estampados de audio
mono a 16 kHz más su información de formato. Para reutilizarlos hay que adaptar el
audio del otro robot a ese formato o implementar otro proveedor `TranscribeSpeech`.
`backend_remappings` del perfil permite remapear nombres internos de proveedores
sin cambiar los contratos públicos. Un detector diferente requiere su proveedor
y `start_yolo:=false`; el perfil no ejecuta comandos arbitrarios de backends nuevos.
La portabilidad a esos robots requiere pruebas propias; esta composición está
preparada para el perfil NAO.

## Reproducir los ajustes locales de YOLO

`portable_hri.repos` fija el repositorio original `mgonzs13/yolo_ros` en el commit
`c6d29ce65da555b125517db6ac53729320422b2b`. El parche incluido
[`patches/yolo_ros-quiet-cpu.patch`](patches/yolo_ros-quiet-cpu.patch) añade
`log_level` y `enable_nnpack`, que utiliza este arranque integrado. Así se
conservan los ajustes de esta instalación sin publicar en el repositorio del autor.

Solo en una copia nueva de ese commit, desde la raíz del workspace:

```bash
git -C src/yolo_ros apply --check ../portable-hri-reference-system/patches/yolo_ros-quiet-cpu.patch
git -C src/yolo_ros apply ../portable-hri-reference-system/patches/yolo_ros-quiet-cpu.patch
```

La copia del laboratorio ya tiene esos cambios: no aplicar el parche dos veces.
Después de compilar las dependencias y `yolo_ros` con su entorno Python, instalar
el launch de YOLO y la aplicación:

```bash
cd ~/Documents/portable_hri_ws
colcon build --packages-select yolo_bringup hri_reference_app --symlink-install
source install/setup.bash
```

El launch anterior `hri_app.launch.py` continúa disponible para ensayos individuales.

## Comprobación de software de órdenes de voz

`tests/vosk_voice_probe.py` usa el modelo real de Vosk, audio español sintético en
memoria, Capabilities2 y servicios simulados. Requiere un dominio DDS vacío y local;
no inicia el driver ni usa micrófono o altavoces. Sus resultados no sustituyen la
prueba acústica con el NAO.

El puente compartido `hri_speech_providers` gestiona solicitudes, identificadores y
arrendamientos. `hri_vosk_providers` y `hri_whisper_providers` seleccionan su motor;
ningún reconocedor decide movimiento. `voice_command_policy.py` interpreta órdenes
y `voice_commands.py` coordina la pausa y las respuestas dentro de FollowPerson.
