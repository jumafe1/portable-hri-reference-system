# Orientación de cámara y transcripción: guía de pruebas

Incremento software del 21 de octubre de 2026. Añade `OrientCamera` y
`TranscribeSpeech` opcionales al seguimiento; la transcripción conserva también
su modo independiente para pruebas. **Este incremento está implementado y tiene
validación software, pero no se ha probado físicamente.** Las
pruebas anteriores de seguimiento no validan automáticamente estas capacidades.

La arquitectura mantiene contratos portables: `follow_person` combina
`DetectPeople`, `MoveVelocity` y, si se solicita y está disponible, `OrientCamera`.
La escucha explícita añade `TranscribeSpeech` bajo el mismo vínculo con Capabilities2.
El proveedor `NaoqiOrientCamera` traduce orientación a articulaciones NAOqi;
la aplicación no envía nombres de articulaciones. `WhisperTranscribeSpeech`
adapta un motor local externo, del mismo modo que `YoloDetectPeople` adapta YOLO.

## Preparación común

En el computador del laboratorio, una vez que los cambios estén disponibles
localmente, compilar también los nuevos contratos y proveedores:

```bash
cd ~/Documents/portable_hri_ws
source /opt/ros/jazzy/setup.bash
source ~/Documents/naoqi/naoqi_ws/install/setup.bash
colcon build --packages-up-to hri_reference_app hri_naoqi_providers hri_whisper_providers --symlink-install
source install/setup.bash
```

Cargar esos entornos en cada terminal. Mantener una sola instancia del driver,
de YOLO sobre su tópico y de `hri_app`/Capabilities2. Elegir una única ejecución:
seguimiento con capacidades opcionales, o prueba independiente de transcripción;
no iniciar dos servidores sobre `/hri/*`. El worker Whisper es un proceso separado
que el mismo launch inicia al habilitar voz con su proveedor. Capabilities2 sigue
gestionando la capacidad y autorizando cada captura; el worker no sustituye al servidor.

## Orientar la cámara dentro del seguimiento

Puede conservarse el driver y YOLO ya activos. Verificar que YOLO procesa la
cámara elegida y que dimensiones y `frame_id` coinciden con los argumentos.
Además, comprobar estos endpoints sin enviar órdenes:

```bash
ros2 topic echo /nao/camera/front/camera_info --once
ros2 topic echo /nao/joint_states --once
ros2 topic info /nao/joint_trajectory
```

Se requieren `CameraInfo` reciente y válido, con dimensiones exactas y el mismo
frame óptico que las detecciones, y feedback reciente de `HeadPitch`/`HeadYaw`.
La calibración admite `plumb_bob` de cinco coeficientes finitos, o modelo y
distorsión vacíos para pinhole. Se rechazan ROI/binning no soportados y modelos
de distorsión distintos; no inventar focales para superar esas verificaciones.

Con las lecturas **320 × 240 y `CameraTop_optical_frame` confirmadas**, el siguiente
comando habilita seguimiento continuo supervisado con orientación y desplazamiento.
Para la primera prueba, interrumpir manualmente con Ctrl+C tras comprobar la respuesta:

```bash
ros2 launch hri_reference_app hri_app.launch.py \
  robot:=nao mode:=follow_person \
  image_width:=320 image_height:=240 expected_frame:=CameraTop_optical_frame \
  orient_camera:=true target_height_ratio:=0.80 max_forward_speed:=0.03 \
  enable_motion:=true confirm:=MOVER
```

Usar suelo plano despejado, una sola persona y supervisión con acceso a parada
física independiente. No ejecutar otras órdenes de cabeza/base ni dejar otro
comportamiento que cambie la orientación. El proveedor no modifica rigidez,
vida autónoma ni yaw para imponer esas condiciones. Empezar visible y centrado,
sin forzar proximidad; comprobar la respuesta antes de ampliar duración o
velocidad. El límite configurable de avance sigue siendo 0.10 m/s, no una
velocidad medida ni garantizada para cualquier plataforma.

No hay vencimiento total: este modo rechaza `duration` y `timeout`. Se conservan
los plazos breves de salud/comunicación. La cabeza debe empezar casi frontal
(yaw medido ≤0.03 rad en magnitud) y dentro del rango de elevación admitido;
no se recentra automáticamente para superar esas condiciones.

La prueba combina ambos movimientos, pero no los ordena simultáneamente:

1. El seguimiento decide avance/giro a partir de una detección válida.
2. Si hace falta encuadrar verticalmente, envía cero a la base durante al menos
   0.4 s y exige una confirmación software reciente de estado detenido.
3. Solicita un ajuste de elevación de hasta 0.03 rad, dentro de ±0.25 rad, como
   máximo una vez por segundo. Elevación positiva significa mirar hacia arriba.
4. Mantiene la base en cero hasta recibir asentamiento medido durante al menos
   0.25 s e imagen capturada después; entonces puede continuar el seguimiento.

El tamaño de referencia con orientación activa se calcula mediante la apertura
angular vertical de la caja, usando calibración y corrección de distorsión. Se
expresa como una ocupación equivalente en una imagen pinhole centrada: `0.80`
**no es distancia en metros ni necesariamente 80 % de la caja cruda en cualquier
inclinación**. Evita confundir el cambio de proyección por pitch puro con cambio
de separación. Sigue siendo una aproximación: no compensa la traslación de la
cámara alrededor del cuello, la postura humana ni cambios del detector.

Mover la cabeza cambia el encuadre, no amplía el campo visual. Una caja recortada
mantiene la base en cero; un recorte vertical puede motivar un ajuste, pero si
ambos extremos están recortados no se asume que mover la cámara resolverá el
problema. No hay marcha atrás, estimación métrica ni evitación de obstáculos.

### Adaptación, diagnóstico y cierre

- `orient_camera:=false` conserva el seguimiento con cámara fija. En preview,
  incluso con `orient_camera:=true`, no se adquiere ni actúa la cámara.
- Ausencia confirmada de la interfaz o lista vacía de proveedores en el catálogo
  produce `fixed_camera_fallback_no_interface` o `fixed_camera_fallback_no_provider`:
  continúa sin cámara móvil, antes de intentar adquirirla.
- Un `camera_provider` explícito inexistente cuando hay otros registrados es un
  error de configuración; no se oculta como fallback. Tampoco se degrada tras
  fallos de adquisición o pérdida de feedback de una cámara ya activa.
- Puede cambiarse `camera_provider:=paquete/Proveedor` y
  `camera_info_topic:=/ruta/camera_info`. `robot:=pepper` remapea los endpoints
  NAOqi, pero no acredita una prueba física en Pepper. El launch actual admite
  NAO/Pepper, no cualquier robot automáticamente.
- El diagnóstico `/hri/follow_person/preview` añade `camera_status`,
  `camera_orientation_active` y `camera_elevation_rad`; el JSON final incluye
  `camera_released` y `camera_hold_acknowledged`.

`Ctrl+C` solicita cero en la base, hold de cabeza y liberación de capacidades.
Con cámara o transcripción habilitadas, el cierre reserva hasta 30 s para la
aplicación y 32 s para mantener el servidor disponible, con escalado a SIGTERM
del servidor a los 34 s. También aplica a preview con voz habilitada. No son
plazos de frenado físico. Hold de cámara conserva
la postura medida, no recentra. Con feedback inválido puede no confirmarse el
hold aunque se libere la capacidad. **Una confirmación software no demuestra
parada física; Ctrl+C no sustituye al mecanismo físico de parada.**

## Transcripción local bajo solicitud explícita

La transcripción produce texto, sin generar respuestas con un modelo de lenguaje
ni llamar a `Speak` automáticamente. Puede utilizarse como prueba independiente
de una ventana, o durante `follow_person` mediante escucha continua bajo
autorización explícita. En seguimiento, WebRTC VAD procesa frames de 20 ms:
abre un fragmento tras detectar voz y lo cierra después de 0.8 s de silencio.
Conserva hasta 200 ms anteriores al inicio de voz, siempre posteriores a la
solicitud autorizada. Se ignoran ráfagas con menos de 100 ms de voz; una frase
muy larga se divide al llegar a 10 s.

Whisper procesa un fragmento en otro hilo mientras el callback de audio sigue
capturando. El buffer actual, dos fragmentos pendientes y una inferencia activa
están acotados. Si se llena la cola, se descarta y vacía la pendiente más antigua;
`speech_queue_full` y `dropped_segments` hacen visible esa pérdida. No se puede
prometer conservar todo el habla si la CPU transcribe más lento de lo que llega.
El silencio solo no genera transcripciones vacías. El detector de voz puede
confundir ruido con habla; esto no garantiza exactitud del texto.

### Dependencias y preparación de Whisper

Para probar el worker se necesitan, en el intérprete usado por ROS:
`rclpy`, `audio_common_msgs`, NumPy,
PyTorch y la distribución `openai-whisper` (módulo Python `whisper`), además de un
archivo de pesos `.pt` local de procedencia confiable. El worker no descarga
pesos automáticamente ni usa una API remota.

La escucha continua añade `webrtcvad-wheels==2.0.14`
([documentación del detector](https://github.com/daanzu/py-webrtcvad-wheels)).
Se instala junto al ejecutable dentro del workspace, sin cambiar el Python del
sistema. Desde `~/Documents/portable_hri_ws`, después de compilar:

```bash
python3 -m pip install --no-deps --target install/hri_whisper_providers/lib/hri_whisper_providers \
  -r src/portable-hri-providers/hri_whisper_providers/requirements.txt
```

Ya se instaló en este workspace. Al recrear `install/` debe repetirse ese paso.
El modelo sigue siendo el `base.pt` existente; no se cambió de modelo.


El 4 de noviembre de 2026 se comprobaron esas dependencias en el computador
del laboratorio y se cargó `~/.cache/whisper/base.pt` en CPU. La preparación
incluye una inferencia con silencio sintético antes de anunciar `model_ready`;
no utiliza el micrófono. Esa comprobación no valida reconocimiento de voz real.

Comprobar `/nao/mic_info` antes de capturar. El adaptador inicial acepta
exclusivamente un canal, 16 kHz, `sample_format: S16LE` y
`coding_format: wave`. En el driver inspeccionado, los bytes son PCM crudo sin
cabecera WAV, pese a esa etiqueta. Otros formatos se rechazan, no se convierten.

El launch usa por defecto `~/.cache/whisper/base.pt`; puede cambiarse mediante
`whisper_model_path:=/ruta/absoluta/modelo.pt`. No hace falta iniciar el worker
en otra terminal. Al cerrar la aplicación también se cierra el worker del launch.

Para gestionar el worker manualmente, iniciar una sola instancia con:

```bash
ros2 run hri_whisper_providers whisper_worker --ros-args \
  -p model_path:=/ruta/absoluta/modelo-confiable.pt \
  -p input_format:=naoqi_pcm_s16le \
  -p audio_topic:=/nao/mic_stamped \
  -p audio_info_topic:=/nao/mic_info
```

En ese caso añadir `start_whisper_worker:=false` al launch de la aplicación.
Para Pepper, el worker iniciado automáticamente recibe `/pepper/mic_stamped`
y `/pepper/mic_info`; un worker manual requiere cambiar ambos parámetros.

### Prueba independiente de una ventana

Con el driver activo y cualquier `hri_app` anterior terminado:

```bash
ros2 launch hri_reference_app hri_app.launch.py \
  robot:=nao mode:=transcribe_speech confirm:=ESCUCHAR \
  record_duration:=3.0 timeout:=60.0 print_transcript:=true
```

Esperar el mensaje `capture_started: hable ahora` antes de hablar. La captura
comienza después de aceptar la solicitud, no graba audio anterior.
En este modo independiente, `record_duration` permite 0.5–10 s; `timeout` debe
ser al menos duración + 2 s y como máximo 120 s. El idioma predeterminado es
español: no es necesario escribir `speech_language:=es`. Ese argumento opcional
acepta `es`, `en` o vacío para detección automática. El servicio portable es
`/hri/speech/transcribe` y el resultado se
publica en `/hri/speech/transcript`, correlacionado mediante `request_id`.

### Transcribir durante FollowPerson

Mantener el driver y YOLO activos y ejecutar una sola aplicación HRI.
Cerrar antes cualquier worker manual, salvo que se indique
`start_whisper_worker:=false`. Primero probar **sin movimiento**:

```bash
ros2 launch hri_reference_app hri_app.launch.py \
  robot:=nao mode:=follow_person \
  image_width:=320 image_height:=240 expected_frame:=CameraTop_optical_frame \
  target_height_ratio:=0.75 max_forward_speed:=0.1 \
  orient_camera:=false enable_motion:=false \
  transcribe_speech:=true confirm_listen:=ESCUCHAR print_transcript:=true
```

El worker muestra `model_loading` y `model_ready`, luego `capture_started` y
`audio_received`. Cada frase genera `transcribing` y `transcription_result`; la
captura permanece activa mientras se transcribe. Una pausa de 0.8 s cierra la
frase, pero el texto se publica cuando termina también la inferencia.
`capture_started` indica aceptación; `audio_received` confirma que llegaron muestras.
La falta de audio produce `audio_capture_timeout` con el número de bytes recibidos.
Una autorización vencida se indica mediante `capture_cancelled`; un resultado
tardío descartado se indica con `result_discarded`. Estos estados no imprimen
audio ni texto reconocido. Solo la aplicación imprime texto con la opción indicada.

El primer arranque de voz permite hasta 60 s de espera por el worker o sus
metadatos de audio, sin bloquear el seguimiento. Se reintenta únicamente tras
rechazo explícito que confirma que no hubo captura. Un timeout de admisión no
se reintenta porque podría haberse aceptado la solicitud. La preparación del
modelo ocurre antes de exponer el servicio de captura, evitando cargarlo dentro
de una solicitud ya autorizada. El límite de la autorización sigue siendo 0.5 s.
El preview añade `speech_status` y `transcripts_received`; `waiting_worker`
puede significar que está cargando o que falta el proceso, y `waiting_audio_info`
indica ausencia de metadatos. Superar el plazo deshabilita solo la voz.

Las dimensiones y el frame deben corresponder a la cámara verificada. Esta
configuración adquiere `DetectPeople` y `TranscribeSpeech`, pero no mueve la
base ni la cabeza. La escucha es opt-in y también puede habilitarse en preview:
`dry_run` describe la ausencia de movimiento, **no la ausencia de captura de audio**.
La transcripción usa español por defecto y sigue activa durante la vida de
FollowPerson, salvo que falle la capacidad de voz. No añadir `record_duration`,
`timeout` ni `duration`: el seguimiento no tiene un temporizador total y
`record_duration` pertenece únicamente a la prueba independiente de una ventana.
Internamente se solicita `continuous=true`, `duration_sec=10.0` (máximo por
fragmento) y `silence_sec=0.8`. Una sola admisión produce múltiples resultados
con el mismo `request_id` y `segment_index` creciente. `final=true` cierra la
solicitud; los resultados normales de frases tienen `final=false`. El estado
`/hri/speech/capture_status` distingue escucha, voz e inferencia y comunica el
tamaño de la cola sin contenido hablado. Los heartbeats mantienen la sesión
durante silencios largos. La ausencia de audio durante 2 s o de heartbeat del
worker durante 2 s termina esa solicitud; no se confunde silencio con falta de
muestras. La inferencia conserva un plazo máximo de 60 s por fragmento.
`speech_language` sigue disponible si se necesita otro idioma.

Para combinarlo con movimiento, una vez verificadas percepción, captura y
paradas, añadir `enable_motion:=true confirm:=MOVER`. Si se desea orientación,
añadir además `orient_camera:=true` y cumplir los requisitos anteriores de
calibración y feedback. Las autorizaciones son independientes: `confirm_listen`
autoriza escuchar; `confirm` autoriza mover. `speech_provider` permite seleccionar
otro proveedor compatible y por defecto es
`hri_whisper_providers/WhisperTranscribeSpeech`.

La voz se adquiere con el mismo vínculo de Capabilities2, sin otra instancia de
`hri_app`. Un fallo de voz deshabilita esa función e intenta liberar su capacidad,
mientras el seguimiento puede continuar. No se reintenta indefinidamente ni se
oculta el error. Los fallos de visión, movimiento o del vínculo siguen deteniendo
el seguimiento completo. Ctrl+C solicita primero cero a la base y libera también
la transcripción; no se debe esperar a que termine una inferencia para pedir parada.

### Dónde ver la transcripción

En otra terminal con los entornos ROS cargados, antes de hablar:

```bash
ros2 topic echo /hri/speech/transcript --field text
```

Se muestra el texto cuando termina la inferencia de cada frase, no palabra por
palabra mientras hablas. El visor no inicia ni autoriza una captura. Para ver
también `success`, `message`, idioma y `request_id`, quitar `--field text`.
El estado de la escucha dentro del seguimiento, sin contenido hablado, se observa con:

```bash
ros2 topic echo /hri/follow_person/speech_status
```

También puede añadirse `print_transcript:=true` al launch para imprimir el texto
en la terminal de la aplicación. Está desactivado por defecto por privacidad.
No iniciar simultáneamente el modo independiente `transcribe_speech` para leer
texto: el tópico y el visor bastan.

### Privacidad y límites

El worker solo crea su suscripción de audio durante una captura aceptada,
mantiene buffers y cola acotados en memoria y no escribe audio ni transcripciones a
archivos. La aplicación no imprime el texto por defecto. **Esto no apaga el
micrófono del robot ni impide que su driver publique audio o que otros clientes
lo reciban/graben.** Restringir acceso al grafo ROS y contar con autorización de
las personas presentes.

Para mostrar el texto en consola puede añadirse `print_transcript:=true`, pero
el texto puede persistir en logs de `ros2 launch`, terminal o redirecciones.
El tópico de transcripción ya contiene texto incluso con impresión desactivada;
un consumidor externo también puede conservarlo. No se activa almacenamiento
duradero de conversación por defecto.

La liberación o pérdida del vínculo invalida resultados tardíos y detiene nuevas
capturas. Una inferencia CPU ya iniciada puede seguir calculando hasta finalizar;
su resultado no debe publicarse tras perder la autorización. VAD y el filtro
de energía no evitan todas las transcripciones erróneas o alucinaciones. El coste
de Whisper junto con YOLO y la calidad de reconocimiento real siguen pendientes
de medición en el computador del laboratorio.

## Verificación software realizada

Se verificaron compilación C++/colcon, sintaxis Python y pruebas unitarias de
geometría, adquisición opcional, ausencia de actuación en preview, feedback,
remapeo, cancelación y liberación aun ante errores de limpieza. La integración
ROS con backend simulado cubre desplazamiento → pausa → orientación → reanudación,
recorte, feedback vencido y Ctrl+C. No es evidencia de movimiento físico.

La transcripción tiene pruebas de captura acotada/formato/lease, worker con
inferencia simulada, proveedor ROS con backend falso y aplicación completa para
éxito, rechazo y cancelación. Esas pruebas iniciales no validaban reconocimiento con micrófono físico ni
rendimiento simultáneo con YOLO. Después, el autor confirmó transcripción real
en la versión anterior a VAD; esa confirmación no valida aún este incremento. No hay type-checker
estático Python ni ESLint configurados; compilar sintaxis no comprueba tipos.

La integración inicial de voz opcional en FollowPerson pasó **10/10 casos ROS simulados**:
voz desactivada, ventanas repetidas, impresión autorizada, rechazo, worker ausente,
resultado ajeno, timestamp antiguo, cancelación con solicitud pendiente, preview
y pérdida visual. Se verificaron también **172 pruebas unitarias de seguimiento,
11 de geometría y 11 del modo independiente de transcripción**.
[Informe compacto](results/follow_person_transcription_20261021.json).
Después de simplificar el comando, aquella versión pasó **175 pruebas
unitarias de seguimiento, 11 de geometría y 11 del modo independiente**, además
de repetir los **10/10 casos ROS** sin argumentos de duración ni idioma en
FollowPerson. Se verificó que las solicitudes usan internamente 3 s y español.
Los 172 casos anteriores se conservan como evidencia del incremento inicial;
el [informe de configuración simplificada](results/follow_person_speech_defaults_20261021.json)
registra la verificación posterior.
Esta evidencia comprueba coordinación, parada y liberación software, no calidad
ASR, funcionamiento del micrófono ni desempeño de inferencia durante marcha real.

El arranque automático del 4 de noviembre pasó 213 pruebas de seguimiento,
23 del worker/núcleo de captura y 12 del modo independiente. Se verificó también
la carga y preparación del modelo real con silencio sintético (10.87 s en ese
ensayo). No se capturó audio real ni se ordenó movimiento en estas verificaciones.
[Informe del incremento](results/whisper_auto_start_20261104.json).

La segmentación VAD y captura durante inferencia se verifican en
[whisper_vad_stream_20261104.json](results/whisper_vad_stream_20261104.json).
Al cambiar el contrato ROS es necesario cerrar la aplicación y workers anteriores,
recompilar `hri_capability_interfaces`, `hri_whisper_providers` y `hri_reference_app`
y cargar de nuevo `install/setup.bash`. Este workspace ya está compilado.

Desde este repositorio, con ROS y overlay cargados, las unidades no necesitan
driver ni hardware:

```bash
python3 -m unittest discover -s tests -p 'test_follow*.py'
python3 -m unittest discover -s tests -p test_camera_geometry.py
python3 -m unittest discover -s tests -p test_transcribe_mode.py
```

La integración siguiente crea endpoints de robot **simulados**: ejecutarla solo
sin drivers, en localhost y en un dominio dedicado libre, nunca en el dominio
de la prueba física:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=195 python3 tests/unified_hri_follow_camera_probe.py --software-only
```

Para repetir la integración de voz, sin otros probes o aplicaciones HRI activos
y con un dominio dedicado libre:

```bash
ROS_LOCALHOST_ONLY=1 ROS_DOMAIN_ID=197 python3 tests/unified_hri_follow_speech_probe.py --software-only
```

Para el seguimiento base y su evidencia anterior, consultar
[FOLLOW_PERSON.md](FOLLOW_PERSON.md).
