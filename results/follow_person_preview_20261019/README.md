# Evidencia software — follow_person sin movimiento

19 de octubre de 2026. Docker ROS 2 Jazzy, arm64, localhost y dominios
dedicados. Detecciones, servicios y receptores del robot simulados; no se
conectó ni movió hardware. Estos resultados no validan seguimiento físico.

| Verificación | Resultado | Evidencia |
| --- | --- | --- |
| Controlador, adaptador y cierre del modo sin grafo | 52/52 | `unit-tests-publication.log` (generado localmente; no incluido en esta publicación) |
| follow_person: verificación histórica anterior al push | 19/19 | `integration/result.json` |
| follow_person: ejecutor unificado, revalidación | 19/19 | `publication_fixed.json` |
| follow_person: segunda corrida del ejecutor unificado | 19/19 | `publication_fixed_repeat.json` |
| Speak, DetectPeople y MoveRelative: ambos robots simulados y argumentos | 8/8 | `legacy.json` |
| MoveVelocity: primera regresión | 10/11 | `velocity_regression_first/result.json` |
| MoveVelocity: repetición en otro dominio aislado | 11/11 | `velocity_regression_repeat/result.json` |

`colcon build --packages-select hri_reference_app --symlink-install` y
`python3 -m compileall -q hri_reference_app` terminaron correctamente.
No hay comprobador estático de tipos ni ESLint configurados para este paquete;
mypy no está instalado. No presentar compileall como análisis estático de tipos.

## Alcance del incremento

El nuevo modo adquiere únicamente DetectPeople. El probe observa también
`/hri/cmd_vel`, `/nao/cmd_vel` y `/pepper/cmd_vel` y no registra publicadores
ni mensajes de movimiento, incluidos ceros. El diagnóstico numérico de
velocidad es un String JSON best-effort, no un comando Twist.

Se verificaron candidatos con/sin tracking ID, selección de tópico YOLO,
ausencia de datos/personas, pérdida tras adquisición, marcas de tiempo
antiguas/futuras/repetidas, frame incorrecto, ambigüedad y SIGINT. Los casos
de fallo esperados terminan con `application_failed` y liberan la capacidad;
eso es una prueba aprobada del manejo del fallo, no un seguimiento aprobado.

La revisión identificó y corrigió dos carreras: tomar tiempo antes del lock
y declarar éxito al vencer el plazo sin revisar el último fallo recibido.
Ambas tienen regresiones deterministas en `test_follow_session.py`. La
fuente sintética se separó del procesamiento de diagnósticos y se espera al
observador antes de inyectar datos, para que una prueba corta no finalice
antes del descubrimiento DDS. No se relajó la caducidad de 0.75 segundos.

## Revalidación previa a publicación

La primera repetición previa al push no reprodujo el 19/19 histórico:
`publication_recheck_failed.json` conserva 13/19 y
`publication_recheck_failed_second.json` conserva otra corrida fallida.
La instrumentación (`publication_recheck_instrumented.json`) mostró pausas
de más de 0.75 s en la propia fuente sintética de 10 Hz, pese a que cada
llamada publish tardaba menos de 0.6 ms.

Un ejecutor de un solo hilo en el banco de pruebas estabilizó la fuente
(intervalos máximos aproximados de 0.106 s y recepción independiente de
`/hri/people` con antigüedad menor a 0.004 s). Sin embargo, la aplicación
seguía procesando menos muestras y declaraba caducidad; evidencia en
`publication_recheck_fixture_fixed.json`. Un grupo de callbacks dedicado
no bastó (`publication_recheck_callback_group.json`). No se relajó el
umbral de 0.75 s ni se convirtieron estos fallos en aprobados.

Tampoco bastó mantener el ejecutor multihilo de YASMIN junto a un receptor
de un solo hilo (`publication_recheck_separate_receiver.json`, 17/19).
La versión final usa un único nodo y un `SingleThreadedExecutor` para este
modo. Se inyecta el nodo mediante las APIs públicas `ServiceState(node=...)`
y `set_ros_loggers(node)`: conserva la orquestación YASMIN, sin modificar
sus atributos privados ni el código de Capabilities2. Con este cambio la
integración obtuvo 19/19 en dos corridas consecutivas (`publication_fixed.json`
y `publication_fixed_repeat.json`). La evidencia localiza
el problema en la configuración de ejecución; no demuestra un defecto
general de YASMIN ni una garantía temporal bajo cualquier carga.

Las pruebas nuevas cubren construcción, rollback y cierre ordenado del
nodo/ejecutor. Si no se logra detener el hilo, se informa fallo y no se
destruye su contexto ROS mientras siga activo. El bond se cierra antes
del nodo. Se preservaron todos los resultados previos, incluidos fallos.

## Hallazgo pendiente de movimiento

En la primera regresión de MoveVelocity, `multiple_publishers_rejected`
falló: el cliente terminó exitosamente y envió 18 comandos pese al publicador
adicional creado por el probe. En la repetición, se rechazó correctamente
antes de enviar órdenes. **La repetición no demuestra una corrección.**

`velocity_mode.py` y el proveedor de movimiento no fueron modificados en
este incremento. El ensayo no conservaba GIDs ni evidencia del momento en
que la aplicación descubrió el publicador adicional; descubrimiento DDS o
temporización son hipótesis, no una causa demostrada. Tampoco se establece
que el fallo sea nuevo o preexistente sin una comparación controlada contra
la versión anterior. Conservar ambos resultados; revisar exclusión y parada
independiente antes de habilitar seguimiento físico. La observación sin
movimiento puede validarse por separado con la cámara real.

Los campos `evidence_directory` de los JSON conservan la ubicación temporal
original dentro del contenedor. Aquí se copiaron sus resultados y logs sin
alterar esos campos. Los logs se conservan localmente y no se incluyen en
esta publicación. No se sobrescribieron resultados físicos previos.
