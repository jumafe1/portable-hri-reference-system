# FollowPerson: duración y cancelación — 2026-10-19

Solo software, Docker ROS 2 Jazzy, dominios aislados y fuentes/receptores simulados. No se ejecutaron robots ni se modificó la vida autónoma.

Cambios: duration 0.1–300 s (default 2); timeout 1–300 s (default 120); se aplica el menor. Preview continúa predeterminado. Ctrl+C conserva el servidor durante la limpieza de la aplicación, con supervisor de cierre acotado.

## Resultados conservados

- `before_shutdown_fix.json`: fallo reproducido del cierre simultáneo. SIGINT al launch dejó la última orden recibida distinta de cero, sin confirmación de parada/liberación. La ejecución terminó anticipadamente al no obtener JSON del caso de grupo completo. El caso de señal solo al hijo aparece fallido por una aserción inicial incorrecta del mensaje, aunque sus flags de parada/liberación eran correctos; la aserción se corrigió al valor real `cancelled_or_ros_shutdown`. No interpretar este informe como una batería completa.
- `motion_first.json`: 21/21 tras corregir el cierre. Incluye señal al hijo, launch y grupo completo con configuraciones 300/120 y 300/300.
- `motion_repeat.json`: 21/21 con aserciones adicionales: cero recibido como máximo 1 s después de la señal y todas las órdenes posteriores permanecen en cero. Tres cancelaciones con `stop_acknowledged`, `motion_released` y `people_released` verdaderos.
- `preview_regression.json`: 20/20, incluido no crear publicadores ni mensajes de velocidad en preview.

Además: 105/105 unitarias repetidas, compilación colcon de hri_reference_app, sintaxis Python, CTest del proveedor 1/1 y diff sin errores. No hay type-checker estático ni ESLint configurados; compileall comprueba sintaxis, no tipos. Revisión independiente del supervisor y API de launch sin bloqueos.

## Límites de la evidencia

Los plazos completos de 120/300 s se verifican con reloj simulado. Las pruebas ROS configuradas a 300 s se cancelan al comenzar el flujo; no constituyen un ensayo continuo de cinco minutos. Las latencias registradas corresponden a recepción de Twist por un nodo simulado, no al frenado de motores. Caída de red/proveedor o SIGKILL pueden impedir limpieza: sigue siendo necesaria la parada física independiente.

El hallazgo anterior de Pepper MoveRelative (7/8) no se corrigió ni se volvió a calificar aquí. Las velocidades máximas, validación de percepción y watchdog permanecen intactos. Ningún resultado físico previo fue sobrescrito.
