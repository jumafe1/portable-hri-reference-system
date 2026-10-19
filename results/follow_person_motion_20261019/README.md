# Integración FollowPerson con MoveVelocity — 19 de octubre de 2026

Pruebas de software en Docker ROS 2 Jazzy, sin drivers ni robots físicos. YASMIN y Capabilities2 reales, detecciones sintéticas y suscriptores de velocidad de prueba. Los JSON son resultados generados, no reconstrucciones de pruebas físicas. Los paths temporales internos indican el lugar original de ejecución.

| Informe conservado | Resultado | Alcance |
| --- | --- | --- |
| [Primera integración](motion_first.json) | 16/16 | Rutas NAO/Pepper, avance y ambos giros, pérdidas, antigüedad, ambigüedad, SIGINT y fuentes duplicadas |
| [Repetición ampliada](motion_repeat.json) | 18/18 | Añade booleano inválido y confirmación sin opt-in: sigue sin movimiento |
| [Regresión preview](preview_regression.json) | 20/20 | Sin publicadores ni mensajes de velocidad por defecto |
| [Regresión MoveVelocity](velocity_regression.json) | 12/12 | Modo aislado y rechazo de publicadores concurrentes |
| [Repetición MoveVelocity](velocity_repeat.json) | 12/12 | Segunda corrida independiente |
| [Proveedor directo](provider_regression.json) | 22/22 | Límites, watchdog, ceros, disponibilidad y exclusión con MoveRelative |
| [Exclusividad directa](exclusivity.json) | 3/3 | Conflicto durante movimiento simulado y reemplazo de GID; fallo enclavado |
| [Fallo previo de velocidad](velocity_before_executor_fix.json) | 10/12 | Conservado: envío interrumpido y `command_timeout` antes del cambio de ejecutor |
| [Otros modos, primera repetición](legacy_modes_first.json) | 7/8 | Pepper MoveRelative informa servicio no disponible; sin llamada al robot simulado |
| [Otros modos, segunda repetición](legacy_modes_repeat.json) | 7/8 | Reproduce el mismo hallazgo; no se considera regresión completa aprobada |

La compilación de tres paquetes con colcon y sintaxis Python pasó; las 87 pruebas unitarias `test_follow_*.py` y CTest `motion_lease` (1/1) pasaron. No hay type-checker estático Python ni ESLint configurados; compileall no sustituye comprobación estática de tipos.

El modo de velocidad anterior enviaba una sola orden y el proveedor detenía por watchdog. Tras inyectar un nodo y SingleThreadedExecutor propios en YASMIN, envió 23–24 órdenes en 1.2 s en las dos regresiones aprobadas; no se aumentó el watchdog. El proveedor ahora comprueba origen único y GID estable después de un periodo de asentamiento del grafo. La revisión independiente detectó además una limpieza diagnóstica que podía impedir liberar DetectPeople; se separaron las tareas y se añadieron pruebas de excepciones.

## Límites de la evidencia

**Hallazgo independiente pendiente:** la regresión de Speak/DetectPeople/MoveRelative no es consistentemente aprobada. La primera ejecución observada fue 5/8; otra con el banco sin modificar fue 8/8 (salidas conservadas en la conversación de desarrollo, no en estos JSON). Con el banco instrumentado y ejecutor de un hilo se obtuvo 7/8 dos veces: Pepper MoveRelative devuelve `move_to_service_unavailable` inmediatamente, aun con preflight de servicios aprobado y cadencia de odometría sintética saludable (intervalo máximo aproximado 0.105 s). Los proveedores Speak/MoveRelative no se modificaron; usan `service_is_ready()` al recibir la petición. Los datos son compatibles con una carrera de descubrimiento del cliente, pero no se atribuye definitivamente la causa a ese mecanismo ni al ejecutor. No se repitieron automáticamente órdenes fallidas para ocultar el fallo. La integración nueva FollowPerson pasa sus pruebas, pero esto no demuestra que todos los modos estén libres de fallos.

Los launchers comparten una ruta SQLite: ejecutar los probes secuencialmente aunque usen dominios ROS diferentes. El banco de regresión ahora exige `--software-only`, localhost, dominio aislado y preflight; conserva cada resultado en un directorio temporal nuevo.

- Un cero recibido o `stop_acknowledged` acredita comportamiento software, no parada física.
- El descubrimiento DDS es asíncrono; no es una exclusión distribuida instantánea ni autenticación.
- La pérdida total de proveedor, driver o red no garantiza parada del robot.
- No se mide distancia, no se evitan obstáculos y no se compensa movimiento de cabeza.
- El ensayo físico está pendiente. Por defecto sigue `dry_run: true`; activar movimiento exige confirmación explícita y un entorno supervisado.
- Esta carpeta no implica commit ni publicación en GitHub. Resultados físicos previos del autor se preservaron sin modificarlos.
