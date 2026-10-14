# Evidencia de software de MoveVelocity

Entorno: ROS 2 Jazzy, Linux ARM64 en Docker, 14 de octubre de 2026.
Sin drivers ni robots físicos; DDS limitado a localhost y dominios de ensayo.

- `provider/result.json`: 22/22, proveedor con entrada Twist y salida simulada.
- `application/result.json`: 11/11, aplicación YASMIN con NAO/Pepper simulados.
- `regression.json`: regresión de los tres modos existentes (8 comprobaciones).

Los informes distinguen la exclusión operativa del fallo de catálogo conocido
de Capabilities2. Una entrada rechazada puede permanecer reportada como running
sin tener endpoints operativos. Los logs conservan ese hecho; no es un arreglo
del núcleo externo.

La prueba CTest `motion_lease` (1/1) verifica exclusión entre procesos y bloqueo ante
incertidumbre. Ninguno de estos ensayos acredita movimiento o parada física.
