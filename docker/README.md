# Entorno Docker

Este entorno instala ROS 2 Jazzy, YASMIN, las herramientas de compilación y
las dependencias binarias de Capabilities2. No incluye el driver SinfonIA de
NAO/Pepper: para una prueba física, el driver puede ejecutarse en el computador
del laboratorio y compartir el mismo dominio ROS 2.

La estructura esperada es un workspace con los repositorios como hermanos:

```text
portable_hri_ws/src/
├── capabilities2/
├── naoqi_utilities_msgs/
├── yolo_ros/
├── portable-hri-interfaces/
├── portable-hri-providers/
└── portable-hri-reference-system/
```

Para prepararla desde cero:

```bash
mkdir -p portable_hri_ws/src
cd portable_hri_ws/src
git clone https://github.com/jumafe1/portable-hri-reference-system.git
vcs import --skip-existing . \
  < portable-hri-reference-system/portable_hri.repos
```

Construir y abrir el contenedor desde `portable_hri_ws/src`:

```bash
export PORTABLE_HRI_SRC="$PWD"
docker compose \
  -f portable-hri-reference-system/docker/compose.yaml \
  build
docker compose \
  -f portable-hri-reference-system/docker/compose.yaml \
  up -d
docker compose \
  -f portable-hri-reference-system/docker/compose.yaml \
  exec dev bash
```

Dentro del contenedor:

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

Para compilar únicamente el contrato y el adaptador de detecciones, sin cargar
el modelo de inferencia:

```bash
colcon build --packages-up-to hri_yolo_providers --symlink-install
source install/setup.bash
python3 src/portable-hri-reference-system/tests/capabilities_yolo_detect_people_probe.py
```

La ejecución real de `yolo_ros` requiere además sus dependencias Python y los
pesos del modelo. La prueba física inicial se realizó en el computador Ubuntu
del laboratorio con el entorno `uv` recomendado por el proyecto upstream; el
contenedor actual verifica el contrato y el adaptador, no rendimiento de
inferencia.

`network_mode: host` facilita la integración ROS 2 con procesos del host. En
Docker Desktop para macOS debe estar habilitada la opción de host networking;
la disponibilidad de multicast DDS en la red local se debe validar por
separado. La conexión directa del driver con el robot también depende de la
ruta de red y del puerto NAOqi de la instalación.
