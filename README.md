# Simulación con brazo robotico Franka

Simulación de un brazo robótico Franka Panda con [Drake](https://drake.mit.edu), controlado por dinámica inversa. El sistema se divide en tres procesos independientes (trayectoria, controlador y simulación física) que se comunican exclusivamente a través de canales LCM.

## Requisitos del sistema

**Hardware:**
- RAM: mínimo 8 GB, recomendado 16 GB. Compilar Drake desde código fuente con Bazel es intensivo en memoria; con menos de 8 GB hay que limitar el paralelismo (`--jobs=1`) y el proceso puede tardar varias horas.
- Almacenamiento: al menos 30 GB libres (el repositorio de Drake y la caché de Bazel ocupan bastante espacio).

**Software base** (entorno probado con estas versiones):
- Sistema operativo: Ubuntu 24.04.3 LTS (Noble Numbat)
- Python: 3.12.3
- Bazel (gestionado por Bazelisk): 9.1.1

## Instalación de Drake

### 1. Dependencias del sistema

```bash
sudo apt update
sudo apt install -y \
    build-essential \
    cmake \
    git \
    curl \
    python3 \
    python3-pip \
    python3-venv \
    gcc \
    g++ \
    unzip \
    zip
```

### 2. Instalar Bazelisk

Drake usa Bazel como sistema de compilación. En lugar de instalar Bazel directamente se recomienda usar Bazelisk, que descarga y gestiona automáticamente la versión correcta de Bazel que requiere el repositorio (especificada en `.bazelversion`).

```bash
wget https://github.com/bazelbuild/bazelisk/releases/latest/download/bazelisk-linux-amd64
chmod +x bazelisk-linux-amd64
sudo mv bazelisk-linux-amd64 /usr/local/bin/bazelisk
```

Verificar la instalación:

```bash
bazelisk --version
```

Repositorio oficial: https://github.com/bazelbuild/bazelisk

### 3. Clonar y compilar Drake

```bash
git clone https://github.com/RobotLocomotion/drake.git
cd drake
bazelisk build //... --jobs=1
```

`--jobs=1` limita el paralelismo para reducir el consumo de memoria durante la compilación. El proceso puede tardar varias horas dependiendo del hardware disponible.

### 4. Instalar en un directorio local

Una vez compilado, Drake se instala en un directorio local (`~/drake-install`) para poder usarlo fuera del entorno de Bazel:

```bash
bazelisk run //:install -- ~/drake-install
```

Esto genera la estructura:

```
~/drake-install/
├── bin/
├── include/
├── lib/
│   ├── libdrake.so
│   └── python3.12/site-packages/pydrake/
└── share/
```

## Configuración del proyecto (C++)

Este proyecto usa **CMake**, no Bazel, apuntando a la instalación local de Drake generada arriba (`~/drake-install`). El `CMakeLists.txt` raíz declara la dependencia:

```cmake
find_package(drake CONFIG REQUIRED
    PATHS $ENV{HOME}/drake-install/lib/cmake/drake
)
```

Para compilar:

```bash
mkdir -p build && cd build
cmake ..
make -j$(nproc)
```

También se puede usar el script `run.sh`, que compila (solo recompila lo que cambió) y corre la simulación.

## Configuración de Python (pydrake, opcional)

Para usar `pydrake` desde scripts Python fuera de Bazel, agregar la ruta de instalación al `PYTHONPATH`. Añadir a `~/.bashrc`:

```bash
export DRAKE_RESOURCE_ROOT=$HOME/drake-install/share
export PYTHONPATH=$HOME/drake-install/lib/python3.12/site-packages:$PYTHONPATH
```

Aplicar los cambios:

```bash
source ~/.bashrc
```

Verificación:

```python
import pydrake
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.analysis import Simulator
print("pydrake OK")
```

## Ejecución

Compilar el proyecto:

```bash
cmake --build build -j4
```

Correr en 3 terminales, en este orden:

```bash
# Terminal 1 - Simulación (planta + Meshcat)
./build/src/mi_sim_lcm
# Abre la URL que imprime en el navegador para ver el robot

# Terminal 2 - Controlador (dinámica inversa)
./build/src/mi_controlador
# Mientras no corra mi_trayectoria, el robot se queda en la pose home

# Terminal 3 - Trayectoria (publica el estado deseado por LCM)
./build/src/mi_trayectoria
# El robot empieza a moverse al recibir los comandos de esta terminal
# Joint 0 oscila +-0.4 rad con periodo de 10 segundos
# Para ajustar: editar kFreqHz y kAmpRad en src/trayectoria.cc
```

Más detalles de la arquitectura de los tres procesos y los canales LCM en [Documentacion.pdf](Documentacion.pdf).


