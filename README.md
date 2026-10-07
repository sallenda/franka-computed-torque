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

## Estructura del código

```
src/
├── comun/        Código compartido por ambas versiones
│   ├── modelo_panda.*   Carga del Panda con fricción viscosa B (el URDF no la trae)
│   ├── referencia.h     Referencias analíticas q_d, dq_d, ddq_d
│   └── controlador.*    Controlador ETSO del paper (modos VSC, FL, FF)
├── sin_lcm/      Réplica del paper en un solo proceso (planta + controlador + logging)
│   ├── experimento.cc       Simulación determinista, escribe log.csv y parametros.csv
│   └── estimar_friccion.cc  Estimación de B por mínimos cuadrados desde un log
└── con_lcm/      Arquitectura de 3 procesos comunicados por LCM (camino al Franka real)
scripts/graficar.py   Figuras estilo Figs. 5-8 del paper
```

El controlador se define una sola vez en `src/comun`, así que cualquier cambio de diseño se refleja en ambas versiones.

## Ejecución

Compilar el proyecto:

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
export DRAKE_RESOURCE_ROOT=$HOME/drake-install/share
```

### Sin LCM (réplica del paper)

```bash
# Modo FL, robot partiendo 0.3 rad desplazado de la referencia (fase de alcance)
./build/src/sin_lcm/experimento --modo=fl --offset=0.3 --t_final=10 --salida=resultados/fl
./build/src/sin_lcm/experimento --modo=vsc --offset=0.3 --t_final=10 --salida=resultados/vsc
./build/src/sin_lcm/experimento --modo=ff --offset=0.3 --t_final=10 --salida=resultados/ff

# Figuras comparando las tres estructuras (una por joint)
python3 scripts/graficar.py resultados/vsc resultados/fl resultados/ff --salida figuras
```

Opciones de `experimento`: `--modo=vsc|fl|ff`, `--referencia=seno|excitacion`, `--t_final`, `--dt` (paso de la planta, 1e-4 por defecto; 0 = continua), `--delta` (precisión Δ en % de A), `--offset` (desplazamiento inicial en rad), `--B_planta` (fricción de la planta, por defecto la misma que asume el controlador), `--dt_log`, `--salida`, `--visualizar` (Meshcat en tiempo real).

Al terminar imprime, por joint, el tiempo de alcance, el error máximo en la fase de seguimiento comparado con e_max = Δ·A y el porcentaje de tiempo con torque saturado.

**Estimación de la fricción viscosa B:**

```bash
./build/src/sin_lcm/experimento --modo=fl --referencia=excitacion --B_planta=6 --t_final=30 --salida=resultados/excitacion
./build/src/sin_lcm/estimar_friccion resultados/excitacion/log.csv            # solo viscosa
./build/src/sin_lcm/estimar_friccion resultados/excitacion/log.csv --coulomb  # viscosa + Coulomb
```

Sirve con cualquier CSV que tenga las columnas `t, q1..q9, dq1..dq9, tau1..tau9`, por ejemplo un log del robot real.

### Con LCM

Correr en 3 terminales, en este orden:

```bash
# Terminal 1 - Simulación (planta + Meshcat)
./build/src/con_lcm/mi_sim_lcm
# Abre la URL que imprime en el navegador para ver el robot

# Terminal 2 - Controlador (ETSO, modo FL por defecto)
./build/src/con_lcm/mi_controlador
# Mientras no corra mi_trayectoria, el robot se queda en la pose home

# Terminal 3 - Trayectoria (publica el estado deseado por LCM)
./build/src/con_lcm/mi_trayectoria
# Joint 1 oscila +-0.4 rad con periodo de 10 segundos
# Para ajustar: editar kFrecuenciaRefHz y kAmplitudRef en src/comun/referencia.h
```

Nota: con el lazo LCM a 5 ms las ganancias del ETSO producen chattering (torque conmutando entre ±τ_max). La versión sin LCM no lo tiene; adaptar el diseño al muestreo es el siguiente paso.

Más detalles de la arquitectura de los tres procesos y los canales LCM en [Documentacion.pdf](Documentacion.pdf).
