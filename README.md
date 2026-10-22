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
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j4
```

Correr en 3 terminales, en cualquier orden (lo usual es este):

```bash
# Terminal 1 - Simulación (planta + Meshcat)
./build/src/mi_sim_lcm
# Abre la URL que imprime en el navegador para ver el robot.
# El robot parte en home y la simulación queda en pausa hasta que responda el controlador.

# Terminal 2 - Controlador (ETSO, modo FL)
./build/src/mi_controlador
# Mientras no corra mi_trayectoria, el robot se queda en la pose home

# Terminal 3 - Trayectoria (publica el estado deseado por LCM)
./build/src/mi_trayectoria
# El robot empieza a moverse al recibir los comandos de esta terminal
# Joint 0 oscila +-0.4 rad con periodo de 10 segundos
# Para ajustar: editar kFreqHz y kAmpRad en src/trayectoria.cc
```

Si el controlador se cae, la simulación se pausa (el robot no se cae) y se reanuda sola al relanzarlo.

### Lazo síncrono a 1 kHz (`src/protocolo_lcm.h`)

Los tres procesos siguen comunicándose solo por LCM, pero en un ciclo síncrono, igual que libfranka con el Franka real: el robot entrega el estado en t y el controlador devuelve el torque de ese mismo instante. Un ciclo, cada 1 ms:

1. `mi_sim_lcm` publica `PANDA_STATUS` con `utime = t`.
2. `mi_trayectoria` responde `DESIRED_STATE` con `timestamp = t`.
3. `mi_controlador`, con estado(t) y referencia(t), publica `PANDA_COMMAND` con `utime = t`.
4. `mi_sim_lcm` aplica ese torque y avanza 1 ms.

Antes cada proceso publicaba con su propio reloj, y entre el estado medido y el torque aplicado se acumulaban de 1 a 6 ms de retardo variable. Ahora el lazo LCM da exactamente lo mismo que una simulación de un solo proceso con el controlador muestreado a 1 ms. Si se pierde un paquete, la planta reenvía el estado a los 50 ms y el controlador reenvía el comando.

### Precisión alcanzable a 1 kHz (Δ por joint)

Con el controlador muestreado cada h = 1 ms, la ganancia del lazo por periodo de cada joint es

κ = h · (α + k) · J_max / j_min

Si κ es muy grande, el torque oscila a la frecuencia de Nyquist (+τ_max, −τ_max, …): eso es el chattering. En la simulación síncrona aparece con κ ≥ ~1.25 y no aparece con κ ≤ ~1.15.

Como k es inversamente proporcional a Δ, el controlador (`src/controlador.cc`, PASO 0g) calcula para cada joint el menor Δ que deja κ ≤ `kKappaMax` = 1.0. Usa ese Δ solo si es mayor que la precisión deseada `kPrecisionPorc`, y lo avisa al partir. La tabla de parámetros muestra Δ, e_max y κ de cada joint. Hoy resulta Δ ≈ [3.5, 2.3, 2.4, 1.9, 1.2, 1.3, 1.2] %.

J_max y j_min se calculan en la región de trabajo (home ± (A + 0.1 rad)), no en todo el rango del URDF. En el rango completo, J_max/j_min llega a ~6 en el joint 1 y obligaría a un Δ cerca de 2 veces mayor. Por eso la planta parte en home: la garantía del diseño vale mientras el robot esté en esa región.

Más detalles de la arquitectura de los tres procesos y los canales LCM en [Documentacion.pdf](Documentacion.pdf).

## Fricción articular

El URDF del Panda no trae fricción, así que la planta simulada (`franka_simulation.cc`) usa el modelo identificado por Gaz et al. (2019) en un Panda real (`src/friccion_franka.*`). Es una sigmoide por joint, una especie de Coulomb suave que se satura en ±0.1–0.7 Nm según el joint:

τ_f(q̇) = φ₁ / (1 + e^(−φ₂(q̇+φ₃))) − φ₁ / (1 + e^(−φ₂φ₃))

El controlador ETSO usa el modelo lineal del paper (J q̈ + B q̇), por eso:

- **B** de cada joint es la fricción viscosa equivalente: el ajuste por mínimos cuadrados de τ_f ≈ B·q̇ en el rango de velocidades de la referencia (|q̇| ≤ 0.25 rad/s). Resulta entre 0.6 y 1.6 Nms/rad.
- Lo que el modelo lineal no explica, max |τ_f − B·q̇| hasta el límite de velocidad del joint, se suma a **τ_Lmax**, como carga no modelada de la ec. (4).

Los φ son los del robot de Gaz et al.; un Franka específico puede tener otros valores.

> C. Gaz, M. Cognetti, A. Oliva, P. Robuffo Giordano, A. De Luca, "Dynamic Identification of the Franka Emika Panda Robot With Retrieval of Feasible Parameters Using Penalty-Based Optimization", IEEE Robotics and Automation Letters, 2019.



