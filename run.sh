#!/bin/bash

export DRAKE_RESOURCE_ROOT=$HOME/drake-install/share

# Ir al directorio del script sin importar desde donde lo llames
cd "$(dirname "$0")"

# Compilar (solo recompila lo que cambió)
mkdir -p build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release > /dev/null 2>&1  # solo la primera vez tiene efecto real
make -j$(nproc)

# Si compiló bien, correr
if [ $? -eq 0 ]; then
    ./src/mi_sim_lcm
else
    echo "Error en compilación"
fi