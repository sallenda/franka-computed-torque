cmake_minimum_required(VERSION 3.16)
project(IPRE2026)

set(CMAKE_CXX_STANDARD 17)

# Apunta a tu instalación de Drake
find_package(drake CONFIG REQUIRED
    PATHS $ENV{HOME}/drake-install/lib/cmake/drake
)

# Codigo compartido: modelo del Panda, referencia y controlador ETSO
add_subdirectory(src/comun)
# Replica del paper en un solo proceso (sin LCM)
add_subdirectory(src/sin_lcm)
# Arquitectura de 3 procesos comunicados por LCM (camino al Franka real)
add_subdirectory(src/con_lcm)
# add_subdirectory(test)  # descomenta cuando lo necesites