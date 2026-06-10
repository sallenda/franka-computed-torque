cmake_minimum_required(VERSION 3.16)
project(IPRE2026)

set(CMAKE_CXX_STANDARD 17)

# Apunta a tu instalación de Drake
find_package(drake CONFIG REQUIRED
    PATHS $ENV{HOME}/drake-install/lib/cmake/drake
)

add_subdirectory(base)
add_subdirectory(lcm)
# add_subdirectory(test)  # descomenta cuando lo necesites