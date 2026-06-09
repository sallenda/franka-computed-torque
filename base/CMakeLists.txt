# Librería compartida con planta y controlador
add_library(ipre2026_base
    planta_comun.cc
    planta_comun.h
    controlador.cc
    controlador.h
)

target_link_libraries(ipre2026_base
    PUBLIC drake::drake
)

target_include_directories(ipre2026_base
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

# Ejecutable principal
add_executable(mi_sim main.cc)

target_link_libraries(mi_sim
    PRIVATE ipre2026_base
    PRIVATE drake::drake
)