# Librería compartida con planta y controlador
add_library(ipre2026_lcm
    franka_simulation.cc
    planta_comun.h
    controlador.cc
    controlador.h
)

target_link_libraries(ipre2026_lcm
    PUBLIC drake::drake
)

target_include_directories(ipre2026_lcm
    PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}
)

# Ejecutable de la simulación (planta del robot)
add_executable(mi_sim_lcm main.cc)

target_link_libraries(mi_sim_lcm
    PRIVATE ipre2026_lcm
    PRIVATE drake::drake
)

# Ejecutable del controlador (proceso separado, comunicado por LCM)
add_executable(mi_controlador franka_task_control.cc)

target_link_libraries(mi_controlador
    PRIVATE ipre2026_lcm
    PRIVATE drake::drake
)

# Ejecutable del publicador de trayectoria
add_executable(mi_trayectoria trayectoria.cc)

target_link_libraries(mi_trayectoria
    PRIVATE drake::drake
)