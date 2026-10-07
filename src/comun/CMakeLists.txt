# Libreria compartida por las versiones con y sin LCM
add_library(ipre2026_comun
    modelo_panda.cc
    controlador.cc
)
target_link_libraries(ipre2026_comun PUBLIC drake::drake)
target_include_directories(ipre2026_comun PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
