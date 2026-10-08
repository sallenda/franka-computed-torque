# Experimento completo en un solo proceso: planta + referencia + ETSO + logging
add_executable(experimento experimento.cc)
target_link_libraries(experimento PRIVATE ipre2026_comun)

# Estimacion de la friccion viscosa B a partir de un log (q, dq, tau)
add_executable(estimar_friccion estimar_friccion.cc)
target_link_libraries(estimar_friccion PRIVATE ipre2026_comun)
