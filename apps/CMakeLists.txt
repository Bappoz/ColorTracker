add_executable(ecv_bench bench/main.cpp)
target_link_libraries(ecv_bench PRIVATE ecv_platform)

add_executable(ecv_sumo_sim sumo_sim/main.cpp)
target_link_libraries(ecv_sumo_sim PRIVATE ecv_platform)

add_executable(ecv_calibrate calibrate/main.cpp)
target_link_libraries(ecv_calibrate PRIVATE ecv_platform)

set(ecv_app_targets ecv_bench ecv_sumo_sim ecv_calibrate)

foreach(target ${ecv_app_targets})
  target_compile_options(${target} PRIVATE -Wall -Wextra)
endforeach()
