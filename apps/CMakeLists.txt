add_executable(ecv_bench bench/main.cpp)
target_link_libraries(ecv_bench PRIVATE ecv_platform)

add_executable(ecv_sumo_sim sumo_sim/main.cpp)
target_link_libraries(ecv_sumo_sim PRIVATE ecv_platform)

add_executable(ecv_calibrate calibrate/main.cpp)
target_link_libraries(ecv_calibrate PRIVATE ecv_platform)

add_executable(ecv_soak soak/main.cpp)
target_link_libraries(ecv_soak PRIVATE ecv_platform)

set(ecv_app_targets ecv_bench ecv_sumo_sim ecv_calibrate ecv_soak)

# Só existem onde a camada V4L2/sysfs foi compilada.
if(ECV_WITH_V4L2 AND UNIX AND NOT APPLE)
  add_executable(ecv_sumo_robot sumo_robot/main.cpp)
  target_link_libraries(ecv_sumo_robot PRIVATE ecv_platform)

  add_executable(ecv_probe probe/main.cpp)
  target_link_libraries(ecv_probe PRIVATE ecv_platform)

  list(APPEND ecv_app_targets ecv_sumo_robot ecv_probe)
endif()

foreach(target ${ecv_app_targets})
  target_compile_options(${target} PRIVATE -Wall -Wextra)
endforeach()
