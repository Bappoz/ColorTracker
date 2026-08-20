cmake_minimum_required(VERSION 3.16)
project(ecv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(ECV_WITH_V4L2 "Compila a fonte de frames V4L2 (Linux/RPi)" ${UNIX})
option(ECV_NATIVE_ARCH "Otimiza para a CPU do host (não usar em build de release cruzado)" OFF)

# Núcleo portátil: nada além da libstdc++ freestanding. É o que compila igual
# no host, no Raspberry Pi e como componente do ESP-IDF.
add_library(ecv_core STATIC
  src/core/types.cpp
  src/core/profile.cpp
  src/vision/color.cpp
  src/vision/threshold.cpp
  src/vision/morphology.cpp
  src/vision/blobs.cpp
  src/vision/linescan.cpp
  src/vision/calibrate.cpp
  src/app/sumo_vision.cpp
)
target_include_directories(ecv_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ecv_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
if(ECV_NATIVE_ARCH)
  target_compile_options(ecv_core PUBLIC -march=native)
endif()

# Camada de plataforma para host/RPi (usa STL e syscalls — fora do ecv_core).
add_library(ecv_platform STATIC
  platform/sim/ppm_io.cpp
  platform/sim/scene_source.cpp
)
target_link_libraries(ecv_platform PUBLIC ecv_core)
target_include_directories(ecv_platform PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/platform)
target_compile_options(ecv_platform PRIVATE -Wall -Wextra -Wpedantic)

if(ECV_WITH_V4L2 AND UNIX AND NOT APPLE)
  target_sources(ecv_platform PRIVATE platform/linux/v4l2_source.cpp platform/linux/pwm_sink.cpp)
  target_compile_definitions(ecv_platform PUBLIC ECV_HAS_V4L2=1)
endif()
