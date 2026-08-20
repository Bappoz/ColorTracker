cmake_minimum_required(VERSION 3.16)
project(ecv LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

option(ECV_NATIVE_ARCH "Otimiza para a CPU do host (não usar em build de release cruzado)" OFF)

# Núcleo portátil: nada além da libstdc++ freestanding. É o que compila igual
# no host, no Raspberry Pi e como componente do ESP-IDF.
add_library(ecv_core STATIC
  src/core/types.cpp
  src/core/profile.cpp
  src/vision/color.cpp
  src/vision/threshold.cpp
)
target_include_directories(ecv_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_options(ecv_core PRIVATE -Wall -Wextra -Wpedantic -Wshadow -Wconversion)
if(ECV_NATIVE_ARCH)
  target_compile_options(ecv_core PUBLIC -march=native)
endif()
