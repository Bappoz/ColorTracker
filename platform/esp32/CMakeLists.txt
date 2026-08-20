# Componente do ESP-IDF. Uso a partir de um projeto IDF:
#
#   set(EXTRA_COMPONENT_DIRS "<caminho>/embedded_computer_vision_models/platform/esp32")
#
# no CMakeLists.txt do projeto, antes de include($ENV{IDF_PATH}/tools/cmake/project.cmake).

idf_component_register(
  SRCS
    "esp32_cam_source.cpp"
    "ledc_motor_sink.cpp"
    "../../src/core/types.cpp"
    "../../src/core/profile.cpp"
    "../../src/vision/color.cpp"
    "../../src/vision/threshold.cpp"
    "../../src/vision/morphology.cpp"
    "../../src/vision/blobs.cpp"
    "../../src/vision/linescan.cpp"
    "../../src/app/sumo_vision.cpp"
  INCLUDE_DIRS
    "."
    "../../include"
  REQUIRES
    esp32-camera
    esp_timer
    esp_driver_ledc
)

# -O2 em vez do -Og padrão do IDF: o laço de pixel é o orçamento inteiro.
target_compile_options(${COMPONENT_LIB} PRIVATE -O2 -ffunction-sections)
