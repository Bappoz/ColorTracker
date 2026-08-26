add_executable(ecv_tests
  test_main.cpp
  test_vision.cpp
  test_control.cpp
  test_track.cpp
  test_pipeline.cpp
)
# A camada de atuação só existe no build Linux; sem ela o resto dos testes roda igual.
if(UNIX AND NOT APPLE)
  target_sources(ecv_tests PRIVATE test_motors.cpp)
endif()
target_link_libraries(ecv_tests PRIVATE ecv_platform)
target_include_directories(ecv_tests PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_compile_options(ecv_tests PRIVATE -Wall -Wextra)

add_test(NAME ecv_tests COMMAND ecv_tests)
