# Build cruzado para Raspberry Pi 3 (BCM2837, Cortex-A53) com SO 64 bits.
#
# Só serve para Raspberry Pi OS / Ubuntu **arm64**. A imagem 32 bits ainda é o
# padrão de muitas instalações do Pi 3: nela o alvo é armv7l e este toolchain
# não vale — confirmar com `uname -m` na placa antes de copiar o binário.
#
# Linka estático por padrão: a glibc do cross-toolchain quase nunca casa com a
# do Raspberry Pi OS, e um binário de benchmark não ganha nada em ser dinâmico.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR aarch64)

set(CMAKE_C_COMPILER aarch64-linux-gnu-gcc)
set(CMAKE_CXX_COMPILER aarch64-linux-gnu-g++)

set(ECV_RPI3_FLAGS "-mcpu=cortex-a53 -mtune=cortex-a53")
set(CMAKE_C_FLAGS_INIT "${ECV_RPI3_FLAGS}")
set(CMAKE_CXX_FLAGS_INIT "${ECV_RPI3_FLAGS}")

option(ECV_CROSS_STATIC "Linka estaticamente (evita incompatibilidade de glibc)" ON)
if(ECV_CROSS_STATIC)
  set(CMAKE_EXE_LINKER_FLAGS_INIT "-static")
endif()

set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
