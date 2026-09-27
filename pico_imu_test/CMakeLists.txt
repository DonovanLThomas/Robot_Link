cmake_minimum_required(VERSION 3.13)

# Use -DPICO_SDK_PATH=/path/to/pico-sdk or the PICO_SDK_PATH environment variable.
if(NOT PICO_SDK_PATH)
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
endif()
if(NOT EXISTS "${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
    message(FATAL_ERROR "Set PICO_SDK_PATH to your Raspberry Pi Pico SDK directory")
endif()
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(pico_imu_test C CXX ASM)
set(CMAKE_C_STANDARD 11)
pico_sdk_init()

add_executable(pico_imu_test main.c)
target_link_libraries(pico_imu_test pico_stdlib hardware_i2c)
target_compile_options(pico_imu_test PRIVATE -Wall -Wextra)
pico_enable_stdio_usb(pico_imu_test 1)
pico_enable_stdio_uart(pico_imu_test 0)
pico_add_extra_outputs(pico_imu_test)
