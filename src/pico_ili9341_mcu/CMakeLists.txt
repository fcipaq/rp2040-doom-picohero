cmake_minimum_required(VERSION 3.12)

# initialize pico_sdk from GIT
# (note this can come from environment, CMake cache etc)
# set(PICO_SDK_FETCH_FROM_GIT on)

# pico_sdk_import.cmake is a single file copied from this SDK
# note: this must happen before project()
include(pico_sdk_import.cmake)

project(pico_ili9341_mcu)

# initialize the Pico SDK
pico_sdk_init()

add_library(pico_ili9341_mcu INTERFACE)

target_sources(pico_ili9341_mcu INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/ili9341_mcu.c
)

target_include_directories(pico_ili9341_mcu INTERFACE
    ${CMAKE_CURRENT_LIST_DIR}/src/include
)

target_link_libraries(pico_ili9341_mcu INTERFACE pico_stdlib hardware_pwm)

