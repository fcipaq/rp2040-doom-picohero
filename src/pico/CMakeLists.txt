add_library(common_pico INTERFACE)
target_sources(common_pico INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}/i_glob.c
        ${CMAKE_CURRENT_LIST_DIR}/i_input.c
        ${CMAKE_CURRENT_LIST_DIR}/i_system.c
        $<$<BOOL:${USE_PICO_NET}>:${CMAKE_CURRENT_LIST_DIR}/piconet.c>
        
        ${CMAKE_CURRENT_LIST_DIR}/i_timer.c
        ${CMAKE_CURRENT_LIST_DIR}/i_video.c
        ${CMAKE_CURRENT_LIST_DIR}/stubs.c

        ${CMAKE_CURRENT_LIST_DIR}/i_picosound.c

        ${CMAKE_CURRENT_LIST_DIR}/controls.c

        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screens/shared.c
        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screen.c
        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screens/lilygo_ttgo.c
        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screens/st7789_240_135.c
        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screens/ili9341_mcu_if.c
        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screens/ssd1306_70_40.cpp
        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screens/ssd1306_70_40_i2c.c
        ${CMAKE_CURRENT_LIST_DIR}/pico-screens/screens/st7735_128_128.c
)
if (PICO_ON_DEVICE)
    target_sources(common_pico INTERFACE
            ${CMAKE_CURRENT_LIST_DIR}/blit.S
            ${CMAKE_CURRENT_LIST_DIR}/picoflash.c
            )
    pico_wrap_function(common_pico malloc)
    pico_wrap_function(common_pico calloc)
    pico_wrap_function(common_pico free)
    target_compile_definitions(common_pico INTERFACE
            PICO_HEAP_SIZE=0
            USE_ZONE_FOR_MALLOC=1
    )
    target_link_libraries(common_pico INTERFACE  hardware_i2c)
endif()
target_include_directories(common_pico INTERFACE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/..
        ${CMAKE_CURRENT_LIST_DIR}/../../textscreen
        )

target_compile_definitions(common_pico INTERFACE
        NO_USE_MOUSE=1
        PICO_AUDIO_PWM_PIO=1
        PICO_AUDIO_PWM_DMA_IRQ=1
        )

target_link_libraries(common_pico INTERFACE pico_stdlib pico_audio_pwm2 pico_multicore pico_scanvideo_dpi hardware_adc)

add_library(pico_cd INTERFACE)
if (TARGET tinyusb_host)
    target_link_libraries(pico_cd INTERFACE tinyusb_host)
endif()
