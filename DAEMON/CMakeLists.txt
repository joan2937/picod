cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)

project(picod)

pico_sdk_init()

add_executable(picod picod.c)

pico_enable_stdio_usb(picod 1)
#pico_enable_stdio_uart(picod 0)

pico_add_extra_outputs(picod)

target_link_libraries(picod pico_stdio pico_stdlib pico_sync pico_multicore pico_unique_id hardware_adc hardware_pwm hardware_spi hardware_i2c hardware_pio hardware_dma hardware_uart)

