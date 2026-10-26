add_executable(parts_check parts_check.c ${CMAKE_CURRENT_LIST_DIR}/../common/notify_hw.c)
pico_generate_pio_header(parts_check ${CMAKE_CURRENT_LIST_DIR}/../common/ws2812.pio)
target_include_directories(parts_check PRIVATE ${CMAKE_CURRENT_LIST_DIR}/../common)
target_link_libraries(parts_check pico_stdlib hardware_pio hardware_pwm hardware_clocks
                      hardware_i2c hardware_adc)
pico_enable_stdio_usb(parts_check 1)
pico_enable_stdio_uart(parts_check 0)
pico_add_extra_outputs(parts_check)
