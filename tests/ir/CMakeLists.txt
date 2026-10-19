add_executable(ir_test ir_test.c)
target_link_libraries(ir_test pico_stdlib hardware_sync)
# USB only: GP0/GP1 are IR_LEFT/IR_CENTER, not UART0.
pico_enable_stdio_usb(ir_test 1)
pico_enable_stdio_uart(ir_test 0)
pico_add_extra_outputs(ir_test)
