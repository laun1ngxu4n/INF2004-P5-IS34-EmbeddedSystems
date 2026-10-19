# Two images from the same source:
#   wifi_mqtt_test         credentials compiled in from wifi_config.h (git-ignored)
#   wifi_mqtt_test_prompt  asks for SSID / password / broker IP over USB serial
#                          at start-up, so it can be shared prebuilt
# Poll mode: lwIP/CYW43 only run when cyw43_arch_poll() is called, which
# matches the design (Telemetry task polls every 10 ms).
# Debug over USB only: GP0/GP1 (UART0) are the line sensors.

function(p5_wifi_target name)
    add_executable(${name} wifi_mqtt_test.c cmd_json.c)
    target_include_directories(${name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})
    target_link_libraries(${name}
        pico_stdlib
        pico_cyw43_arch_lwip_poll
        pico_lwip_mqtt
        pico_rand
    )
    pico_enable_stdio_usb(${name} 1)
    pico_enable_stdio_uart(${name} 0)
    pico_add_extra_outputs(${name})
endfunction()

p5_wifi_target(wifi_mqtt_test_prompt)
target_compile_definitions(wifi_mqtt_test_prompt PRIVATE WIFI_RUNTIME_CONFIG=1)

if (EXISTS ${CMAKE_CURRENT_LIST_DIR}/wifi_config.h)
    p5_wifi_target(wifi_mqtt_test)
endif()
