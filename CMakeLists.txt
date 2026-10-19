cmake_minimum_required(VERSION 3.13)

# Pico W is required for the WiFi test; the other tests run on it too.
set(PICO_BOARD pico_w CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(p5_is34_bringup C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

# Common warnings for every test.
add_compile_options(-Wall -Wextra -Wno-unused-parameter)

add_subdirectory(tests/parts_check)
add_subdirectory(tests/notify)
add_subdirectory(tests/encoder)
add_subdirectory(tests/ir)
add_subdirectory(tests/imu)
add_subdirectory(tests/ultrasonic)

# WiFi: the prompt version always builds; the compiled-in version only
# when tests/wifi_mqtt/wifi_config.h exists (never committed).
add_subdirectory(tests/wifi_mqtt)
if (NOT EXISTS ${CMAKE_CURRENT_LIST_DIR}/tests/wifi_mqtt/wifi_config.h)
    message(STATUS
        "wifi_config.h not found - building only wifi_mqtt_test_prompt "
        "(asks for WiFi settings over USB serial).")
endif()
