cmake_minimum_required(VERSION 3.12)

# Pull in SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_linbus_motor_tester C CXX ASM)

# Initialize the SDK
pico_sdk_init()

add_executable(linbus_motor_tester
    linbus_motor_tester.c
)

# Pull in common dependencies
target_link_libraries(linbus_motor_tester
    pico_stdlib
    pico_time
    hardware_uart
    hardware_gpio
)

# Enable usb output, disable uart output
pico_enable_stdio_usb(linbus_motor_tester 1)
pico_enable_stdio_uart(linbus_motor_tester 1)

# Create map/bin/hex/uf2 file etc.
pico_add_extra_outputs(linbus_motor_tester)
