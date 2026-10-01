# This is a copy of <PICO_SDK_PATH>/external/pico_sdk_import.cmake

# This should be added to the top of your CMakeLists.txt

if (DEFINED ENV{PICO_SDK_PATH})
    set(PICO_SDK_PATH $ENV{PICO_SDK_PATH})
    message("Using PICO_SDK_PATH from environment ('${PICO_SDK_PATH}')")
endif ()

if (NOT PICO_SDK_PATH)
    message(FATAL_ERROR "PICO_SDK_PATH environment variable not set")
endif ()

include("${PICO_SDK_PATH}/pico_sdk_init.cmake")
