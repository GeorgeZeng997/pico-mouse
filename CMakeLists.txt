cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.1.1)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.1.1)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico CACHE STRING "Board type")

include(pico_sdk_import.cmake)

project(dev_hid_composite C CXX ASM)

pico_sdk_init()

add_executable(dev_hid_composite
    src/main.c
    src/paw3205.c
    src/mouse_buttons.c
    src/cdc_protocol.c
    src/usb_descriptors.c
)

target_include_directories(dev_hid_composite PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/inc
)

target_compile_options(dev_hid_composite PRIVATE
    -Wall
    -Wextra
)

target_link_libraries(dev_hid_composite
    pico_stdlib
    pico_multicore
    pico_sync
    pico_unique_id
    tinyusb_device
    tinyusb_board
    hardware_gpio
)

pico_add_extra_outputs(dev_hid_composite)
