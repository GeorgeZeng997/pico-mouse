# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

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

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(dev_hid_composite C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

cmake_minimum_required(VERSION 3.13)

add_executable(dev_hid_composite
    main.c
    usb_descriptors.c
    mpu9250_spi.c
    # hello_usb.c  // 如果你有 hello_usb.c 也可以加进来
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(dev_hid_composite PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(dev_hid_composite
    pico_stdlib
    pico_unique_id
    pico_multicore
    tinyusb_device
    tinyusb_board
    hardware_spi
)

# 如果你需要自定义VID/PID，可以这样设置（可选）
target_compile_definitions(dev_hid_composite PRIVATE
    CFG_TUD_VENDOR_ID=0x046D
    CFG_TUD_PRODUCT_ID=0xC077
)

pico_add_extra_outputs(dev_hid_composite)

# add url via pico_set_program_url
