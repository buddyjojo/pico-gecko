# SPDX-License-Identifier: MIT

cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(pico_gecko C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()
  
add_executable(pico_gecko pico_gecko.c usb-descriptors.c)

pico_generate_pio_header(pico_gecko ${CMAKE_CURRENT_LIST_DIR}/exirx.pio)
pico_generate_pio_header(pico_gecko ${CMAKE_CURRENT_LIST_DIR}/exitx.pio)

target_include_directories(pico_gecko PUBLIC
	./
	../pico-sdk/lib/tinyusb/src)

target_link_libraries(pico_gecko
	pico_multicore
	pico_stdlib
	hardware_flash
	tinyusb_device
	tinyusb_board
        hardware_pio)
        
pico_add_extra_outputs(pico_gecko)
