# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.13.1)
set(BOARD nucleo_l432kc)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(stm32_pm_blinky)

## This project
target_sources(app PRIVATE src/project/main.cpp)

# SPI
# target_sources(app PRIVATE src/spi/transceive_master_double.cpp)
# target_sources(app PRIVATE src/spi/transceive_slave_double.cpp)

# UART
# target_sources(app PRIVATE src/uart/uart_send.cpp)

target_include_directories(app PUBLIC external/PX4-Matrix)

target_include_directories(app PUBLIC 
external/PX4-Matrix
src/communication
)

target_link_libraries(app PRIVATE
model_lib
)


add_subdirectory(src/simulator)
