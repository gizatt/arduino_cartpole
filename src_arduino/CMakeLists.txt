cmake_minimum_required(VERSION 3.5)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake)
message("Arduino SDK path " ${ARDUINO_SDK_PATH})

project(cartpole_src_arduino)

include_directories(${CMAKE_SOURCE_DIR})

generate_arduino_library(zcm_c
        SRCS zcm/zcm.c zcm/nonblocking.c zcm/transport/generic_serial_transport.c
        HDRS zcm/zcm.h zcm/zcm_coretypes.h zcm/nonblocking.h zcm/transport.h zcm/zcm_private.h zcm/transport/generic_serial_transport.h
        BOARD uno)

generate_arduino_firmware(cartpole_plane
        SRCS cartpole_plant.cpp time_utils.h
        LIBS zcm_c 
        BOARD uno
        PORT /dev/cartpole_uno
        )

