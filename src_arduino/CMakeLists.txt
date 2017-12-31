cmake_minimum_required(VERSION 3.5)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/arduino-cmake/cmake/ArduinoToolchain.cmake)
message("Arduino SDK path " ${ARDUINO_SDK_PATH})

project(cartpole_src_arduino)

generate_arduino_firmware(blink
        SRCS blink.cpp
        BOARD uno
        PORT /dev/ttyACM0
        )