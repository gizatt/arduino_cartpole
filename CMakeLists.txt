cmake_minimum_required(VERSION 3.5)
project(cartpole)

# Set default build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "Release" CACHE STRING
    "The type of build. Options are: Debug Release RelWithDebInfo MinSizeRel."
    FORCE)
endif()

# Build shared libraries by default
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

# Set default install prefix
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX "${CMAKE_BINARY_DIR}/install" CACHE STRING
    "Prefix for installation of sub-packages (note: required during build!)"
    FORCE)
endif()

# Options
# Setup external projects
include(ExternalProject)
set_property(DIRECTORY PROPERTY EP_PREFIX ${CMAKE_BINARY_DIR})
set(source_dir ${CMAKE_SOURCE_DIR})
set(build_dir ${CMAKE_BINARY_DIR})

set(CMAKE_ARGS_FOR_EXTERNALS
  -DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DCMAKE_C_COMPILER_LAUNCHER:FILEPATH=${CMAKE_C_COMPILER_LAUNCHER}"
  "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
  "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}"
  "-DCMAKE_CXX_COMPILER_LAUNCHER:FILEPATH=${CMAKE_CXX_COMPILER_LAUNCHER}"
  "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
  "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
  "-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${CMAKE_INSTALL_PREFIX}"
  "-DCMAKE_PREFIX_PATH:PATH=${CMAKE_INSTALL_PREFIX}"
  "-DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}"
  -DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}
)

if (0)
  ExternalProject_Add(zcm
    SOURCE_DIR ${build_dir}/externals/zcm
    BINARY_DIR ${build_dir}/zcm
    GIT_REPOSITORY https://github.com/ZeroCM/zcm
    GIT_TAG f7ba28ef8d16183065a3296d25189c78142b50c4
    BUILD_ALWAYS 1
    CONFIGURE_COMMAND cd ${build_dir}/externals/zcm/ && ${build_dir}/externals/zcm/waf configure --use-serial --use-udpm --prefix=${build_dir}
    BUILD_COMMAND cd ${build_dir}/externals/zcm/ && ${build_dir}/externals/zcm/waf
    INSTALL_COMMAND cd ${build_dir}/externals/zcm/ && ${build_dir}/externals/zcm/waf install
  )
endif()

set(ARDUINO_SDK_PATH ${build_dir}/src/arduino)
ExternalProject_Add(arduino
  URL https://downloads.arduino.cc/arduino-1.8.5-linux64.tar.xz
  URL_MD5 a95fa15b88b8dc4ddb79ac28665f763b
  CONFIGURE_COMMAND ""
  BUILD_COMMAND ""
  INSTALL_COMMAND ""
)


ExternalProject_Add(src_arduino
  SOURCE_DIR ${source_dir}/src_arduino
  BINARY_DIR ${build_dir}/src_arduino
  BUILD_ALWAYS 1
  CMAKE_CACHE_ARGS
    ${CMAKE_ARGS_FOR_EXTERNALS}
    -DARDUINO_SDK_PATH:STRING=${ARDUINO_SDK_PATH}
  INSTALL_COMMAND ""
  DEPENDS arduino
)