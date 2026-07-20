cmake_minimum_required(VERSION 3.22)
project(arm_controls LANGUAGES CXX)

if(NOT CMAKE_SYSTEM_NAME STREQUAL "Linux")
  message(FATAL_ERROR "arm_controls requires Linux and SocketCAN")
endif()

option(ARM_CONTROLS_BUILD_NATIVE "Build the physical-arm native runtime" ON)
option(ARM_CONTROLS_BUILD_TESTING "Build the native unit tests" OFF)

# A source build keeps native dependencies private to this project.
# `build_deps.sh` installs the pinned dependencies here, and this CMake
# configuration exposes them without requiring shell activation.
set(_arm_controls_default_deps_dir "${CMAKE_CURRENT_SOURCE_DIR}/.deps")
if(DEFINED ENV{ARM_CONTROLS_DEPS_DIR})
  set(_arm_controls_default_deps_dir "$ENV{ARM_CONTROLS_DEPS_DIR}")
endif()
set(ARM_CONTROLS_DEPS_DIR "${_arm_controls_default_deps_dir}" CACHE PATH
    "Directory containing arm_controls build dependencies")
set(ARM_CONTROLS_PINOCCHIO_PREFIX "${ARM_CONTROLS_DEPS_DIR}/pinocchio/install" CACHE PATH
    "Pinocchio installation prefix used by arm_controls")
set(ARM_CONTROLS_CPPZMQ_PREFIX "${ARM_CONTROLS_DEPS_DIR}/cppzmq/install" CACHE PATH
    "cppzmq installation prefix used by arm_controls")

if(EXISTS "${ARM_CONTROLS_PINOCCHIO_PREFIX}/lib/pkgconfig/pinocchio.pc")
  list(PREPEND CMAKE_PREFIX_PATH "${ARM_CONTROLS_PINOCCHIO_PREFIX}")
  if(DEFINED ENV{PKG_CONFIG_PATH} AND NOT "$ENV{PKG_CONFIG_PATH}" STREQUAL "")
    set(ENV{PKG_CONFIG_PATH}
        "${ARM_CONTROLS_PINOCCHIO_PREFIX}/lib/pkgconfig:$ENV{PKG_CONFIG_PATH}")
  else()
    set(ENV{PKG_CONFIG_PATH} "${ARM_CONTROLS_PINOCCHIO_PREFIX}/lib/pkgconfig")
  endif()
endif()

if(EXISTS "${ARM_CONTROLS_CPPZMQ_PREFIX}/include/zmq.hpp")
  list(PREPEND CMAKE_PREFIX_PATH "${ARM_CONTROLS_CPPZMQ_PREFIX}")
  set(ARM_CONTROLS_CPPZMQ_INCLUDE_DIR "${ARM_CONTROLS_CPPZMQ_PREFIX}/include")
endif()

# Sanitizer plumbing for the test targets. Valid values: "" (off), "address,undefined",
# "thread". Applied globally so the exercised arm_controls sources are instrumented too.
set(ARM_CONTROLS_SANITIZER "" CACHE STRING "Comma-separated -fsanitize= list for test builds")
if(ARM_CONTROLS_SANITIZER)
  add_compile_options(-fsanitize=${ARM_CONTROLS_SANITIZER} -fno-sanitize-recover=all -g -fno-omit-frame-pointer)
  add_link_options(-fsanitize=${ARM_CONTROLS_SANITIZER})
endif()

if(ARM_CONTROLS_BUILD_TESTING)
  enable_testing()
  add_subdirectory(native/arm_controls/tests)
endif()

if(ARM_CONTROLS_BUILD_NATIVE)
  set(NO_ROS ON CACHE BOOL "Build without ROS" FORCE)
  add_subdirectory(native/arm_controls)
endif()
