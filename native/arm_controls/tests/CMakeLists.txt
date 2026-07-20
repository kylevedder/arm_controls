# Unit and SocketCAN transport tests that do not require Pinocchio or ZMQ.
# The project is Linux-only; syscall wrappers exercise the transport without
# opening physical CAN hardware.
include(FetchContent)
set(_googletest_find_package_args)
if(CMAKE_VERSION VERSION_GREATER_EQUAL 3.24)
  list(APPEND _googletest_find_package_args FIND_PACKAGE_ARGS NAMES GTest)
endif()
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/releases/download/v1.16.0/googletest-1.16.0.tar.gz
  URL_HASH SHA256=78c676fc63881529bf97bf9d45948d905a66833fbfa5318ea2cd7478cb98f399
  ${_googletest_find_package_args}
)
unset(_googletest_find_package_args)
FetchContent_MakeAvailable(googletest)

set(ARM_CONTROLS_ROOT ${CMAKE_CURRENT_SOURCE_DIR}/..)

# Boost is only needed for header-only circular_buffer. Prefer the CMake config
# (Homebrew, apt libboost-*-dev) but fall back to a bare include-path probe so a
# headers-only install is enough.
find_package(Boost CONFIG QUIET)
if(NOT Boost_FOUND)
  find_path(ARM_CONTROLS_BOOST_INCLUDE_DIR boost/circular_buffer.hpp REQUIRED)
endif()

add_executable(arm_controls_tests
  test_math_ops.cpp
  test_filter.cpp
  test_stall_detect.cpp
  test_hold_checker.cpp
  test_current_estimation.cpp
  test_device_config.cpp
  test_driver_can.cpp
  test_posix_syscall_hooks.cpp
  test_servo_dm_status.cpp
  test_driver_base.cpp
  ${ARM_CONTROLS_ROOT}/src/can/math_ops.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_filter.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_stall_detect.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_current_estimation.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_hold_checker.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_device_config.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_driver_can.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_servo_dm_status.cpp
  ${ARM_CONTROLS_ROOT}/src/arm_controls_info.cpp
)
target_include_directories(arm_controls_tests PRIVATE ${ARM_CONTROLS_ROOT}/include)
target_compile_features(arm_controls_tests PRIVATE cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  target_compile_options(arm_controls_tests PRIVATE -Wall -Wextra)
endif()
if(TARGET Boost::headers)
  target_link_libraries(arm_controls_tests PRIVATE Boost::headers)
else()
  target_include_directories(arm_controls_tests PRIVATE ${ARM_CONTROLS_BOOST_INCLUDE_DIR})
endif()

target_link_libraries(arm_controls_tests PRIVATE GTest::gtest_main)

if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
  target_link_options(arm_controls_tests PRIVATE
    -Wl,--wrap=socket
    -Wl,--wrap=ioctl
    -Wl,--wrap=bind
    -Wl,--wrap=setsockopt
    -Wl,--wrap=close
    -Wl,--wrap=select
    -Wl,--wrap=write
    -Wl,--wrap=read)
endif()

include(GoogleTest)
gtest_discover_tests(arm_controls_tests DISCOVERY_TIMEOUT 60)
