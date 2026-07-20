cmake_minimum_required(VERSION 3.20)
project(arm_controls_native)
set(ARM_CONTROLS_TARGET arm_controls_node)

# Boost: avoid deprecated FindBoost (CMake >= 3.30 warns; CMP0167 NEW uses BoostConfig.cmake).
if(POLICY CMP0167)
  cmake_policy(SET CMP0167 NEW)
endif()

# C++17 is required by the native control implementation.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Export compile commands for clangd and other tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

add_compile_definitions(ENABLE_ALGO_PINO)
add_compile_options(-Wall -Wextra -Wpedantic)
# -Wno-psabi is GCC-only (silences the ARM struct-passing ABI warning).
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
  add_compile_options(-Wno-psabi)
endif()

# Find bundled shared libraries next to the packaged Linux executable.
set(CMAKE_INSTALL_RPATH "$ORIGIN/../libs")

# Boost.System is header-only on supported Linux distributions.
find_package(Boost CONFIG REQUIRED COMPONENTS program_options)
find_package(PkgConfig REQUIRED)
find_package(Eigen3 REQUIRED)
pkg_check_modules(PC_ZMQ REQUIRED libzmq)
pkg_check_modules(PINOCCHIO REQUIRED pinocchio)

# Let CMake's runtime dependency scanner resolve local build dependencies
# without hard-coding those directories into the installed executable.
set(CMAKE_BUILD_RPATH ${PINOCCHIO_LIBRARY_DIRS} ${PC_ZMQ_LIBRARY_DIRS})

# Include directories
include_directories(
  include
  /usr/local/include
  ${Boost_INCLUDE_DIRS}
  ${PC_ZMQ_INCLUDE_DIRS}
  ${ARM_CONTROLS_CPPZMQ_INCLUDE_DIR}
  ${PINOCCHIO_INCLUDE_DIRS}
)

link_directories(/usr/local/lib)

# pkg-config gives -L in *_LIBRARY_DIRS; plain -l flags in target_link_libraries need those dirs here.
if(PINOCCHIO_LIBRARY_DIRS)
  link_directories(${PINOCCHIO_LIBRARY_DIRS})
endif()
if(PC_ZMQ_LIBRARY_DIRS)
  link_directories(${PC_ZMQ_LIBRARY_DIRS})
endif()

# Define the common source files
set(COMMON_SOURCES
  src/can/math_ops.cpp
  src/arm_controls_node.cpp
  src/arm_controls_command_line_args.cpp
  src/arm_controls_info.cpp
  src/arm_controls_topic.cpp
  src/arm_controls_topic_zmq.cpp
  src/arm_controls_driver.cpp
  src/arm_controls_driver_arx.cpp
  src/arm_controls_driver_can.cpp
  src/arm_controls_joint.cpp
  src/arm_controls_servo.cpp
  src/arm_controls_servo_dm.cpp
  src/arm_controls_servo_dm_status.cpp
  src/arm_controls_servo_can_encoder.cpp
  src/arm_controls_device.cpp
  src/arm_controls_device_arm.cpp
  src/arm_controls_device_arm_arx.cpp
  src/arm_controls_device_effector.cpp
  src/arm_controls_device_effector_arx.cpp
  src/arm_controls_device_config.cpp
  src/arm_controls_algo.cpp
  src/arm_controls_algo_pino.cpp
  src/arm_controls_stall_detect.cpp
  src/arm_controls_force_feedback.cpp
  src/arm_controls_filter.cpp
  src/arm_controls_current_estimation.cpp
  src/arm_controls_hold_checker.cpp
  src/arm_controls_safe_mode.cpp

)

add_executable(${ARM_CONTROLS_TARGET} ${COMMON_SOURCES})

target_link_libraries(${ARM_CONTROLS_TARGET}
  ${PC_ZMQ_LDFLAGS}
  ${PINOCCHIO_LDFLAGS}
  Boost::program_options
)

# Use a transitive DT_RPATH instead of DT_RUNPATH. Libraries copied beside the
# executable by the runtime-dependency install rule have no reason to know the
# wheel layout themselves; the node's $ORIGIN/../libs path must therefore also
# apply to their dependencies.
target_link_options(${ARM_CONTROLS_TARGET} PRIVATE -Wl,--disable-new-dtags)

if(ARM_CONTROLS_BUILD_TESTING)
  if(NOT TARGET GTest::gtest_main)
    find_package(GTest REQUIRED)
  endif()
  include(GoogleTest)

  set(ARM_CONTROLS_TOPIC_ZMQ_TEST_SOURCES ${COMMON_SOURCES})
  list(REMOVE_ITEM ARM_CONTROLS_TOPIC_ZMQ_TEST_SOURCES src/arm_controls_node.cpp)

  add_executable(arm_controls_topic_zmq_tests
    tests/test_driver_arx.cpp
    tests/test_topic_zmq.cpp
    ${ARM_CONTROLS_TOPIC_ZMQ_TEST_SOURCES}
  )
  target_compile_definitions(arm_controls_topic_zmq_tests PRIVATE ARM_CONTROLS_SOURCE_DIR="${CMAKE_SOURCE_DIR}")
  target_link_libraries(arm_controls_topic_zmq_tests
    GTest::gtest_main
    ${PC_ZMQ_LDFLAGS}
    ${PINOCCHIO_LDFLAGS}
    Boost::program_options
  )
  gtest_discover_tests(
    arm_controls_topic_zmq_tests
    DISCOVERY_TIMEOUT 60
    PROPERTIES LABELS native-integration)
endif()

install(TARGETS ${ARM_CONTROLS_TARGET}
  RUNTIME_DEPENDENCY_SET arm_controls_runtime_dependencies
  RUNTIME DESTINATION arm_controls/bin)

# Ship the node's complete non-system shared-library closure. The executable's
# $ORIGIN/../libs RPATH above resolves these files after wheel installation;
# a binary consumer therefore does not need Pinocchio, ZeroMQ, or Boost from
# apt. Keep the Ubuntu base runtime libraries external.
install(RUNTIME_DEPENDENCY_SET arm_controls_runtime_dependencies
  PRE_EXCLUDE_REGEXES
    "^linux-vdso\\.so.*"
    "^ld-linux.*"
  POST_EXCLUDE_REGEXES
    ".*/libc\\.so\\..*"
    ".*/libdl\\.so\\..*"
    ".*/libgcc_s\\.so\\..*"
    ".*/libm\\.so\\..*"
    ".*/libpthread\\.so\\..*"
    ".*/libresolv\\.so\\..*"
    ".*/librt\\.so\\..*"
    ".*/libstdc\\+\\+\\.so\\..*"
  LIBRARY DESTINATION arm_controls/libs
  RUNTIME DESTINATION arm_controls/libs)

# libFuzzer harnesses for the wire-facing CAN status parsers. Compiles the
# runtime sources (minus the node's main) with fuzzer + ASan instrumentation so
# arbitrary CAN frames can be replayed through the reception-thread parsers.
option(ARM_CONTROLS_BUILD_FUZZERS "Build libFuzzer harnesses (requires Clang)" OFF)
if(ARM_CONTROLS_BUILD_FUZZERS)
  if(NOT CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    message(FATAL_ERROR "ARM_CONTROLS_BUILD_FUZZERS requires the Clang compiler")
  endif()

  set(FUZZ_RUNTIME_SOURCES ${COMMON_SOURCES})
  list(REMOVE_ITEM FUZZ_RUNTIME_SOURCES src/arm_controls_node.cpp)

  foreach(fuzz_name fuzz_servo_status_parsers)
    add_executable(${fuzz_name}
      tests/fuzz/${fuzz_name}.cpp
      ${FUZZ_RUNTIME_SOURCES}
    )
    target_compile_options(${fuzz_name} PRIVATE
      -fsanitize=fuzzer,address -fno-omit-frame-pointer -g -O1)
    target_link_options(${fuzz_name} PRIVATE -fsanitize=fuzzer,address)
    target_link_libraries(${fuzz_name}
      ${PC_ZMQ_LDFLAGS}
      ${PINOCCHIO_LDFLAGS}
      Boost::program_options
    )
  endforeach()
endif()
