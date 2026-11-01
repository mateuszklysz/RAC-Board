# SPDX-License-Identifier: MIT
# RAC-Board (README.md): Jak II's JET-Board in Ratchet & Clank, a mod for OpenRAC's native port.
#
#   python3 tools/fetch_sdk.py      # the SDK sdk.lock pins, into .sdk/
#   cmake -S . -B build -G Ninja
#   cmake --build build
#   ctest --test-dir build
cmake_minimum_required(VERSION 3.20)
project(rac_board LANGUAGES C CXX)
include(CTest)

# The SDK: the release sdk.lock pins (tools/fetch_sdk.py puts it in .sdk/); OPENRAC_SDK=<dir> (an
# unpacked SDK, or sdk/ in a checkout of OpenRAC) or OpenRACMod_DIR=<dir>/cmake for another.
if(NOT OpenRACMod_DIR)
  if(DEFINED OPENRAC_SDK)
    set(OpenRACMod_DIR "${OPENRAC_SDK}/cmake")
  elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/sdk.lock")
    file(READ "${CMAKE_CURRENT_SOURCE_DIR}/sdk.lock" lock)
    string(JSON sdk_version GET "${lock}" version)
    set(OpenRACMod_DIR "${CMAKE_CURRENT_SOURCE_DIR}/.sdk/openrac-mod-sdk-${sdk_version}/cmake")
    if(NOT EXISTS "${OpenRACMod_DIR}/OpenRACModConfig.cmake")
      message(FATAL_ERROR "No OpenRAC mod SDK ${sdk_version}: run python3 tools/fetch_sdk.py first "
                          "(or pass -DOPENRAC_SDK=<an unpacked SDK>)")
    endif()
  endif()
endif()
find_package(OpenRACMod 1.12 REQUIRED)

# Jak II's code: no fused multiply-adds, so every float comes out as OpenGOAL computes it.
set(rac_board_fp_flags)
if(MSVC)
  set(rac_board_fp_flags /fp:precise)
else()
  set(rac_board_fp_flags -ffp-contract=off -fno-fast-math)
endif()

file(GLOB rac_board_core_sources CONFIGURE_DEPENDS src/jak/*.cpp src/board/*.cpp)
add_library(rac_board_core STATIC ${rac_board_core_sources})
target_include_directories(rac_board_core PUBLIC src)
set_target_properties(rac_board_core PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON
                                                POSITION_INDEPENDENT_CODE ON)
target_compile_options(rac_board_core PRIVATE ${rac_board_fp_flags})
if(NOT MSVC)
  target_compile_options(rac_board_core PRIVATE -Wall -Wextra)
endif()

# The mod's library: the adapter over the core.
file(GLOB rac_board_mod_sources CONFIGURE_DEPENDS src/rac1/*.cpp)
openrac_add_mod(rac_board MANIFEST openrac-mod.json SOURCES ${rac_board_mod_sources})
target_link_libraries(rac_board PRIVATE rac_board_core)
set_target_properties(rac_board PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
target_compile_options(rac_board PRIVATE ${rac_board_fp_flags})
# The board's model and sounds (tools/make_board.py, tools/make_sounds.py) and the third-party
# notice go in the mod's folder beside the library: the mod loads them by these paths.
get_target_property(rac_board_package rac_board OPENRAC_MOD_PACKAGE_DIR)
file(COPY assets THIRD_PARTY.md DESTINATION "${rac_board_package}")

# The tests build only when testing is on: one executable per tests/*_test.cpp, each a CTest test.
if(BUILD_TESTING)
  file(GLOB rac_board_tests CONFIGURE_DEPENDS tests/*_test.cpp)
  foreach(test_source ${rac_board_tests})
    get_filename_component(test_name ${test_source} NAME_WE)
    add_executable(rac_board_${test_name} ${test_source})
    target_link_libraries(rac_board_${test_name} PRIVATE rac_board_core)
    target_include_directories(rac_board_${test_name} PRIVATE tests)
    set_target_properties(rac_board_${test_name} PROPERTIES CXX_STANDARD 17 CXX_STANDARD_REQUIRED ON)
    target_compile_options(rac_board_${test_name} PRIVATE ${rac_board_fp_flags})
    add_test(NAME rac_board_${test_name} COMMAND rac_board_${test_name})
  endforeach()
endif()
